// perception_experiment: LiDAR perception problems and their fixes, measured.
//
//   perception_experiment ekf [options]       EKF consistency and accuracy when range
//                                             errors are correlated (a range bias, or a
//                                             map that is only a rasterization of the
//                                             real walls), for each measurement model.
//   perception_experiment grazing [options]   the grazing-beam cutoff swept from 45 to
//                                             90 deg, against the grazing model.
//   perception_experiment dynamic [options]   moving obstacles seen through the LiDAR:
//                                             oracle sightings vs. map subtraction and
//                                             tracking, with and without learned
//                                             reversals and occlusion awareness.
//
// Worlds (ekf, grazing):
//   grid        the map is the truth (the simulator's usual world)
//   continuous  the obstacles map's blocks as polygons, each shifted by up to
//               0.45 cells and rotated by up to --rotation deg; the outer walls shifted
//               and tilted too. The robot's map is the rasterization of that world
//               (majority rule) at --resolution: a map as good as a grid can be.
//
// Options: --runs N, --seed N, --beams N, --noise M (white range noise), --bias M and
//          --bias-time S (LiDAR range bias), --world grid|continuous, --resolution M,
//          --rotation DEG, --output DIR (default results/perception).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "alignment.hpp"
#include "astar.hpp"
#include "ekf.hpp"
#include "footprint.hpp"
#include "geometry2d.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "obstacle_tracking.hpp"
#include "simulator.hpp"
#include "dynamic_obstacles.hpp"
#include "diff_drive.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kChi2Bound3 = 7.815;  // 95 % quantile of chi^2 with 3 dof

struct Options {
    std::string mode;
    int runs = 40;
    unsigned seed = 1;
    int beams = 0;  ///< 0: 72 (ekf, grazing) or 360 (dynamic)
    double noise = 0.02;
    double bias = 0.0;
    double bias_time = 0.0;
    std::string world = "continuous";
    double resolution = 0.5;
    double rotation = 6.0;
    std::filesystem::path output = "results/perception";
    bool sweep = false;
    bool lines = false;  ///< maperror: measure against the wall-line surface
    bool trace = false;
    double min_speed = 0.2, max_speed = 0.5;  ///< dynamic: obstacle speeds, m/s  ///< dynamic: print every failed run as an obstacle file  ///< ekf: also sweep the wall-state prior (offset and tilt std)
};

// ---------------------------------------------------------------------------------
// Worlds

/// The obstacles map's blocks (connected components, all rectangles) as boxes, m.
std::vector<amr::Box> mapBlocks(const amr::GridMap& map) {
    std::vector<amr::Box> boxes;
    std::vector<bool> seen(map.cellCount(), false);
    const double r = map.resolution();
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(map.width()) +
                                  static_cast<std::size_t>(x);
            if (seen[i] || !map.isOccupied({x, y})) {
                continue;
            }
            std::vector<amr::GridCoord> stack{{x, y}};
            seen[i] = true;
            int x0 = x, x1 = x, y0 = y, y1 = y;
            while (!stack.empty()) {
                const amr::GridCoord c = stack.back();
                stack.pop_back();
                x0 = std::min(x0, c.x);
                x1 = std::max(x1, c.x);
                y0 = std::min(y0, c.y);
                y1 = std::max(y1, c.y);
                const amr::GridCoord steps[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (const amr::GridCoord& d : steps) {
                    const amr::GridCoord n{c.x + d.x, c.y + d.y};
                    if (!map.inBounds(n) || !map.isOccupied(n)) {
                        continue;
                    }
                    const std::size_t j = static_cast<std::size_t>(n.y) * static_cast<std::size_t>(map.width()) +
                                          static_cast<std::size_t>(n.x);
                    if (!seen[j]) {
                        seen[j] = true;
                        stack.push_back(n);
                    }
                }
            }
            boxes.push_back({x0 * r, y0 * r, (x1 + 1) * r, (y1 + 1) * r});
        }
    }
    return boxes;
}

/// A world the robot moves in: a map it knows, and the truth its LiDAR sees.
struct World {
    amr::GridMap map;                 // what the robot knows (planning, EKF)
    std::vector<amr::Polygon> truth;  // empty: the map is the truth
};

World makeWorld(const amr::GridMap& layout, const Options& o, std::mt19937& rng) {
    if (o.world == "grid") {
        return {layout, {}};
    }
    std::uniform_real_distribution<double> shift(-0.45 * o.resolution, 0.45 * o.resolution);
    std::uniform_real_distribution<double> turn(-o.rotation * kPi / 180.0, o.rotation * kPi / 180.0);
    std::vector<amr::Polygon> polygons;
    for (const amr::Box& b : mapBlocks(layout)) {
        const amr::WorldPoint center{0.5 * (b.x0 + b.x1) + shift(rng), 0.5 * (b.y0 + b.y1) + shift(rng)};
        polygons.push_back(amr::makeRectangle(center, b.x1 - b.x0, b.y1 - b.y0, turn(rng)));
    }
    // Outer walls: 2 m thick, inner faces shifted by up to 0.45 cells and tilted by up
    // to a degree.
    const double w = layout.width() * layout.resolution();
    const double h = layout.height() * layout.resolution();
    std::uniform_real_distribution<double> tilt(-kPi / 180.0, kPi / 180.0);
    polygons.push_back(amr::makeRectangle({-1.0 + shift(rng), h / 2}, 2.0, h + 4.0, tilt(rng)));
    polygons.push_back(amr::makeRectangle({w + 1.0 + shift(rng), h / 2}, 2.0, h + 4.0, tilt(rng)));
    polygons.push_back(amr::makeRectangle({w / 2, -1.0 + shift(rng)}, w + 4.0, 2.0, tilt(rng)));
    polygons.push_back(amr::makeRectangle({w / 2, h + 1.0 + shift(rng)}, w + 4.0, 2.0, tilt(rng)));
    const int cells_x = static_cast<int>(std::lround(w / o.resolution));
    const int cells_y = static_cast<int>(std::lround(h / o.resolution));
    return {amr::rasterize(polygons, cells_x, cells_y, o.resolution, amr::RasterRule::Majority),
            polygons};
}

double clearanceTo(const std::vector<amr::Polygon>& polygons, amr::WorldPoint p) {
    double best = 1e9;
    for (const amr::Polygon& poly : polygons) {
        best = std::min(best, amr::distanceToPolygon(poly, p));
    }
    return best;
}

/// A random route (diff-drive, footprint 0.3 m) of at least 6 m on the world's map,
/// whose trajectory keeps 5 cm from the true walls. Empty if none was found.
std::vector<amr::TrajectorySample> randomRoute(const World& world, std::mt19937& rng) {
    const amr::GridMap& map = world.map;
    const amr::GridMap inflated = amr::inflateObstacles(map, 0.3);
    std::uniform_int_distribution<int> xs(0, map.width() - 1);
    std::uniform_int_distribution<int> ys(0, map.height() - 1);
    for (int attempt = 0; attempt < 200; ++attempt) {
        const amr::GridCoord start{xs(rng), ys(rng)};
        const amr::GridCoord goal{xs(rng), ys(rng)};
        if (!inflated.isFree(start) || !inflated.isFree(goal)) {
            continue;
        }
        const amr::WorldPoint a = map.gridToWorld(start);
        const amr::WorldPoint b = map.gridToWorld(goal);
        if (std::hypot(a.x - b.x, a.y - b.y) < 6.0) {
            continue;
        }
        const amr::PlanResult plan = amr::planWithFootprint(map, inflated, start, goal);
        if (!plan.success()) {
            continue;
        }
        amr::MotionConfig motion;
        motion.model = amr::RobotModel::DiffDrive;
        motion.robot_radius = 0.3;
        motion.initial_heading = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
        const amr::SimulationResult sim = amr::simulatePath(map, plan.path, motion);
        if (!sim.success()) {
            continue;
        }
        bool clear = true;
        if (!world.truth.empty()) {
            for (const auto& s : sim.trajectory) {
                if (clearanceTo(world.truth, {s.state.x, s.state.y}) < 0.05) {
                    clear = false;
                    break;
                }
            }
        }
        if (clear) {
            return sim.trajectory;
        }
    }
    return {};
}

std::vector<amr::LidarScan> scanWorld(const World& world,
                                      const std::vector<amr::TrajectorySample>& trajectory,
                                      const amr::LidarConfig& lidar) {
    if (world.truth.empty()) {
        return amr::scanTrajectory(world.map, trajectory, lidar);
    }
    return amr::scanTrajectoryWith(
        [&world](amr::WorldPoint origin, double angle, double max_range, double) {
            double best = max_range;
            for (const amr::Polygon& poly : world.truth) {
                best = std::min(best, amr::rayToPolygon(poly, origin, angle));
            }
            return amr::BeamRange{best < max_range, best};
        },
        trajectory, lidar);
}

// ---------------------------------------------------------------------------------
// EKF variants

struct Variant {
    std::string name;
    amr::EkfConfig config;
    int subsample = 1;  ///< use every k-th beam only
};

struct Score {
    double nees_sum = 0.0;
    double inside = 0.0;      ///< samples with NEES below the 95 % bound
    double position_sq = 0.0;
    double heading_sq = 0.0;
    double samples = 0.0;
    double used = 0.0;
    double rejected = 0.0;
    double seconds = 0.0;
    double scans = 0.0;
    std::vector<double> run_nees;  ///< mean NEES per run
};

void scoreRun(Score& score, const std::vector<amr::LocalizationSample>& samples, double seconds) {
    double run = 0.0;
    for (const auto& s : samples) {
        const double ex = s.truth.x - s.estimate.x;
        const double ey = s.truth.y - s.estimate.y;
        const double et = amr::wrapAngle(s.truth.theta - s.estimate.theta);
        score.nees_sum += s.nees;
        score.inside += s.nees <= kChi2Bound3 ? 1.0 : 0.0;
        score.position_sq += ex * ex + ey * ey;
        score.heading_sq += et * et;
        score.used += static_cast<double>(s.update.used);
        score.rejected += static_cast<double>(s.update.rejected);
        run += s.nees;
    }
    score.samples += static_cast<double>(samples.size());
    score.scans += static_cast<double>(samples.size());
    score.seconds += seconds;
    score.run_nees.push_back(run / static_cast<double>(samples.size()));
}

double median(std::vector<double> v) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

std::vector<amr::LidarScan> subsampled(const std::vector<amr::LidarScan>& scans, int k,
                                       amr::LidarConfig& lidar) {
    // Keep beams 0, k, 2k, ...; as a scan of beams / k beams over the same field of view.
    if (k <= 1) {
        return scans;
    }
    std::vector<amr::LidarScan> out = scans;
    for (auto& s : out) {
        amr::LidarScan t = s;
        t.angles.clear();
        t.ranges.clear();
        t.hits.clear();
        for (std::size_t i = 0; i < s.ranges.size(); i += static_cast<std::size_t>(k)) {
            t.angles.push_back(s.angles[i]);
            t.ranges.push_back(s.ranges[i]);
            t.hits.push_back(s.hits[i]);
        }
        s = t;
    }
    lidar.beams = static_cast<int>(out.front().ranges.size());
    return out;
}

void writeSummary(const std::vector<Variant>& variants, const std::vector<Score>& scores,
                  const Options& o, const std::string& title, std::ostream& csv) {
    std::cout << title << "\n";
    std::cout << std::left << std::setw(44) << "variant" << std::right << std::setw(10)
              << "mean NEES" << std::setw(10) << "median" << std::setw(10) << "in 95%"
              << std::setw(11) << "pos RMS cm" << std::setw(11) << "hdg RMS deg" << std::setw(9)
              << "beams" << std::setw(10) << "ms/scan" << "\n";
    csv << "variant,mean_nees,median_run_nees,inside_95,position_rms_m,heading_rms_rad,"
           "beams_used_per_scan,beams_rejected_per_scan,ms_per_scan\n";
    for (std::size_t v = 0; v < variants.size(); ++v) {
        const Score& s = scores[v];
        const double mean = s.nees_sum / s.samples;
        const double pos = std::sqrt(s.position_sq / s.samples);
        const double hdg = std::sqrt(s.heading_sq / s.samples);
        std::cout << std::left << std::setw(44) << variants[v].name << std::right << std::fixed
                  << std::setprecision(2) << std::setw(10) << mean << std::setw(10)
                  << median(s.run_nees) << std::setw(9) << std::setprecision(1)
                  << 100.0 * s.inside / s.samples << "%" << std::setw(11) << std::setprecision(2)
                  << 100.0 * pos << std::setw(11) << hdg * 180.0 / kPi << std::setw(9)
                  << std::setprecision(1) << s.used / s.scans << std::setw(10)
                  << std::setprecision(3) << 1000.0 * s.seconds / s.scans << "\n";
        csv << '"' << variants[v].name << '"' << ',' << std::setprecision(6) << mean << ','
            << median(s.run_nees) << ',' << s.inside / s.samples << ',' << pos << ',' << hdg << ','
            << s.used / s.scans << ',' << s.rejected / s.scans << ','
            << 1000.0 * s.seconds / s.scans << "\n";
    }
    (void)o;
}

int runEkf(const Options& o, const std::vector<Variant>& variants, const std::string& name,
           const std::string& title) {
    const amr::MapLoadResult layout = amr::loadGridMapCsv("maps/obstacles.csv", 0.5);
    if (!layout.ok) {
        std::cerr << layout.error << " (run from the repository root)\n";
        return 1;
    }
    std::mt19937 rng(o.seed);
    std::vector<Score> scores(variants.size());
    int done = 0;
    for (int run = 0; run < o.runs; ++run) {
        const World world = makeWorld(layout.map, o, rng);
        const std::vector<amr::TrajectorySample> trajectory = randomRoute(world, rng);
        if (trajectory.empty()) {
            continue;
        }
        amr::LidarConfig lidar;
        lidar.beams = o.beams > 0 ? o.beams : 72;
        lidar.noise_stddev = o.noise;
        lidar.bias_stddev = o.bias;
        lidar.bias_time = o.bias_time;
        lidar.seed = static_cast<std::uint32_t>(rng());
        const std::vector<amr::LidarScan> scans = scanWorld(world, trajectory, lidar);
        const auto odometry_seed = static_cast<std::uint32_t>(rng());
        for (std::size_t v = 0; v < variants.size(); ++v) {
            amr::EkfConfig config = variants[v].config;
            config.enabled = true;
            config.odometry.seed = odometry_seed;
            amr::LidarConfig used = lidar;
            const std::vector<amr::LidarScan> input = subsampled(scans, variants[v].subsample, used);
            const auto t0 = std::chrono::steady_clock::now();
            const auto samples = amr::runLocalization(world.map, trajectory, input, used, config);
            const double seconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            scoreRun(scores[v], samples, seconds);
        }
        ++done;
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / ("perception_" + name + ".csv"));
    std::ostringstream heading;
    heading << title << " (" << done << " runs, world " << o.world << ", " << o.resolution
            << " m cells, " << (o.beams > 0 ? o.beams : 72) << " beams, noise " << o.noise << " m, bias " << o.bias
            << " m)";
    writeSummary(variants, scores, o, heading.str(), csv);
    return 0;
}

amr::EkfConfig base(double sigma) {
    amr::EkfConfig c;
    c.range_sigma = sigma;
    return c;
}

int modeEkf(const Options& o) {
    // Map error of a rasterization: walls move to the nearest cell face, an error
    // uniform over +-res/2 (std res / sqrt 12), plus the staircase of slanted walls.
    const double map_sigma = o.world == "grid" ? 0.0 : o.resolution / std::sqrt(12.0);
    const double inflated = std::sqrt(o.noise * o.noise + map_sigma * map_sigma + o.bias * o.bias);
    std::vector<Variant> v;
    v.push_back({"independent beams (classic)", base(o.noise), 1});
    v.push_back({"independent, sigma inflated", base(inflated), 1});
    v.push_back({"independent, inflated, every 6th beam", base(inflated), 6});
    if (o.bias > 0.0) {
        amr::EkfConfig c = base(o.noise);
        c.model.bias_sigma = o.bias;
        c.model.bias_time = o.bias_time;
        v.push_back({"bias state", c, 1});
    }
    if (map_sigma > 0.0) {
        amr::EkfConfig c = base(o.noise);
        c.model.map_sigma = map_sigma;
        c.model.bias_sigma = o.bias;
        c.model.bias_time = o.bias_time;
        v.push_back({"wall states (offset + tilt)", c, 1});
        c.model.map_residual = 0.25 * map_sigma;
        v.push_back({"wall states + residual patches", c, 1});
        amr::EkfConfig lines_model = base(o.noise);
        lines_model.model.surface = amr::SurfaceKind::Lines;
        v.push_back({"line surface, independent beams", lines_model, 1});
        lines_model.model.map_sigma = map_sigma;
        lines_model.model.bias_sigma = o.bias;
        lines_model.model.bias_time = o.bias_time;
        v.push_back({"line surface + wall states", lines_model, 1});
        lines_model.model.map_residual = 0.25 * map_sigma;
        v.push_back({"line surface + wall states + residual", lines_model, 1});
        if (o.sweep) {
            for (const double k : {1.0, 2.0, 3.0}) {
                for (const double tilt : {0.02, 0.05}) {
                    amr::EkfConfig swept = lines_model;
                    swept.model.map_residual = 0.0;
                    swept.model.map_sigma = k * map_sigma;
                    swept.model.map_tilt = tilt;
                    std::ostringstream name;
                    name << "lines + walls, sigma x" << k << ", tilt " << tilt;
                    v.push_back({name.str(), swept, 1});
                }
            }
        }
    }
    return runEkf(o, v, "ekf_" + o.world, "EKF with correlated range errors");
}

/// Grazing beams: the classic cutoff swept from 45 to 89 deg, against keeping every
/// beam with its linearization (and map-error) variance. On the grid world with the
/// grid surface and independent beams; on the continuous world with wall lines and
/// wall states.
int modeGrazing(const Options& o) {
    const double map_sigma = o.world == "grid" ? 0.0 : o.resolution / std::sqrt(12.0);
    amr::EkfConfig model = base(o.noise);
    if (o.world != "grid") {
        model.model.surface = amr::SurfaceKind::Lines;
        model.model.map_sigma = map_sigma;
    }
    std::vector<Variant> v;
    for (const double degrees : {45.0, 60.0, 70.0, 78.46, 84.0, 87.0, 89.0}) {
        amr::EkfConfig c = model;
        c.model.max_incidence = degrees * kPi / 180.0;
        std::ostringstream name;
        name << "cutoff " << degrees << " deg";
        v.push_back({name.str(), c, 1});
    }
    amr::EkfConfig modeled = model;
    modeled.model.grazing = amr::GrazingPolicy::Model;
    v.push_back({"model: every beam, linearization variance", modeled, 1});
    return runEkf(o, v, "grazing_" + o.world, "Grazing beams");
}

/// Diagnostics: the map error seen by exact beams from the true poses (grid surface, or
/// the wall lines with --lines), grouped by face line in 2 m pieces: how much of it a
/// per-wall offset + tilt explains, and how often the map is grossly wrong (> 0.5 m).
int modeMapError(const Options& o) {
    const amr::MapLoadResult layout = amr::loadGridMapCsv("maps/obstacles.csv", 0.5);
    std::mt19937 rng(o.seed);
    double total_sq = 0.0, resid_sq = 0.0, offset_resid_sq = 0.0, count = 0.0, gross = 0.0;
    for (int run = 0; run < o.runs; ++run) {
        const World world = makeWorld(layout.map, o, rng);
        const auto trajectory = randomRoute(world, rng);
        if (trajectory.empty()) {
            continue;
        }
        amr::LidarConfig lidar;
        lidar.beams = o.beams > 0 ? o.beams : 72;
        const auto scans = scanWorld(world, trajectory, lidar);
        const auto surface = amr::makeSurface(world.map, o.lines ? amr::SurfaceKind::Lines : amr::SurfaceKind::Grid);
        // group key -> (s, e) samples
        std::map<long long, std::vector<std::pair<double, double>>> groups;
        for (std::size_t k = 0; k < scans.size(); k += 5) {
            const auto& sc = scans[k];
            for (std::size_t i = 0; i < sc.ranges.size(); ++i) {
                if (!sc.hits[i]) continue;
                const auto h = surface->cast(sc.origin, sc.angles[i], lidar.max_range);
                if (!h.hit || !h.smooth) continue;
                const double c = amr::incidenceCosine(h, sc.angles[i]);
                if (c < 0.2) continue;
                const double e = (sc.ranges[i] - h.range) * c;  // along the normal
                if (std::fabs(e) > 0.5) {
                    gross += 1;
                    continue;
                }
                const int dir = std::fabs(h.normal.x) >= std::fabs(h.normal.y) ? (h.normal.x > 0 ? 0 : 1) : (h.normal.y > 0 ? 2 : 3);
                const amr::WorldPoint in{h.point.x - 0.3 * o.resolution * h.normal.x, h.point.y - 0.3 * o.resolution * h.normal.y};
                const amr::GridCoord cell = world.map.worldToGrid(in);
                const double sc_along = dir < 2 ? h.point.y : h.point.x;
                // component id: flood label via map blocks would be better; use cell coordinate along the normal axis band
                const long long key = (static_cast<long long>(dir) << 40) + (dir < 2 ? cell.x : cell.y) * 100000LL + (static_cast<long long>(std::floor(sc_along / 2.0)));
                groups[key].push_back({sc_along, e});
            }
        }
        for (const auto& [key, v] : groups) {
            double ms = 0, me = 0;
            for (auto [a, e] : v) { ms += a; me += e; }
            ms /= static_cast<double>(v.size());
            me /= static_cast<double>(v.size());
            double sss = 0, sse = 0;
            for (auto [a, e] : v) { sss += (a - ms) * (a - ms); sse += (a - ms) * (e - me); }
            const double slope = sss > 0 ? sse / sss : 0.0;
            for (auto [a, e] : v) {
                total_sq += e * e;
                offset_resid_sq += (e - me) * (e - me);
                const double r = e - me - slope * (a - ms);
                resid_sq += r * r;
                count += 1;
            }
        }
    }
    std::cout << "map error along the normal: rms " << std::sqrt(total_sq / count)
              << " m; after a per-face-line offset " << std::sqrt(offset_resid_sq / count)
              << " m; after offset + tilt " << std::sqrt(resid_sq / count) << " m; gross (>0.5 m) "
              << gross / (gross + count) * 100 << " % (" << gross << " of " << gross + count << ")\n";
    return 0;
}

// ---------------------------------------------------------------------------------
// Moving obstacles

struct DynamicVariant {
    std::string name;
    amr::DynamicScenario scenario;  // planning map and obstacles filled per scenario
};

struct DynamicScore {
    int runs = 0, reached = 0, collisions = 0, timeouts = 0;
    double travel = 0.0;          // summed over reached runs, s
    double paired_travel = 0.0;   // summed over runs the oracle also reached, s
    int paired = 0;
    double waits = 0.0;           // s
    double reversals = 0.0;       // robot direction reversals
    double replans = 0.0;
    int emerged = 0;              // collisions with an obstacle in view < 1 s before contact
    std::vector<double> visible_before;  // per collision: time in view before contact, s
};

/// 3 obstacles patrolling random straight lanes of the map (2-8 cells, 0.2-0.5 m/s,
/// radius 0.25-0.35 m), clear of the start and goal cells.
std::vector<amr::DynamicObstacle> randomObstacles(const amr::GridMap& map, amr::GridCoord start,
                                                  amr::GridCoord goal, std::mt19937& rng,
                                                  double min_speed, double max_speed) {
    std::vector<amr::DynamicObstacle> obstacles;
    std::uniform_int_distribution<int> xs(0, map.width() - 1);
    std::uniform_int_distribution<int> ys(0, map.height() - 1);
    std::uniform_int_distribution<int> length(2, 8);
    std::uniform_real_distribution<double> speed(min_speed, max_speed);
    std::uniform_real_distribution<double> radius(0.25, 0.35);
    while (obstacles.size() < 3) {
        const amr::GridCoord a{xs(rng), ys(rng)};
        const bool horizontal = rng() % 2 == 0;
        int cells = length(rng);
        amr::GridCoord b = a;
        while (cells-- > 0) {
            const amr::GridCoord next{b.x + (horizontal ? 1 : 0), b.y + (horizontal ? 0 : 1)};
            if (!map.isFree(next) || next == goal || next == start) {
                break;
            }
            b = next;
        }
        amr::DynamicObstacle o;
        o.route = {map.gridToWorld(a), map.gridToWorld(b)};
        o.speed = speed(rng);
        o.radius = radius(rng);
        if (b == a || !map.isFree(a) || a == start || a == goal ||
            !amr::validateDynamicObstacle(map, o).empty()) {
            continue;
        }
        obstacles.push_back(o);
    }
    return obstacles;
}

/// Number of times the robot's direction of motion flips (> 150 deg) between moves.
int countReversals(const std::vector<amr::TrajectorySample>& trajectory) {
    int count = 0;
    double last = 0.0;
    bool have = false;
    for (std::size_t k = 1; k < trajectory.size(); ++k) {
        const double dx = trajectory[k].state.x - trajectory[k - 1].state.x;
        const double dy = trajectory[k].state.y - trajectory[k - 1].state.y;
        if (std::hypot(dx, dy) < 1e-9) {
            continue;
        }
        const double heading = std::atan2(dy, dx);
        if (have && std::fabs(amr::wrapAngle(heading - last)) > 150.0 * kPi / 180.0) {
            ++count;
        }
        last = heading;
        have = true;
    }
    return count;
}

/// How long obstacle `index` had been continuously in line of sight (within range)
/// of the robot before the end of the trajectory.
double timeInView(const amr::GridMap& map, const std::vector<amr::TrajectorySample>& trajectory,
                  const amr::DynamicObstacle& obstacle, double range) {
    double since = trajectory.back().state.time;
    for (std::size_t k = trajectory.size(); k-- > 0;) {
        const auto& s = trajectory[k].state;
        const amr::WorldPoint c = amr::obstaclePosition(obstacle, s.time);
        if (!amr::isObstacleVisible(map, {s.x, s.y}, c, obstacle.radius, range)) {
            break;
        }
        since = s.time;
    }
    return trajectory.back().state.time - since;
}

int modeDynamic(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv("maps/obstacles.csv", 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << " (run from the repository root)\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    const amr::GridCoord start{1, 1}, goal{18, 18};
    std::vector<DynamicVariant> variants;
    amr::DynamicScenario oracle;
    oracle.perception.mode = amr::ObstaclePerception::Oracle;
    amr::DynamicScenario blind = oracle;
    blind.replanning.sensing_range = 1e-6;
    variants.push_back({"blind (no reaction)", blind});
    variants.push_back({"oracle sightings (original)", oracle});
    amr::DynamicScenario lidar = oracle;
    lidar.perception.mode = amr::ObstaclePerception::Lidar;
    const int beams = o.beams > 0 ? o.beams : 360;
    lidar.perception.lidar.beams = beams;
    lidar.perception.lidar.max_range = oracle.replanning.sensing_range;
    lidar.perception.lidar.noise_stddev = o.noise;
    amr::DynamicScenario cv = lidar;
    cv.perception.tracker.prediction = amr::MotionPrediction::ConstantVelocity;
    variants.push_back({"LiDAR tracking, constant velocity", cv});
    variants.push_back({"LiDAR tracking, learned reversals", lidar});
    if (o.sweep) {  // ablations
        amr::DynamicScenario no_inflation = lidar;
        no_inflation.perception.max_inflation = 0.0;
        variants.push_back({"  ... no uncertainty inflation", no_inflation});
        amr::DynamicScenario no_coast = lidar;
        no_coast.perception.tracker.coast_time = 0.3;
        variants.push_back({"  ... no coasting when unseen", no_coast});
    }

    std::mt19937 rng(o.seed);
    std::vector<std::vector<DynamicScore>> scores(2, std::vector<DynamicScore>(variants.size()));
    for (int run = 0; run < o.runs; ++run) {
        const auto obstacles = randomObstacles(map, start, goal, rng, o.min_speed, o.max_speed);
        const auto noise_seed = static_cast<std::uint32_t>(rng());
        const amr::PlanResult plan = amr::planAStar(map, start, goal);
        for (int model = 0; model < 2; ++model) {
            amr::MotionConfig motion;
            motion.model = model == 0 ? amr::RobotModel::Point : amr::RobotModel::DiffDrive;
            motion.max_time = 600.0;
            bool oracle_reached = false;
            double oracle_travel = 0.0;
            for (std::size_t v = 0; v < variants.size(); ++v) {
                amr::DynamicScenario scenario = variants[v].scenario;
                scenario.planning_map = &map;
                scenario.obstacles = obstacles;
                scenario.perception.lidar.seed = noise_seed;
                const amr::SimulationResult sim =
                    amr::simulatePath(map, plan.path, motion, amr::Connectivity::Four, &scenario);
                DynamicScore& score = scores[static_cast<std::size_t>(model)][v];
                ++score.runs;
                score.reached += sim.status == amr::NavigationStatus::GoalReached ? 1 : 0;
                score.timeouts += sim.status == amr::NavigationStatus::Timeout ? 1 : 0;
                score.travel += sim.status == amr::NavigationStatus::GoalReached ? sim.travelTime() : 0.0;
                if (v == 1) {
                    oracle_reached = sim.status == amr::NavigationStatus::GoalReached;
                    oracle_travel = sim.travelTime();
                }
                if (v >= 1 && oracle_reached && sim.status == amr::NavigationStatus::GoalReached) {
                    score.paired_travel += sim.travelTime() - oracle_travel;
                    ++score.paired;
                }
                score.waits += static_cast<double>(sim.wait_steps) * motion.dt;
                score.reversals += countReversals(sim.trajectory);
                score.replans += static_cast<double>(sim.replans.size());
                if (o.trace && sim.status != amr::NavigationStatus::GoalReached) {
                    std::cout << "# run " << run << ", " << (model == 0 ? "point" : "diff-drive") << ", "
                              << variants[v].name << ": " << amr::toString(sim.status) << " at ("
                              << sim.trajectory.back().state.x << ", " << sim.trajectory.back().state.y << ")\n";
                    for (const auto& ob : obstacles) {
                        const amr::GridCoord a = map.worldToGrid(ob.route.front());
                        const amr::GridCoord b = map.worldToGrid(ob.route.back());
                        std::cout << ob.radius << ", " << ob.speed << ", " << a.x << "," << a.y << ", " << b.x
                                  << "," << b.y << "\n";
                    }
                }
                if (sim.status == amr::NavigationStatus::Collision && sim.collided_obstacle < 0) {
                    ++score.collisions;  // a wall (no moving obstacle involved)
                    std::cerr << "wall collision: run " << run << ", model " << model << ", " << variants[v].name << "\n";
                } else if (sim.status == amr::NavigationStatus::Collision) {
                    ++score.collisions;
                    const double seen = timeInView(map, sim.trajectory,
                                                   obstacles[static_cast<std::size_t>(sim.collided_obstacle)],
                                                   oracle.replanning.sensing_range);
                    score.visible_before.push_back(seen);
                    score.emerged += seen < 1.0 ? 1 : 0;
                }
            }
        }
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "perception_dynamic.csv");
    csv << "robot,variant,runs,reached,collisions,timeouts,mean_travel_s,extra_travel_vs_oracle_s,mean_wait_s,mean_reversals,"
           "mean_replans,collisions_in_view_under_1s\n";
    std::cout << "Moving obstacles (" << o.runs << " scenarios, 3 obstacles each at " << o.min_speed << "-"
              << o.max_speed << " m/s, " << beams
              << "-beam LiDAR, noise " << o.noise << " m)\n";
    for (int model = 0; model < 2; ++model) {
        const std::string robot = model == 0 ? "point" : "diff-drive";
        std::cout << "\n" << robot << " robot\n"
                  << std::left << std::setw(38) << "variant" << std::right << std::setw(9) << "reached"
                  << std::setw(11) << "collisions" << std::setw(10) << "timeouts" << std::setw(11)
                  << "travel s" << std::setw(12) << "vs oracle" << std::setw(9) << "wait s" << std::setw(11) << "reversals"
                  << std::setw(9) << "replans" << std::setw(14) << "seen < 1 s" << "\n";
        for (std::size_t v = 0; v < variants.size(); ++v) {
            const DynamicScore& s = scores[static_cast<std::size_t>(model)][v];
            const double n = std::max(1, s.runs);
            std::cout << std::left << std::setw(38) << variants[v].name << std::right << std::fixed
                      << std::setprecision(1) << std::setw(8) << 100.0 * s.reached / n << "%"
                      << std::setw(11) << s.collisions << std::setw(10) << s.timeouts << std::setw(11)
                      << (s.reached > 0 ? s.travel / s.reached : 0.0) << std::setw(11)
                      << (s.paired > 0 ? s.paired_travel / s.paired : 0.0) << "s" << std::setw(9) << s.waits / n
                      << std::setw(11) << std::setprecision(2) << s.reversals / n << std::setw(9)
                      << std::setprecision(1) << s.replans / n << std::setw(14) << s.emerged << "\n";
            csv << robot << ",\"" << variants[v].name << "\"," << s.runs << ',' << s.reached << ','
                << s.collisions << ',' << s.timeouts << ',' << (s.reached > 0 ? s.travel / s.reached : 0.0)
                << ',' << (s.paired > 0 ? s.paired_travel / s.paired : 0.0) << ',' << s.waits / n << ',' << s.reversals / n << ',' << s.replans / n << ',' << s.emerged
                << "\n";
        }
    }
    return 0;
}

/// Blind junctions: a corridor (rows 9-10) crossed by three side corridors (columns
/// 7-8, 18-19, 29-30) that walls hide until the robot is at the crossing. One obstacle
/// per side corridor crosses the main corridor at 0.6-1.2 m/s on a long lane, with a
/// random phase. The robot drives from the west end to the east end.
amr::GridMap junctionMap() {
    const int w = 40, h = 20;
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(w * h), amr::GridMap::kOccupied);
    const auto open = [&](int x, int y) { cells[static_cast<std::size_t>(y * w + x)] = amr::GridMap::kFree; };
    for (int x = 0; x < w; ++x) {
        open(x, 9);
        open(x, 10);
    }
    for (const int c : {7, 18, 29}) {
        for (int y = 0; y < h; ++y) {
            open(c, y);
            open(c + 1, y);
        }
    }
    return amr::GridMap(w, h, 0.5, std::move(cells));
}

int modeJunction(const Options& o) {
    const amr::GridMap map = junctionMap();
    const amr::GridCoord start{1, 9}, goal{38, 10};
    std::mt19937 rng(o.seed);
    std::uniform_real_distribution<double> speed(o.min_speed, o.max_speed);
    std::uniform_int_distribution<int> phase(0, 19);
    std::vector<DynamicVariant> variants;
    amr::DynamicScenario oracle;
    oracle.perception.mode = amr::ObstaclePerception::Oracle;
    variants.push_back({"oracle sightings (original)", oracle});
    amr::DynamicScenario lidar = oracle;
    lidar.perception.mode = amr::ObstaclePerception::Lidar;
    lidar.perception.lidar.beams = o.beams > 0 ? o.beams : 360;
    lidar.perception.lidar.max_range = oracle.replanning.sensing_range;
    lidar.perception.lidar.noise_stddev = o.noise;
    variants.push_back({"LiDAR tracking, learned reversals", lidar});
    amr::DynamicScenario aware = lidar;
    aware.perception.occlusion_aware = true;
    variants.push_back({"  + occlusion-aware (blind spots)", aware});
    std::vector<std::vector<DynamicScore>> scores(2, std::vector<DynamicScore>(variants.size()));
    const amr::PlanResult plan = amr::planAStar(map, start, goal);
    for (int run = 0; run < o.runs; ++run) {
        std::vector<amr::DynamicObstacle> obstacles;
        for (const int c : {7, 18, 29}) {
            amr::DynamicObstacle ob;
            // Start the lane at a random row, so the crossing time is random.
            const int top = phase(rng) % 3;
            ob.route = {map.gridToWorld({c + static_cast<int>(rng() % 2), top}),
                        map.gridToWorld({c + static_cast<int>(rng() % 2), 19 - top})};
            ob.route[1].x = ob.route[0].x;
            ob.speed = speed(rng);
            ob.radius = 0.25;
            // Random phase: shift the lane start along the cycle by a random time.
            obstacles.push_back(ob);
        }
        // A random time offset: rotate each obstacle's start by mirroring the route.
        for (auto& ob : obstacles) {
            if (rng() % 2 == 0) {
                std::swap(ob.route[0], ob.route[1]);
            }
        }
        const auto noise_seed = static_cast<std::uint32_t>(rng());
        for (int model = 0; model < 2; ++model) {
            amr::MotionConfig motion;
            motion.model = model == 0 ? amr::RobotModel::Point : amr::RobotModel::DiffDrive;
            motion.max_time = 300.0;
            for (std::size_t v = 0; v < variants.size(); ++v) {
                amr::DynamicScenario scenario = variants[v].scenario;
                scenario.planning_map = &map;
                scenario.obstacles = obstacles;
                scenario.perception.lidar.seed = noise_seed;
                const amr::SimulationResult sim =
                    amr::simulatePath(map, plan.path, motion, amr::Connectivity::Four, &scenario);
                DynamicScore& score = scores[static_cast<std::size_t>(model)][v];
                ++score.runs;
                score.reached += sim.status == amr::NavigationStatus::GoalReached ? 1 : 0;
                score.timeouts += sim.status == amr::NavigationStatus::Timeout ? 1 : 0;
                score.travel += sim.status == amr::NavigationStatus::GoalReached ? sim.travelTime() : 0.0;
                score.waits += static_cast<double>(sim.wait_steps) * motion.dt;
                score.reversals += countReversals(sim.trajectory);
                score.replans += static_cast<double>(sim.replans.size());
                if (sim.status == amr::NavigationStatus::Collision && sim.collided_obstacle < 0) {
                    ++score.collisions;  // a wall (no moving obstacle involved)
                } else if (sim.status == amr::NavigationStatus::Collision) {
                    ++score.collisions;
                    const double seen = timeInView(map, sim.trajectory,
                                                   obstacles[static_cast<std::size_t>(sim.collided_obstacle)],
                                                   oracle.replanning.sensing_range);
                    score.emerged += seen < 1.0 ? 1 : 0;
                }
            }
        }
    }
    std::cout << "Blind junctions (" << o.runs << " runs, 3 crossing obstacles at " << o.min_speed << "-"
              << o.max_speed << " m/s)\n";
    for (int model = 0; model < 2; ++model) {
        std::cout << "\n" << (model == 0 ? "point" : "diff-drive") << " robot\n"
                  << std::left << std::setw(38) << "variant" << std::right << std::setw(9) << "reached"
                  << std::setw(11) << "collisions" << std::setw(10) << "timeouts" << std::setw(11) << "travel s"
                  << std::setw(9) << "wait s" << std::setw(14) << "seen < 1 s" << "\n";
        for (std::size_t v = 0; v < variants.size(); ++v) {
            const DynamicScore& s = scores[static_cast<std::size_t>(model)][v];
            const double n = std::max(1, s.runs);
            std::cout << std::left << std::setw(38) << variants[v].name << std::right << std::fixed
                      << std::setprecision(1) << std::setw(8) << 100.0 * s.reached / n << "%" << std::setw(11)
                      << s.collisions << std::setw(10) << s.timeouts << std::setw(11)
                      << (s.reached > 0 ? s.travel / s.reached : 0.0) << std::setw(9) << s.waits / n
                      << std::setw(14) << s.emerged << "\n";
        }
    }
    return 0;
}

/// Prediction accuracy: a robot standing at a random free cell watches one obstacle
/// patrol a random lane (60 s, 360-beam LiDAR). Every scan, each confirmed track's
/// predicted position 1 s and 2 s ahead is compared with where the obstacle really is
/// then, for constant-velocity and learned-reversal prediction (same detections).
int modePrediction(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv("maps/obstacles.csv", 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << " (run from the repository root)\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    std::mt19937 rng(o.seed);
    std::uniform_int_distribution<int> xs(0, map.width() - 1), ys(0, map.height() - 1);
    amr::LidarConfig lidar;
    lidar.beams = o.beams > 0 ? o.beams : 360;
    lidar.max_range = 3.0;
    lidar.noise_stddev = o.noise;
    const double dt = 0.1;
    struct Stat {
        double sq[2] = {0.0, 0.0};
        double count[2] = {0.0, 0.0};
        double far[2] = {0.0, 0.0};  // errors > 0.5 m
    };
    Stat stats[2];  // 0: constant velocity, 1: learned reversals
    Stat near_reversal[2];
    int used = 0;
    for (int run = 0; run < o.runs; ++run) {
        const auto obstacles = randomObstacles(map, {1, 1}, {18, 18}, rng, o.min_speed, o.max_speed);
        const amr::DynamicObstacle& ob = obstacles.front();
        // A free robot cell within 2.5 m of the lane's middle, in line of sight of it.
        const amr::WorldPoint middle{0.5 * (ob.route[0].x + ob.route[1].x), 0.5 * (ob.route[0].y + ob.route[1].y)};
        amr::WorldPoint robot{-1, -1};
        for (int attempt = 0; attempt < 500; ++attempt) {
            const amr::GridCoord c{xs(rng), ys(rng)};
            const amr::WorldPoint p = map.gridToWorld(c);
            const double d = std::hypot(p.x - middle.x, p.y - middle.y);
            if (map.isFree(c) && d > 0.8 && d < 2.5 &&
                !amr::castRay(map, p, std::atan2(middle.y - p.y, middle.x - p.x), d).hit &&
                amr::pointSegmentDistance(p, ob.route[0], ob.route[1]) > ob.radius + 0.2) {
                robot = p;
                break;
            }
        }
        if (robot.x < 0) {
            continue;
        }
        ++used;
        amr::TrackerConfig cv_config;
        cv_config.prediction = amr::MotionPrediction::ConstantVelocity;
        amr::ObstacleTracker trackers[2] = {amr::ObstacleTracker(cv_config), amr::ObstacleTracker(amr::TrackerConfig{})};
        lidar.seed = static_cast<std::uint32_t>(rng());
        amr::RangeErrorModel errors(lidar);
        const auto offsets = amr::beamOffsets(lidar);
        const double period = 2.0 * amr::routeLength(ob) / ob.speed;
        for (int k = 0; k < 600; ++k) {
            const double t = k * dt;
            amr::LidarScan scan;
            scan.origin = robot;
            scan.time = t;
            errors.nextScan(t);
            for (const double a : offsets) {
                const amr::RayHit wall = amr::castRay(map, robot, a, lidar.max_range);
                const double r = std::min(wall.hit ? wall.range : 1e300,
                                          amr::rayToCircle(amr::obstaclePosition(ob, t), ob.radius, robot, a));
                const bool hit = r <= lidar.max_range;
                scan.angles.push_back(a);
                scan.ranges.push_back(hit ? errors.measure(r) : lidar.max_range);
                scan.hits.push_back(hit);
            }
            const auto detections = amr::detectMovingObstacles(map, scan, lidar.max_range, true, amr::DetectionConfig{});
            for (int m = 0; m < 2; ++m) {
                trackers[m].update(t, detections, map, scan, lidar.max_range);
                for (const amr::Track& track : trackers[m].tracks()) {
                    if (!track.confirmed || t < 10.0) {
                        continue;  // after a warm-up of 10 s (a few reversals)
                    }
                    const auto path = trackers[m].predictPath(track, dt, 20);
                    for (int h = 0; h < 2; ++h) {
                        const std::size_t step = h == 0 ? 10 : 20;
                        const amr::WorldPoint truth = amr::obstaclePosition(ob, t + static_cast<double>(step) * dt);
                        const double e = std::hypot(path[step].center.x - truth.x, path[step].center.y - truth.y);
                        // Near a reversal: one happens within the prediction window.
                        const double phase = std::fmod(t, period / 2.0);
                        const bool reversal = period / 2.0 - phase < static_cast<double>(step) * dt;
                        for (Stat* st : {&stats[m], reversal ? &near_reversal[m] : nullptr}) {
                            if (st == nullptr) {
                                continue;
                            }
                            st->sq[h] += e * e;
                            st->far[h] += e > 0.5 ? 1.0 : 0.0;
                            st->count[h] += 1.0;
                        }
                    }
                }
            }
        }
    }
    std::cout << "Obstacle motion prediction (" << used << " lanes, " << o.min_speed << "-" << o.max_speed
              << " m/s, " << lidar.beams << "-beam LiDAR)\n"
              << std::left << std::setw(26) << "prediction" << std::right << std::setw(14) << "RMS 1 s / 2 s"
              << std::setw(22) << "> 0.5 m off at 2 s" << std::setw(36) << "reversal within 2 s: RMS, > 0.5 m" << "\n";
    for (int m = 0; m < 2; ++m) {
        const Stat& a = stats[m];
        const Stat& b = near_reversal[m];
        std::cout << std::left << std::setw(26) << (m == 0 ? "constant velocity" : "learned reversals") << std::right
                  << std::fixed << std::setprecision(2) << std::setw(8) << std::sqrt(a.sq[0] / a.count[0]) << " / "
                  << std::sqrt(a.sq[1] / a.count[1]) << " m" << std::setw(15) << std::setprecision(1)
                  << 100.0 * a.far[1] / a.count[1] << " %" << std::setw(18) << std::setprecision(2)
                  << std::sqrt(b.sq[1] / b.count[1]) << " m, " << std::setprecision(1)
                  << 100.0 * b.far[1] / b.count[1] << " %\n";
    }
    return 0;
}

bool parse(int argc, char** argv, Options& o) {
    if (argc < 2) {
        return false;
    }
    o.mode = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sweep" || arg == "--lines" || arg == "--trace") {
            (arg == "--sweep" ? o.sweep : arg == "--lines" ? o.lines : o.trace) = true;
            continue;
        }
        if (i + 1 >= argc) {
            return false;
        }
        const std::string value = argv[++i];
        if (arg == "--runs") {
            o.runs = std::atoi(value.c_str());
        } else if (arg == "--seed") {
            o.seed = static_cast<unsigned>(std::strtoul(value.c_str(), nullptr, 10));
        } else if (arg == "--beams") {
            o.beams = std::atoi(value.c_str());
        } else if (arg == "--noise") {
            o.noise = std::atof(value.c_str());
        } else if (arg == "--bias") {
            o.bias = std::atof(value.c_str());
        } else if (arg == "--bias-time") {
            o.bias_time = std::atof(value.c_str());
        } else if (arg == "--world") {
            o.world = value;
        } else if (arg == "--resolution") {
            o.resolution = std::atof(value.c_str());
        } else if (arg == "--rotation") {
            o.rotation = std::atof(value.c_str());
        } else if (arg == "--speeds") {
            const auto comma = value.find(',');
            o.min_speed = std::atof(value.substr(0, comma).c_str());
            o.max_speed = comma == std::string::npos ? o.min_speed : std::atof(value.substr(comma + 1).c_str());
        } else if (arg == "--output") {
            o.output = value;
        } else {
            return false;
        }
    }
    return o.world == "grid" || o.world == "continuous";
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::cerr << "usage: perception_experiment ekf|grazing|dynamic [--runs N] [--seed N] "
                     "[--beams N] [--noise M] [--bias M] [--bias-time S] [--world grid|continuous] "
                     "[--resolution M] [--rotation DEG] [--output DIR]\n";
        return 2;
    }
    if (o.mode == "ekf") {
        return modeEkf(o);
    }
    if (o.mode == "prediction") {
        return modePrediction(o);
    }
    if (o.mode == "junction") {
        return modeJunction(o);
    }
    if (o.mode == "dynamic") {
        return modeDynamic(o);
    }
    if (o.mode == "grazing") {
        return modeGrazing(o);
    }
    if (o.mode == "maperror") {
        return modeMapError(o);
    }
    std::cerr << "unknown mode '" << o.mode << "'\n";
    return 2;
}
