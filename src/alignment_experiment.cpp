// alignment_experiment: grid-alignment artifacts in occupancy-grid planning.
//
//   alignment_experiment doorway [options]   door width sweep: for each width, many
//                                            random grid rotations and sub-cell
//                                            offsets; every rasterization rule, the
//                                            LiDAR grid, and the alignment-robust
//                                            planners. Truth: passable iff width >= 2 r.
//   alignment_experiment clutter [options]   random rotated boxes: how often each
//                                            method's answer changes with alignment,
//                                            and how often it is wrong.
//   alignment_experiment example [options]   one door (0.75 m) on an axis-aligned grid at
//                                            two sub-cell offsets, one passable and one
//                                            blocked (any overlap), for the figure
//   alignment_experiment classify [options]  grid NO_PATH from LiDAR maps: discretization
//                                            artifacts vs. ghosts vs. real blocks;
//                                            problem 1's grid recovery against the
//                                            point-aware recovery.
//
// Options: --trials N (alignments per width / worlds / cases), --seed N,
//          --resolution M (default 0.1), --radius M (default 0.3), --output DIR
//          (default results/alignment). Writes alignment_<mode>.csv and a summary.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "alignment.hpp"
#include "footprint.hpp"
#include "ghost_perception.hpp"
#include "grid_map.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Options {
    std::string mode;
    int trials = 60;
    unsigned seed = 1;
    double resolution = 0.1;
    double radius = 0.3;
    std::filesystem::path output = "results/alignment";
};

/// A scene placed under a randomly aligned grid, in the grid frame.
struct Placed {
    amr::ContinuousWorld world;
    amr::WorldPoint start;
    amr::WorldPoint goal;
    std::vector<amr::WorldPoint> viewpoints;
    double angle = 0.0;
    amr::WorldPoint subcell;
};

Placed place(const std::vector<amr::Polygon>& obstacles, amr::WorldPoint start,
             amr::WorldPoint goal, const std::vector<amr::WorldPoint>& viewpoints,
             const std::vector<amr::GhostDisc>& ghosts, amr::WorldPoint center, double extent,
             double resolution, std::mt19937& rng, double fixed_angle = -1.0) {
    std::uniform_real_distribution<double> angle_dist(0.0, kPi / 2.0);
    std::uniform_real_distribution<double> sub(0.0, resolution);
    const double random_angle = angle_dist(rng);
    const double angle = fixed_angle >= 0.0 ? fixed_angle : random_angle;
    const amr::WorldPoint subcell{sub(rng), sub(rng)};
    const amr::GridPlacement g = amr::placeGrid(center, extent, resolution, angle, subcell);
    std::vector<amr::Polygon> polygons;
    for (const amr::Polygon& p : obstacles) {
        polygons.push_back(amr::polygonToGridFrame(g.pose, p));
    }
    std::vector<amr::GhostDisc> placed_ghosts;
    for (amr::GhostDisc d : ghosts) {
        d.center = amr::worldToGridFrame(g.pose, d.center);
        d.direction -= angle;
        placed_ghosts.push_back(d);
    }
    std::vector<amr::WorldPoint> views;
    for (const amr::WorldPoint& v : viewpoints) {
        views.push_back(amr::worldToGridFrame(g.pose, v));
    }
    return {amr::ContinuousWorld(std::move(polygons), std::move(placed_ghosts), g.cells, g.cells,
                                 resolution),
            amr::worldToGridFrame(g.pose, start), amr::worldToGridFrame(g.pose, goal), views,
            angle, subcell};
}

amr::GridCoord cellOf(amr::WorldPoint p, double res) {
    return {static_cast<int>(std::floor(p.x / res)), static_cast<int>(std::floor(p.y / res))};
}

bool gridPassable(const amr::GridMap& map, const Placed& s, double radius) {
    const double res = map.resolution();
    return amr::planWithFootprint(map, amr::inflateObstacles(map, radius), cellOf(s.start, res),
                                  cellOf(s.goal, res))
        .success();
}

amr::SensorModel exactSensor() {
    amr::SensorModel sensor;
    sensor.beams = 720;
    sensor.miss_probability = 0.0;
    sensor.spurious_probability = 0.0;
    return sensor;
}

/// Every method's answer for one placed scene.
std::map<std::string, bool> answers(const Placed& s, double radius, double spacing,
                                    std::mt19937& rng) {
    std::map<std::string, bool> out;
    const amr::ContinuousWorld& w = s.world;
    for (const amr::RasterRule rule : {amr::RasterRule::AnyOverlap, amr::RasterRule::CenterInside,
                                       amr::RasterRule::Majority}) {
        out[amr::toString(rule)] =
            gridPassable(amr::rasterize(w.obstacles(), w.width(), w.height(), w.resolution(), rule), s,
                         radius);
    }
    amr::OccupancyBelief belief(w.width(), w.height(), w.resolution());
    amr::PointMap points(w.width(), w.height(), w.resolution());
    for (const amr::WorldPoint& v : s.viewpoints) {
        amr::scanInto(belief, points, w, v, exactSensor(), rng);
    }
    out["lidar_grid"] = gridPassable(belief.map(), s, radius);
    const std::vector<bool> observed = amr::observedCells(belief);
    out["lidar_points"] = amr::planOnClearance(points.segments(2.0 * radius, 2.0 * kPi / exactSensor().beams),
                                               w.width(), w.height(),
                                               w.resolution(), s.start, s.goal, radius, spacing,
                                               &observed)
                              .found;
    out["exact_lattice"] = amr::planOnPolygons(w, s.start, s.goal, radius, spacing).found;
    return out;
}

const std::vector<std::string> kMethods = {"any_overlap", "center_inside", "majority",
                                           "lidar_grid", "lidar_points", "exact_lattice"};

int runDoorway(const Options& o) {
    const double spacing = o.resolution / 4.0;
    std::ofstream csv(o.output / "alignment_doorway.csv");
    csv << "door_width,angle_deg,subcell_x,subcell_y,truth,method,passable\n";
    std::mt19937 rng(o.seed);
    std::map<std::string, std::map<int, int>> passable;  // method -> width (mm) -> count
    std::map<std::string, int> unsafe;                   // passable although truly blocked
    std::map<std::string, int> blocked_wrongly;
    std::vector<int> widths;
    for (int mm = 500; mm <= 1100; mm += 10) {
        widths.push_back(mm);
    }
    for (const int mm : widths) {
        const double width = mm / 1000.0;
        const amr::DoorwayScene scene = amr::makeDoorwayScene(width);
        const bool truth = width >= 2.0 * o.radius - 1e-12;
        for (int t = 0; t < o.trials; ++t) {
            const Placed s = place(scene.obstacles, scene.start, scene.goal, scene.viewpoints, {},
                                   {2.0, 2.0}, 4.4, o.resolution, rng);
            for (const auto& [method, ok] : answers(s, o.radius, spacing, rng)) {
                csv << std::fixed << std::setprecision(3) << width << ',' << s.angle * 180.0 / kPi
                    << ',' << std::setprecision(4) << s.subcell.x << ',' << s.subcell.y << ','
                    << truth << ',' << method << ',' << ok << '\n';
                passable[method][mm] += ok;
                unsafe[method] += ok && !truth;
                blocked_wrongly[method] += !ok && truth;
            }
        }
    }
    std::cout << "Doorway: robot radius " << o.radius << " m (passable iff width >= "
              << 2 * o.radius << " m), cells " << o.resolution << " m, lattice " << spacing
              << " m, " << o.trials << " random alignments per width\n\n";
    std::cout << std::left << std::setw(15) << "method" << std::setw(42)
              << "widths where alignment decides (m)" << std::setw(10) << "band (m)"
              << std::setw(22) << "passable when blocked" << "blocked when passable\n";
    for (const std::string& m : kMethods) {
        int lo = -1, hi = -1;
        for (const int mm : widths) {
            const int n = passable[m][mm];
            if (n > 0 && n < o.trials) {
                lo = lo < 0 ? mm : lo;
                hi = mm;
            }
        }
        std::ostringstream band, range;
        if (lo < 0) {
            range << "none (same answer for every alignment)";
            band << "0";
        } else {
            range << lo / 1000.0 << " - " << hi / 1000.0;
            band << (hi - lo + 10) / 1000.0;
        }
        std::cout << std::left << std::setw(15) << m << std::setw(42) << range.str() << std::setw(10)
                  << band.str() << std::setw(22) << unsafe[m] << blocked_wrongly[m] << '\n';
    }
    return 0;
}

int runClutter(const Options& o) {
    const double spacing = o.resolution / 4.0;
    constexpr int kAlignments = 12;
    std::ofstream csv(o.output / "alignment_clutter.csv");
    csv << "world,alignment,truth,method,passable\n";
    std::mt19937 rng(o.seed);
    std::map<std::string, int> flips, wrong, unsafe;
    int worlds = 0;
    int borderline = 0;
    while (worlds < o.trials) {
        // A 4 m room with 6 random boxes; start and goal on opposite sides.
        const amr::DoorwayScene room = amr::makeDoorwayScene(4.0);  // no dividing wall
        std::vector<amr::Polygon> obstacles = room.obstacles;
        std::uniform_real_distribution<double> pos(0.6, 3.4), size(0.2, 0.9), turn(0.0, kPi);
        for (int k = 0; k < 6; ++k) {
            obstacles.push_back(amr::makeRectangle({pos(rng), pos(rng)}, size(rng), size(rng), turn(rng)));
        }
        const amr::WorldPoint start{0.7, 0.7};  // 0.7 m from the room walls
        const amr::WorldPoint goal{3.3, 3.3};
        bool clear_ends = true;
        for (const amr::Polygon& p : obstacles) {
            clear_ends = clear_ends && amr::distanceToPolygon(p, start) > 0.5 &&
                         amr::distanceToPolygon(p, goal) > 0.5;
        }
        if (!clear_ends) {
            continue;
        }
        const std::vector<amr::WorldPoint> views = {start, goal, {0.7, 3.3}, {3.3, 0.7}, {2.0, 2.0}};
        // Truth from the first placement (it does not depend on the grid).
        const Placed first = place(obstacles, start, goal, views, {}, {2.0, 2.0}, 4.4, o.resolution, rng);
        const amr::Passability truth =
            amr::truePassability(first.world, first.start, first.goal, o.radius, 0.005);
        if (truth == amr::Passability::Borderline) {
            ++borderline;
            continue;
        }
        const bool passable_truth = truth == amr::Passability::Passable;
        std::map<std::string, std::vector<bool>> seen;
        for (int a = 0; a < kAlignments; ++a) {
            const Placed s = place(obstacles, start, goal, views, {}, {2.0, 2.0}, 4.4, o.resolution, rng);
            for (const auto& [method, ok] : answers(s, o.radius, spacing, rng)) {
                seen[method].push_back(ok);
                csv << worlds << ',' << a << ',' << passable_truth << ',' << method << ',' << ok << '\n';
            }
        }
        for (const auto& [method, v] : seen) {
            const bool any = std::find(v.begin(), v.end(), true) != v.end();
            const bool all = std::find(v.begin(), v.end(), false) == v.end();
            flips[method] += any && !all;
            for (const bool ok : v) {
                wrong[method] += ok != passable_truth;
                unsafe[method] += ok && !passable_truth;
            }
        }
        ++worlds;
    }
    std::cout << "Clutter: " << worlds << " random rooms (6 rotated boxes), " << kAlignments
              << " alignments each, cells " << o.resolution << " m, robot radius " << o.radius
              << " m (" << borderline << " borderline worlds skipped)\n\n";
    std::cout << std::left << std::setw(15) << "method" << std::setw(28) << "worlds whose answer flips"
              << std::setw(20) << "wrong answers" << "passable when blocked\n";
    const int total = worlds * kAlignments;
    for (const std::string& m : kMethods) {
        std::cout << std::left << std::setw(15) << m << std::setw(28)
                  << (std::to_string(flips[m]) + " / " + std::to_string(worlds)) << std::setw(20)
                  << (std::to_string(wrong[m]) + " / " + std::to_string(total)) << unsafe[m] << '\n';
    }
    return 0;
}

int runExample(const Options& o) {
    // The same door on an axis-aligned grid: only the sub-cell offset differs.
    constexpr double kWidth = 0.75;
    const amr::DoorwayScene scene = amr::makeDoorwayScene(kWidth);
    std::mt19937 rng(o.seed);
    int found = 0;
    bool want = true;  // first a passable alignment, then a blocked one
    std::ofstream meta(o.output / "example_meta.csv");
    meta << "example,door_width,angle_deg,subcell_x,subcell_y,radius,resolution,passable,start_x,start_y,goal_x,goal_y\n";
    std::ofstream cells(o.output / "example_cells.csv");
    cells << "example,x,y,occupied,inflated\n";
    std::ofstream polys(o.output / "example_polygons.csv");
    polys << "example,polygon,vertex,x,y\n";
    std::ofstream path(o.output / "example_path.csv");
    path << "example,x,y\n";
    for (int attempt = 0; attempt < 1000 && found < 2; ++attempt) {
        const Placed s = place(scene.obstacles, scene.start, scene.goal, scene.viewpoints, {},
                               {2.0, 2.0}, 4.4, o.resolution, rng, 0.0);
        const amr::ContinuousWorld& w = s.world;
        const amr::GridMap map = amr::rasterize(w.obstacles(), w.width(), w.height(), o.resolution,
                                                amr::RasterRule::AnyOverlap);
        const amr::GridMap inflated = amr::inflateObstacles(map, o.radius);
        const amr::PlanResult plan = amr::planWithFootprint(map, inflated, cellOf(s.start, o.resolution),
                                                            cellOf(s.goal, o.resolution));
        if (plan.success() != want) {
            continue;
        }
        meta << found << ',' << kWidth << ',' << s.angle * 180.0 / kPi << ',' << s.subcell.x << ','
             << s.subcell.y << ',' << o.radius << ',' << o.resolution << ',' << plan.success() << ','
             << s.start.x << ',' << s.start.y << ',' << s.goal.x << ',' << s.goal.y << '\n';
        for (int y = 0; y < w.height(); ++y) {
            for (int x = 0; x < w.width(); ++x) {
                cells << found << ',' << x << ',' << y << ',' << map.isOccupied({x, y}) << ','
                      << inflated.isOccupied({x, y}) << '\n';
            }
        }
        for (std::size_t p = 0; p < w.obstacles().size(); ++p) {
            const auto& v = w.obstacles()[p].vertices;
            for (std::size_t k = 0; k < v.size(); ++k) {
                polys << found << ',' << p << ',' << k << ',' << v[k].x << ',' << v[k].y << '\n';
            }
        }
        for (const amr::GridCoord& c : plan.path) {
            path << found << ',' << c.x << ',' << c.y << '\n';
        }
        std::cout << "example " << found << ": angle " << s.angle * 180.0 / kPi << " deg, offset ("
                  << s.subcell.x << ", " << s.subcell.y << ") -> "
                  << (plan.success() ? "passable" : "NO_PATH") << '\n';
        ++found;
        want = false;
    }
    return found == 2 ? 0 : 1;
}

int runClassify(const Options& o) {
    std::ofstream csv(o.output / "alignment_classify.csv");
    csv << "case,kind,door_width,pipeline,outcome,artifact,rounds,beams,real_cells_cleared\n";
    std::mt19937 rng(o.seed);
    amr::SensorModel sensor;  // default noise (misses, spurious returns)
    sensor.beams = 720;
    const std::vector<std::string> kinds = {"artifact", "ghost", "real"};
    std::map<std::string, std::map<std::string, std::map<std::string, int>>> table;
    std::map<std::string, std::map<std::string, std::vector<double>>> beams;
    std::map<std::string, int> cases;
    std::map<std::string, int> generated;
    int id = 0;
    for (const std::string& kind : kinds) {
        while (cases[kind] < o.trials) {
            if (++generated[kind] > 50 * o.trials) {
                std::cerr << "warning: too few grid NO_PATH cases of kind " << kind << '\n';
                break;
            }
            std::uniform_real_distribution<double> narrow(2 * o.radius + 0.005, 2 * o.radius + 0.15);
            std::uniform_real_distribution<double> wide(2 * o.radius + 0.3, 2 * o.radius + 0.6);
            std::uniform_real_distribution<double> blocked(2 * o.radius - 0.15, 2 * o.radius - 0.005);
            const double width = kind == "artifact" ? narrow(rng) : kind == "ghost" ? wide(rng) : blocked(rng);
            const amr::DoorwayScene scene = amr::makeDoorwayScene(width);
            std::vector<amr::GhostDisc> ghosts;
            if (kind == "ghost") {
                std::uniform_real_distribution<double> heading(-kPi, kPi), jitter(-0.1, 0.1);
                // A strong reflection (e.g. a glass pane edge): 8 cm, 95 % of in-sector beams,
                // a 140 deg sector.
                ghosts.push_back({{scene.door_center.x + jitter(rng), scene.door_center.y + jitter(rng)},
                                  0.08, heading(rng), 1.2, 0.95});
            }
            const Placed s = place(scene.obstacles, scene.start, scene.goal, scene.viewpoints, ghosts,
                                   {2.0, 2.0}, 4.4, o.resolution, rng);
            amr::OccupancyBelief belief(s.world.width(), s.world.height(), o.resolution);
            amr::PointMap points(s.world.width(), s.world.height(), o.resolution);
            for (const amr::WorldPoint& v : s.viewpoints) {
                amr::scanInto(belief, points, s.world, v, sensor, rng);
            }
            if (gridPassable(belief.map(), s, o.radius)) {
                continue;  // only grid NO_PATH cases
            }
            ++cases[kind];
            ++id;
            const unsigned trial_seed = static_cast<unsigned>(rng());
            // Problem 1's recovery on the grid alone.
            {
                std::mt19937 r(trial_seed);
                amr::RecoveryConfig config;
                config.robot_radius = o.radius;
                std::vector<amr::WorldPoint> views = s.viewpoints;
                const amr::RecoveryResult result = amr::recover(
                    amr::RecoveryStrategy::MostLikely, belief, s.world, views,
                    cellOf(s.start, o.resolution), cellOf(s.goal, o.resolution), sensor, config, r);
                const std::string outcome = amr::toString(result.outcome);
                table[kind]["grid_recovery"][outcome]++;
                if (result.outcome == amr::RecoveryOutcome::Recovered) {
                    beams[kind]["grid_recovery"].push_back(static_cast<double>(result.beams));
                }
                csv << id << ',' << kind << ',' << std::fixed << std::setprecision(4) << width
                    << ",grid_recovery," << outcome << ",0," << result.rounds << ',' << result.beams
                    << ',' << result.real_cells_cleared << '\n';
            }
            // Point-aware: plan on the points first, then explain and re-sense.
            {
                std::mt19937 r(trial_seed);
                amr::PointRecoveryConfig config;
                config.robot_radius = o.radius;
                config.lattice_spacing = o.resolution / 4.0;
                const amr::PointRecoveryResult result =
                    amr::recoverWithPoints(belief, points, s.world, s.start, s.goal, sensor, config, r);
                std::string outcome = amr::toString(result.outcome);
                table[kind]["point_recovery"][outcome]++;
                table[kind]["point_recovery"][result.artifact ? "said_artifact" : "said_sensor"]++;
                if (!result.reason.empty()) {
                    std::cerr << "  point recovery gave up (" << kind << ", door " << width
                              << "): " << result.reason << '\n';
                }
                if (result.outcome == amr::RecoveryOutcome::Recovered) {
                    beams[kind]["point_recovery"].push_back(static_cast<double>(result.beams));
                }
                csv << id << ',' << kind << ',' << std::fixed << std::setprecision(4) << width
                    << ",point_recovery," << outcome << ',' << result.artifact << ',' << result.rounds
                    << ',' << result.beams << ",0\n";
            }
        }
    }
    std::cout << "Classify: grid NO_PATH from noisy LiDAR maps (720 beams), cells " << o.resolution
              << " m, robot radius " << o.radius << " m, " << o.trials << " cases per kind\n"
              << "  artifact: door 1-15 cm wider than 2 r (passable); ghost: door 30-60 cm wider "
                 "than 2 r with a reflection in it; real: door 1-15 cm narrower than 2 r (blocked)\n"
              << "  grid NO_PATH rate: ";
    for (const std::string& kind : kinds) {
        std::cout << kind << " " << cases[kind] << "/" << generated[kind] << "  ";
    }
    std::cout << "\n\n" << std::left << std::setw(10) << "kind" << std::setw(16) << "pipeline"
              << std::setw(11) << "recovered" << std::setw(8) << "unsafe" << std::setw(9) << "gave up"
              << std::setw(16) << "said artifact" << "median beams\n";
    for (const std::string& kind : kinds) {
        for (const std::string pipeline : {"grid_recovery", "point_recovery"}) {
            auto& t = table[kind][pipeline];
            auto b = beams[kind][pipeline];
            std::sort(b.begin(), b.end());
            std::cout << std::left << std::setw(10) << kind << std::setw(16) << pipeline
                      << std::setw(11) << t["RECOVERED"] << std::setw(8) << t["UNSAFE"] << std::setw(9)
                      << t["GAVE_UP"] << std::setw(16)
                      << (pipeline == "point_recovery" ? std::to_string(t["said_artifact"]) : "-")
                      << (b.empty() ? 0.0 : b[b.size() / 2]) << '\n';
        }
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    o.mode = argc >= 2 ? argv[1] : "";
    if (o.mode != "doorway" && o.mode != "clutter" && o.mode != "classify" && o.mode != "example") {
        std::cerr << "usage: alignment_experiment doorway|clutter|classify|example [--trials N] [--seed N] "
                     "[--resolution M] [--radius M] [--output DIR]\n";
        return 1;
    }
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string arg = argv[i];
        if (arg == "--trials") {
            o.trials = std::max(1, std::atoi(argv[i + 1]));
        } else if (arg == "--seed") {
            o.seed = static_cast<unsigned>(std::strtoul(argv[i + 1], nullptr, 10));
        } else if (arg == "--resolution") {
            o.resolution = std::atof(argv[i + 1]);
        } else if (arg == "--radius") {
            o.radius = std::atof(argv[i + 1]);
        } else if (arg == "--output") {
            o.output = argv[i + 1];
        } else {
            std::cerr << "unknown option " << arg << '\n';
            return 1;
        }
    }
    std::filesystem::create_directories(o.output);
    if (o.mode == "doorway") {
        return runDoorway(o);
    }
    if (o.mode == "example") {
        return runExample(o);
    }
    return o.mode == "clutter" ? runClutter(o) : runClassify(o);
}
