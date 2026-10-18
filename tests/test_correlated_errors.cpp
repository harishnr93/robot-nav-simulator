// Tests for correlated LiDAR errors and the EKF's measurement models: the range
// bias process, the augmented filter (bias and wall map-error states), the wall-line
// surface recovered from the grid, and grazing beams.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "alignment.hpp"
#include "astar.hpp"
#include "ekf.hpp"
#include "footprint.hpp"
#include "geometry2d.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "simulator.hpp"
#include "surface_model.hpp"
#include "test_framework.hpp"

using amr::EkfConfig;
using amr::GridMap;
using amr::LidarConfig;
using amr::WorldPoint;

namespace {

constexpr double kPi = 3.14159265358979323846;

GridMap makeMap(const std::vector<std::string>& rows, double resolution) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                   resolution, std::move(cells));
}

/// 10 m x 10 m room with a few obstacles (0.5 m cells), as in test_ekf.
GridMap room() {
    return makeMap({"....................", "....................", "...###..............",
                    "...###.........##...", "...............##...", "....................",
                    "........####........", "........####........", "....................",
                    "....................", "..##................", "..##.........#......",
                    ".............#......", ".............#......", "....................",
                    "......#####.........", "....................", "................##..",
                    "................##..", "...................."},
                   0.5);
}

std::vector<amr::TrajectorySample> driveThrough(const GridMap& map, amr::GridCoord start,
                                                amr::GridCoord goal, double radius = 0.0) {
    const GridMap inflated = amr::inflateObstacles(map, radius);
    const auto plan = amr::planWithFootprint(map, inflated, start, goal);
    amr::MotionConfig config;
    config.model = amr::RobotModel::DiffDrive;
    config.robot_radius = radius;
    config.initial_heading = kPi / 2;
    return amr::simulatePath(map, plan.path, config).trajectory;
}

double meanNees(const std::vector<amr::LocalizationSample>& samples) {
    double sum = 0.0;
    for (const auto& s : samples) {
        sum += s.nees;
    }
    return sum / static_cast<double>(samples.size());
}

// ---------------------------------------------------------------------------------
// Range bias (lidar.hpp)

void testBiasProcess() {
    // Constant bias: the same in every scan; Gauss-Markov: stationary std and the
    // lag-one correlation exp(-dt / tau).
    LidarConfig constant;
    constant.beams = 1;
    constant.bias_stddev = 0.03;
    amr::RangeErrorModel fixed(constant);
    fixed.nextScan(0.0);
    const double first = fixed.bias();
    for (int k = 1; k < 50; ++k) {
        fixed.nextScan(0.1 * k);
        CHECK_EQ(fixed.bias(), first);
    }
    double sum_sq = 0.0;
    for (std::uint32_t seed = 0; seed < 4000; ++seed) {
        constant.seed = seed;
        amr::RangeErrorModel model(constant);
        model.nextScan(0.0);
        sum_sq += model.bias() * model.bias();
    }
    CHECK_NEAR(std::sqrt(sum_sq / 4000.0), 0.03, 0.0015);

    LidarConfig drifting = constant;
    drifting.bias_time = 2.0;
    drifting.seed = 7;
    amr::RangeErrorModel model(drifting);
    std::vector<double> b;
    for (int k = 0; k < 40000; ++k) {
        model.nextScan(0.1 * k);
        b.push_back(model.bias());
    }
    double var = 0.0, lag = 0.0;
    for (std::size_t k = 0; k + 1 < b.size(); ++k) {
        var += b[k] * b[k];
        lag += b[k] * b[k + 1];
    }
    CHECK_NEAR(std::sqrt(var / static_cast<double>(b.size() - 1)), 0.03, 0.003);
    CHECK_NEAR(lag / var, std::exp(-0.1 / 2.0), 0.005);
}

void testBiasShiftsEveryBeamOfAScan() {
    const GridMap map = room();
    const auto trajectory = driveThrough(map, {1, 1}, {18, 18});
    LidarConfig exact;
    exact.beams = 36;
    LidarConfig biased = exact;
    biased.bias_stddev = 0.05;
    biased.bias_time = 1.0;
    const auto a = amr::scanTrajectory(map, trajectory, exact);
    const auto b = amr::scanTrajectory(map, trajectory, biased);
    REQUIRE(a.size() == b.size());
    double previous = 0.0;
    bool changes = false;
    for (std::size_t k = 0; k < a.size(); ++k) {
        double shift = 0.0;
        bool have = false;
        for (std::size_t i = 0; i < a[k].ranges.size(); ++i) {
            CHECK_EQ(a[k].hits[i], b[k].hits[i]);
            if (!a[k].hits[i]) {
                CHECK_EQ(a[k].ranges[i], b[k].ranges[i]);  // misses keep max_range
                continue;
            }
            if (a[k].ranges[i] > exact.max_range - 0.2) {
                continue;  // near the range limit the biased range may be clamped
            }
            if (!have) {
                shift = b[k].ranges[i] - a[k].ranges[i];
                have = true;
            }
            CHECK_NEAR(b[k].ranges[i] - a[k].ranges[i], shift, 1e-12);
        }
        changes = changes || (k > 0 && shift != previous);
        previous = shift;
    }
    CHECK(changes);  // the bias drifts between scans
}

/// Without a bias the scans are exactly those of the original model: white noise
/// from one generator, drawn for each beam with a return, clamped to [0, range].
void testNoBiasKeepsTheOriginalScans() {
    const GridMap map = room();
    const auto trajectory = driveThrough(map, {1, 1}, {18, 18});
    LidarConfig lidar;
    lidar.beams = 24;
    lidar.noise_stddev = 0.08;
    lidar.seed = 11;
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    std::mt19937 rng(lidar.seed);
    std::normal_distribution<double> noise(0.0, lidar.noise_stddev);
    const auto offsets = amr::beamOffsets(lidar);
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        const auto& s = trajectory[k].state;
        for (std::size_t i = 0; i < offsets.size(); ++i) {
            const amr::RayHit ray = amr::castRay(map, {s.x, s.y}, s.theta + offsets[i], lidar.max_range);
            const double expected =
                ray.hit ? std::clamp(ray.range + noise(rng), 0.0, lidar.max_range) : ray.range;
            CHECK_EQ(scans[k].ranges[i], expected);
        }
    }
}

// ---------------------------------------------------------------------------------
// Augmented EKF (ekf.hpp)

void testAugmentedEqualsClassic() {
    const GridMap map = room();
    const auto trajectory = driveThrough(map, {1, 1}, {18, 18});
    LidarConfig lidar;
    lidar.beams = 72;
    lidar.noise_stddev = 0.03;
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    EkfConfig classic;
    classic.enabled = true;
    classic.range_sigma = 0.03;
    classic.model.grazing = amr::GrazingPolicy::Cutoff;  // the classic rule
    EkfConfig augmented = classic;
    augmented.model.max_incidence += 1e-12;  // not the classic model any more
    REQUIRE(classic.model.classic());
    REQUIRE(!augmented.model.classic());
    const auto a = amr::runLocalization(map, trajectory, scans, lidar, classic);
    const auto b = amr::runLocalization(map, trajectory, scans, lidar, augmented);
    REQUIRE(a.size() == b.size());
    for (std::size_t k = 0; k < a.size(); ++k) {
        CHECK_NEAR(a[k].estimate.x, b[k].estimate.x, 1e-9);
        CHECK_NEAR(a[k].estimate.y, b[k].estimate.y, 1e-9);
        CHECK_NEAR(a[k].estimate.theta, b[k].estimate.theta, 1e-9);
        CHECK_NEAR(a[k].covariance[0], b[k].covariance[0], 1e-12);
        CHECK_EQ(a[k].update.used, b[k].update.used);
        CHECK_EQ(a[k].update.rejected, b[k].update.rejected);
    }
}

/// A range bias shared by all beams makes the independent-beam filter overconfident;
/// a bias state restores consistency and estimates the bias.
void testBiasStateRestoresConsistency() {
    const GridMap map = room();
    const auto trajectory = driveThrough(map, {1, 1}, {18, 18});
    double classic_nees = 0.0, bias_nees = 0.0, bias_error = 0.0;
    const int runs = 16;
    for (int run = 0; run < runs; ++run) {
        LidarConfig lidar;
        lidar.beams = 72;
        lidar.noise_stddev = 0.02;
        lidar.bias_stddev = 0.03;
        lidar.seed = static_cast<std::uint32_t>(100 + run);
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        amr::RangeErrorModel truth(lidar);
        truth.nextScan(0.0);
        EkfConfig config;
        config.enabled = true;
        config.range_sigma = 0.02;
        config.odometry.seed = static_cast<std::uint32_t>(run);
        classic_nees += meanNees(amr::runLocalization(map, trajectory, scans, lidar, config));
        config.model.bias_sigma = 0.03;
        const auto samples = amr::runLocalization(map, trajectory, scans, lidar, config);
        bias_nees += meanNees(samples);
        bias_error += std::fabs(samples.back().bias - truth.bias());
    }
    classic_nees /= runs;
    bias_nees /= runs;
    std::cout << "    mean NEES with a 3 cm bias: independent beams " << classic_nees
              << ", bias state " << bias_nees << "; bias error " << 1000.0 * bias_error / runs
              << " mm\n";
    CHECK(classic_nees > 20.0);  // measured ~ 600
    CHECK(bias_nees > 2.4 && bias_nees < 4.5);
    CHECK(bias_error / runs < 0.003);
}

/// A continuous world (shifted, slightly rotated boxes) rasterized at 0.1 m: the map is
/// wrong by up to half a cell, wall by wall. Independent beams are wildly overconfident;
/// wall map-error states on the wall-line surface are close to consistent.
struct ContinuousCase {
    GridMap map;
    std::vector<amr::Polygon> truth;
};

ContinuousCase rasterizedRoom(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> shift(-0.045, 0.045);
    std::uniform_real_distribution<double> turn(-0.08, 0.08);
    std::vector<amr::Polygon> truth;
    for (const amr::Box& b : {amr::Box{1.5, 1.0, 3.0, 2.0}, amr::Box{4.0, 3.0, 6.0, 4.0},
                              amr::Box{7.0, 6.0, 8.0, 9.0}, amr::Box{1.0, 6.5, 3.5, 7.5}}) {
        truth.push_back(amr::makeRectangle({0.5 * (b.x0 + b.x1) + shift(rng), 0.5 * (b.y0 + b.y1) + shift(rng)},
                                           b.x1 - b.x0, b.y1 - b.y0, turn(rng)));
    }
    truth.push_back(amr::makeRectangle({-1.0 + shift(rng), 5.0}, 2.0, 14.0, 0.0));
    truth.push_back(amr::makeRectangle({11.0 + shift(rng), 5.0}, 2.0, 14.0, 0.0));
    truth.push_back(amr::makeRectangle({5.0, -1.0 + shift(rng)}, 14.0, 2.0, 0.0));
    truth.push_back(amr::makeRectangle({5.0, 11.0 + shift(rng)}, 14.0, 2.0, 0.0));
    return {amr::rasterize(truth, 100, 100, 0.1, amr::RasterRule::Majority), truth};
}

void testWallStatesOnARasterizedWorld() {
    double classic = 0.0, walls = 0.0, classic_rms = 0.0, walls_rms = 0.0;
    const int runs = 6;
    for (int run = 0; run < runs; ++run) {
        const ContinuousCase world = rasterizedRoom(static_cast<unsigned>(run));
        const auto trajectory = driveThrough(world.map, {5, 5}, {90, 90}, 0.3);
        REQUIRE(!trajectory.empty());
        LidarConfig lidar;
        lidar.beams = 72;
        lidar.noise_stddev = 0.02;
        lidar.seed = static_cast<std::uint32_t>(run);
        const auto scans = amr::scanTrajectoryWith(
            [&](WorldPoint o, double angle, double max_range, double) {
                double best = max_range;
                for (const auto& p : world.truth) {
                    best = std::min(best, amr::rayToPolygon(p, o, angle));
                }
                return amr::BeamRange{best < max_range, best};
            },
            trajectory, lidar);
        EkfConfig config;
        config.enabled = true;
        config.range_sigma = 0.02;
        config.odometry.seed = static_cast<std::uint32_t>(run);
        const auto rms = [](const std::vector<amr::LocalizationSample>& samples) {
            double sq = 0.0;
            for (const auto& s : samples) {
                sq += std::pow(s.truth.x - s.estimate.x, 2) + std::pow(s.truth.y - s.estimate.y, 2);
            }
            return std::sqrt(sq / static_cast<double>(samples.size()));
        };
        const auto a = amr::runLocalization(world.map, trajectory, scans, lidar, config);
        config.model.surface = amr::SurfaceKind::Lines;
        config.model.map_sigma = 0.1 / std::sqrt(12.0);
        const auto b = amr::runLocalization(world.map, trajectory, scans, lidar, config);
        classic += meanNees(a);
        walls += meanNees(b);
        classic_rms += rms(a);
        walls_rms += rms(b);
        CHECK(b.back().map_states > 0);
    }
    std::cout << "    rasterized world (0.1 m): mean NEES independent " << classic / runs
              << ", wall states " << walls / runs << "; position RMS " << 100.0 * classic_rms / runs
              << " cm -> " << 100.0 * walls_rms / runs << " cm\n";
    CHECK(classic / runs > 100.0);
    CHECK(walls / runs < 8.0);
    CHECK(walls_rms < classic_rms);
}

void testValidation() {
    EkfConfig config;
    config.enabled = true;
    CHECK(amr::validateEkfConfig(config).empty());
    auto bad = config;
    bad.model.bias_sigma = -1.0;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.model.bias_time = -1.0;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.model.map_sigma = -0.1;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.model.map_tilt = std::nan("");
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.model.max_map_states = 1;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.model.max_incidence = 0.0;
    CHECK(!amr::validateEkfConfig(bad).empty());
    LidarConfig lidar;
    lidar.beams = 8;
    lidar.bias_stddev = -0.1;
    CHECK(!amr::validateLidarConfig(lidar).empty());
    lidar.bias_stddev = 0.1;
    lidar.bias_time = -1.0;
    CHECK(!amr::validateLidarConfig(lidar).empty());
}

// ---------------------------------------------------------------------------------
// Wall-line surface (surface_model.hpp)

void testBoundaryChains() {
    // One block (2 x 3 cells) and a diagonal pair (one chain: diagonal cells touch).
    const GridMap map = makeMap({"........", ".##.....", ".##.....", ".##..#..", "......#.", "........"}, 1.0);
    const auto chains = amr::boundaryChains(map);
    // The map border (inner side of the outside), the block, the diagonal pair.
    REQUIRE(chains.size() == 3);
    std::vector<std::size_t> sizes;
    for (const auto& c : chains) {
        sizes.push_back(c.size());
    }
    std::sort(sizes.begin(), sizes.end());
    CHECK_EQ(sizes[0], 8u);   // two diagonal cells: 4 + 4 faces in one chain
    CHECK_EQ(sizes[1], 10u);  // 2 x 3 block: perimeter 10
    CHECK_EQ(sizes[2], 28u);  // 8 x 6 map border
}

void testAxisAlignedWallsAreExact() {
    // On a grid of axis-aligned rectangles the fitted lines are the cell faces, and
    // the line surface gives the grid's ranges (except at exact corners).
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(std::string(AMR_SOURCE_DIR) + "/maps/obstacles.csv", 0.5);
    REQUIRE(loaded.ok);
    const amr::LineSurface lines(loaded.map);
    const amr::GridSurface grid(loaded.map);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> coordinate(0.05, 9.95);
    std::uniform_real_distribution<double> angle(-kPi, kPi);
    int compared = 0;
    for (int k = 0; k < 3000; ++k) {
        const WorldPoint o{coordinate(rng), coordinate(rng)};
        if (!loaded.map.isFree(loaded.map.worldToGrid(o))) {
            continue;
        }
        const double a = angle(rng);
        const amr::SurfaceHit g = grid.cast(o, a, 20.0);
        const amr::SurfaceHit l = lines.cast(o, a, 20.0);
        if (!g.smooth) {
            continue;
        }
        CHECK(l.hit);
        CHECK_NEAR(l.range, g.range, 1e-9);
        CHECK_NEAR(l.normal.x, g.normal.x, 1e-9);
        CHECK_NEAR(l.normal.y, g.normal.y, 1e-9);
        ++compared;
    }
    CHECK(compared > 2000);
}

/// A slanted wall rasterized at 0.1 m: the grid's ranges carry the staircase error and
/// jumping normals; the fitted lines follow the true wall.
void testSlantedWallIsRecovered() {
    for (const double degrees : {3.0, 6.0, 17.0, 33.0}) {
        const amr::Polygon wall = amr::makeRectangle({5.03, 6.47}, 8.0, 2.0, degrees * kPi / 180.0);
        const GridMap map = amr::rasterize({wall}, 100, 100, 0.1, amr::RasterRule::Majority);
        const amr::LineSurface lines(map);
        const amr::GridSurface grid(map);
        double line_sq = 0.0, grid_sq = 0.0, line_dydx = 0.0, grid_dydx = 0.0;
        int n = 0;
        double previous_line = 0.0, previous_grid = 0.0;
        // Beams 20 deg off vertical, towards the risers: on the grid they alternately
        // hit treads (dr/dy = -1/sin) and risers (dr/dy = 0); on the fitted line dr/dy
        // stays constant.
        const double beam = kPi / 2 + 0.35;
        for (double x = 3.0; x <= 6.0; x += 0.01) {
            const WorldPoint o{x, 2.0};
            const double truth = amr::rayToPolygon(wall, o, beam);
            const amr::SurfaceHit l = lines.cast(o, beam, 9.0);
            const amr::SurfaceHit g = grid.cast(o, beam, 9.0);
            line_sq += std::pow(l.range - truth, 2);
            grid_sq += std::pow(g.range - truth, 2);
            const double jl = amr::surfaceJacobian(l, beam)[1];
            const double jg = g.smooth ? amr::surfaceJacobian(g, beam)[1] : previous_grid;
            if (n > 0) {
                line_dydx = std::max(line_dydx, std::fabs(jl - previous_line));
                grid_dydx = std::max(grid_dydx, std::fabs(jg - previous_grid));
            }
            previous_line = jl;
            previous_grid = jg;
            ++n;
        }
        const double line_rms = std::sqrt(line_sq / n);
        const double grid_rms = std::sqrt(grid_sq / n);
        std::cout << "    " << degrees << " deg wall: range RMS lines " << 1000.0 * line_rms
                  << " mm, grid " << 1000.0 * grid_rms << " mm; largest Jacobian jump lines "
                  << line_dydx << ", grid " << grid_dydx << '\n';
        CHECK(line_rms < 0.006);
        CHECK(line_rms < 0.3 * grid_rms);
        // Along the wall the line surface's derivative does not jump (one segment);
        // the grid's jumps between 0 and the staircase steps' 1/cos.
        CHECK(line_dydx < 1e-9);
        CHECK(grid_dydx > 0.5);
    }
}

/// The range Jacobian of either surface matches finite differences of its own cast.
void testSurfaceJacobians() {
    const ContinuousCase world = rasterizedRoom(4);
    const amr::LineSurface lines(world.map);
    const amr::GridSurface grid(world.map);
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> coordinate(0.5, 9.5);
    std::uniform_real_distribution<double> angle(-kPi, kPi);
    int checked = 0;
    for (int k = 0; k < 4000 && checked < 600; ++k) {
        const WorldPoint o{coordinate(rng), coordinate(rng)};
        const double a = angle(rng);
        for (const amr::SurfaceModel* surface : {static_cast<const amr::SurfaceModel*>(&lines),
                                                 static_cast<const amr::SurfaceModel*>(&grid)}) {
            const amr::SurfaceHit hit = surface->cast(o, a, 20.0);
            if (!hit.hit || !hit.smooth || hit.range <= 0.0 || amr::incidenceCosine(hit, a) < 0.3) {
                continue;
            }
            const auto j = amr::surfaceJacobian(hit, a);
            const double h = 1e-6;
            const amr::SurfaceHit dx = surface->cast({o.x + h, o.y}, a, 20.0);
            const amr::SurfaceHit dy = surface->cast({o.x, o.y + h}, a, 20.0);
            const amr::SurfaceHit dt = surface->cast(o, a + h, 20.0);
            // Skip beams that cross onto another face within the step.
            if (dx.feature != hit.feature || dy.feature != hit.feature || dt.feature != hit.feature ||
                std::fabs(dx.normal.x - hit.normal.x) + std::fabs(dy.normal.y - hit.normal.y) +
                        std::fabs(dt.normal.x - hit.normal.x) > 1e-12) {
                continue;
            }
            CHECK_NEAR((dx.range - hit.range) / h, j[0], 1e-4 * (1.0 + std::fabs(j[0])));
            CHECK_NEAR((dy.range - hit.range) / h, j[1], 1e-4 * (1.0 + std::fabs(j[1])));
            CHECK_NEAR((dt.range - hit.range) / h, j[2], 1e-3 * (1.0 + std::fabs(j[2])));
            ++checked;
        }
    }
    CHECK(checked >= 600);
}


// ---------------------------------------------------------------------------------
// Grazing beams (ekf.hpp, GrazingPolicy)

/// The range Hessian (used for the linearization variance) matches finite differences
/// of the Jacobian on a flat face, including near grazing.
void testRangeHessian() {
    const GridMap map = makeMap({"..........", "..........", "..........", "......####",
                                 "......####", "......####", ".........."}, 1.0);
    const amr::GridSurface grid(map);
    for (const double angle : {0.1, 0.6, 1.2, 1.45, -0.3}) {
        const WorldPoint o{1.3, 4.4};
        const amr::SurfaceHit hit = grid.cast(o, angle, 30.0);
        if (!hit.smooth) {
            continue;
        }
        const auto g = amr::surfaceHessian(hit, angle);
        const double h = 1e-6;
        const auto j0 = amr::surfaceJacobian(hit, angle);
        const auto jx = amr::surfaceJacobian(grid.cast({o.x + h, o.y}, angle, 30.0), angle);
        const auto jy = amr::surfaceJacobian(grid.cast({o.x, o.y + h}, angle, 30.0), angle);
        const auto jt = amr::surfaceJacobian(grid.cast(o, angle + h, 30.0), angle + h);
        for (int r = 0; r < 3; ++r) {
            const double scale = 1e-4 * (1.0 + std::fabs(g[static_cast<std::size_t>(3 * r)]) +
                                         std::fabs(g[static_cast<std::size_t>(3 * r + 2)]));
            CHECK_NEAR((jx[static_cast<std::size_t>(r)] - j0[static_cast<std::size_t>(r)]) / h, g[static_cast<std::size_t>(3 * r)], scale);
            CHECK_NEAR((jy[static_cast<std::size_t>(r)] - j0[static_cast<std::size_t>(r)]) / h, g[static_cast<std::size_t>(3 * r + 1)], scale);
            CHECK_NEAR((jt[static_cast<std::size_t>(r)] - j0[static_cast<std::size_t>(r)]) / h, g[static_cast<std::size_t>(3 * r + 2)], scale * 10.0);
        }
    }
}

/// On the grid world (exact map) the grazing model keeps every beam and stays
/// consistent; raising the cutoff without modeling makes the filter overconfident.
void testGrazingModelKeepsBeamsConsistently() {
    const GridMap map = room();
    const auto trajectory = driveThrough(map, {1, 1}, {18, 18});
    double model_nees = 0.0, open_nees = 0.0, classic_nees = 0.0;
    std::size_t model_beams = 0, classic_beams = 0;
    const int runs = 12;
    for (int run = 0; run < runs; ++run) {
        LidarConfig lidar;
        lidar.beams = 72;
        lidar.noise_stddev = 0.02;
        lidar.seed = static_cast<std::uint32_t>(300 + run);
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        EkfConfig config;
        config.enabled = true;
        config.range_sigma = 0.02;
        config.odometry.seed = static_cast<std::uint32_t>(run);
        const auto model = amr::runLocalization(map, trajectory, scans, lidar, config);
        config.model.grazing = amr::GrazingPolicy::Cutoff;
        const auto classic = amr::runLocalization(map, trajectory, scans, lidar, config);
        config.model.max_incidence = 89.0 * kPi / 180.0;
        const auto open = amr::runLocalization(map, trajectory, scans, lidar, config);
        model_nees += meanNees(model);
        classic_nees += meanNees(classic);
        open_nees += meanNees(open);
        for (std::size_t k = 0; k < model.size(); ++k) {
            model_beams += model[k].update.used;
            classic_beams += classic[k].update.used;
        }
    }
    std::cout << "    mean NEES: grazing model " << model_nees / runs << ", cutoff 78.5 deg "
              << classic_nees / runs << ", cutoff 89 deg " << open_nees / runs << "; beams "
              << model_beams << " vs " << classic_beams << '\n';
    CHECK(model_nees / runs > 2.4 && model_nees / runs < 5.0);
    CHECK(open_nees / runs > 3.0 * model_nees / runs);
    CHECK(model_beams > classic_beams);
}

/// A grazing beam whose hit lies near the end of a wall is skipped by the model
/// policy (it could slide off the wall); the same incidence mid-wall is used.
void testGrazingNearAWallEnd() {
    // One long wall (row 5, columns 2..17) in an otherwise open map.
    std::vector<std::string> rows(12, std::string(20, '.'));
    for (int x = 2; x < 18; ++x) {
        rows[5][static_cast<std::size_t>(x)] = '#';
    }
    const GridMap map = makeMap(rows, 0.5);
    const amr::LineSurface lines(map);
    amr::MeasurementModelConfig model;
    model.surface = amr::SurfaceKind::Lines;
    model.map_sigma = 0.05;
    const amr::Matrix3 p{1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-6};
    // From y = 1.0 the wall's top face (y = 2.5) is 1.5 m away; at 70 deg incidence the
    // beam travels 1.5 tan(70) = 4.1 m along the wall. The wall spans x = 1 .. 9 m: aim
    // at x = 8.9 (0.1 m from its end, less than the 2 sigma tan(70) = 0.28 m slide) and
    // at x = 5.6 (mid-wall).
    const double incidence = 70.0 * kPi / 180.0;
    const double along = 1.5 * std::tan(incidence);
    for (const auto& [target, expect_used] : {std::pair{8.9, 0u}, std::pair{5.6, 1u}}) {
        const WorldPoint origin{target - along, 1.0};
        const double angle = kPi / 2 - incidence;  // down and to the right
        const amr::SurfaceHit hit = lines.cast(origin, angle, 20.0);
        REQUIRE(hit.hit);
        amr::AugmentedEkf ekf({origin.x, origin.y, 0.0}, p, lines, model, map.resolution());
        const auto stats = ekf.update({angle}, {hit.range}, {true}, 20.0, 0.02, 3.0);
        CHECK_EQ(stats.used, expect_used);
    }
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Range bias: constant, or Gauss-Markov with the given std and time", testBiasProcess},
        {"Range bias shifts every beam of a scan alike", testBiasShiftsEveryBeamOfAScan},
        {"Without a bias the scans are unchanged", testNoBiasKeepsTheOriginalScans},
        {"Augmented EKF without nuisance states equals the classic filter", testAugmentedEqualsClassic},
        {"Bias state restores consistency under a range bias", testBiasStateRestoresConsistency},
        {"Wall states on a rasterized world", testWallStatesOnARasterizedWorld},
        {"Measurement-model validation", testValidation},
        {"Boundary chains of the grid's obstacles", testBoundaryChains},
        {"Axis-aligned walls: line surface equals the grid", testAxisAlignedWallsAreExact},
        {"Slanted walls: lines recover the wall, grid keeps the staircase", testSlantedWallIsRecovered},
        {"Surface Jacobians vs. finite differences", testSurfaceJacobians},
        {"Range Hessian vs. finite differences", testRangeHessian},
        {"Grazing model keeps every beam, consistently", testGrazingModelKeepsBeamsConsistently},
        {"Grazing beam near a wall end is skipped", testGrazingNearAWallEnd},
    });
}
