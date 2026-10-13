// Tests for the simulated 2D LiDAR: exact ray casting, beam layout, noise and
// scans along a trajectory.
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::GridMap;
using amr::LidarConfig;
using amr::RayHit;
using amr::WorldPoint;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

GridMap makeMap(const std::vector<std::string>& rows, double resolution = 1.0) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                   resolution, std::move(cells));
}

void testAnalyticDistances() {
    // 1 m cells; obstacle column at x = 5 (cells span x in [5, 6]).
    const GridMap map = makeMap({".....#....", ".....#....", ".....#....", ".....#....",
                                 ".....#...."});
    const WorldPoint origin{1.5, 2.5};
    RayHit east = amr::castRay(map, origin, 0.0, 20.0);
    CHECK(east.hit);
    CHECK_NEAR(east.range, 3.5, kTolerance);                 // to the face x = 5
    RayHit west = amr::castRay(map, origin, kPi, 20.0);
    CHECK(west.hit);
    CHECK_NEAR(west.range, 1.5, kTolerance);                 // to the map border x = 0
    RayHit south = amr::castRay(map, origin, kPi / 2, 20.0); // +y is down the map
    CHECK_NEAR(south.range, 2.5, kTolerance);                // border y = 5
    // 30 deg below +x: reaches x = 5 after 3.5 / cos(30 deg).
    RayHit angled = amr::castRay(map, origin, kPi / 6, 20.0);
    CHECK_NEAR(angled.range, 3.5 / std::cos(kPi / 6), kTolerance);
    // Out of range: no return, range reported as the maximum.
    RayHit short_range = amr::castRay(map, origin, 0.0, 3.0);
    CHECK(!short_range.hit);
    CHECK_NEAR(short_range.range, 3.0, kTolerance);
    // Exactly at the range limit counts as a hit.
    CHECK(amr::castRay(map, origin, 0.0, 3.5).hit);
}

void testCornerRays() {
    // A 45 deg ray from a cell center passes exactly through cell corners.
    // Obstacle diagonally ahead: hit at the shared corner.
    const GridMap diagonal = makeMap({"...", "...", "..#"});
    const RayHit hit = amr::castRay(diagonal, {0.5, 0.5}, kPi / 4, 10.0);
    CHECK(hit.hit);
    CHECK_NEAR(hit.range, std::sqrt(2.0) * 1.5, 1e-9);       // corner (2, 2)
    // Obstacle only beside the corner the ray grazes: touching counts as a hit.
    const GridMap side = makeMap({"...", ".#.", "..."});
    const RayHit graze = amr::castRay(side, {0.5, 1.5}, -kPi / 4, 10.0);  // up-right
    CHECK(graze.hit);
    CHECK_NEAR(graze.range, std::sqrt(2.0) * 0.5, 1e-9);     // corner (1, 1) of the obstacle
}

void testSensorInsideObstacle() {
    const GridMap map = makeMap({"#.."});
    const RayHit hit = amr::castRay(map, {0.5, 0.5}, 0.0, 5.0);
    CHECK(hit.hit);
    CHECK_NEAR(hit.range, 0.0, kTolerance);
}

/// Independent exact reference: the nearest entry of the ray into any occupied
/// cell square (closed, slab test) or into the area outside the map.
double slabRange(const GridMap& map, WorldPoint origin, double angle, double max_range) {
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    const double res = map.resolution();
    double best = max_range;
    const auto entry = [&](double x0, double y0, double x1, double y1) {
        double t_min = 0.0;
        double t_max = max_range;
        const double o[2] = {origin.x, origin.y};
        const double d[2] = {dx, dy};
        const double lo[2] = {x0, y0};
        const double hi[2] = {x1, y1};
        for (int a = 0; a < 2; ++a) {
            if (d[a] == 0.0) {
                if (o[a] < lo[a] || o[a] > hi[a]) {
                    return;
                }
                continue;
            }
            double t1 = (lo[a] - o[a]) / d[a];
            double t2 = (hi[a] - o[a]) / d[a];
            if (t1 > t2) {
                std::swap(t1, t2);
            }
            t_min = std::max(t_min, t1);
            t_max = std::min(t_max, t2);
            if (t_min > t_max) {
                return;
            }
        }
        best = std::min(best, t_min);
    };
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (map.isOccupied({x, y})) {
                entry(x * res, y * res, (x + 1) * res, (y + 1) * res);
            }
        }
    }
    // Leaving the map: the first border crossing.
    const double w = map.width() * res;
    const double h = map.height() * res;
    if (dx > 0) best = std::min(best, (w - origin.x) / dx);
    if (dx < 0) best = std::min(best, -origin.x / dx);
    if (dy > 0) best = std::min(best, (h - origin.y) / dy);
    if (dy < 0) best = std::min(best, -origin.y / dy);
    return best;
}

/// Brute-force sampling (1 mm): the first sampled point in an occupied cell or off
/// the map. Sampling can step over a thin sliver of an obstacle, so it can only be
/// LATER than the exact range, never earlier.
double sampledRange(const GridMap& map, WorldPoint origin, double angle, double max_range) {
    const double step = 0.001;
    for (double t = 0.0; t <= max_range; t += step) {
        const WorldPoint p{origin.x + t * std::cos(angle), origin.y + t * std::sin(angle)};
        if (!map.isFree(map.worldToGrid(p))) {
            return t;
        }
    }
    return max_range;
}

void testAgainstBruteForce() {
    std::mt19937 rng(17);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    int checked = 0;
    for (int trial = 0; trial < 60; ++trial) {
        const int width = 5 + trial % 20;
        const int height = 5 + (trial * 7) % 20;
        const double res = trial % 2 ? 0.5 : 0.25;
        std::bernoulli_distribution occupied(0.25);
        std::vector<std::uint8_t> cells(static_cast<std::size_t>(width * height));
        for (auto& c : cells) {
            c = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
        }
        const GridMap map(width, height, res, std::move(cells));
        for (int ray = 0; ray < 40; ++ray) {
            const WorldPoint origin{unit(rng) * width * res, unit(rng) * height * res};
            if (!map.isFree(map.worldToGrid(origin))) {
                continue;
            }
            const double angle = unit(rng) * 2.0 * kPi - kPi;
            const double max_range = 6.0;
            const RayHit hit = amr::castRay(map, origin, angle, max_range);
            // Exact agreement with the independent slab-test reference.
            const double reference = slabRange(map, origin, angle, max_range);
            CHECK_NEAR(hit.range, reference, 1e-9);
            CHECK(hit.hit == (reference < max_range));
            // Never later than sampling (which can only miss obstacles).
            CHECK(hit.range <= sampledRange(map, origin, angle, max_range) + 1e-9);
            if (hit.hit) {
                // Just before the hit point the ray is in free space; at it, an
                // obstacle cell (or the border) is touched.
                const double before = std::max(0.0, hit.range - 1e-6);
                const WorldPoint p{origin.x + before * std::cos(angle),
                                   origin.y + before * std::sin(angle)};
                CHECK(map.isFree(map.worldToGrid(p)));
            }
            ++checked;
        }
    }
    CHECK(checked > 1500);
}

void testBeamLayout() {
    LidarConfig full;
    full.beams = 8;
    const auto offsets = amr::beamOffsets(full);
    REQUIRE(offsets.size() == 8);
    CHECK_NEAR(offsets.front(), -kPi, kTolerance);
    CHECK_NEAR(offsets[4], 0.0, kTolerance);                 // one beam straight ahead
    CHECK_NEAR(offsets.back(), kPi - kPi / 4, kTolerance);   // no duplicate at +pi
    LidarConfig partial;
    partial.beams = 5;
    partial.fov = kPi / 2;
    const auto narrow = amr::beamOffsets(partial);
    REQUIRE(narrow.size() == 5);
    CHECK_NEAR(narrow.front(), -kPi / 4, kTolerance);        // ends of the field of view
    CHECK_NEAR(narrow.back(), kPi / 4, kTolerance);
    CHECK_NEAR(narrow[2], 0.0, kTolerance);
    LidarConfig single;
    single.beams = 1;
    single.fov = kPi / 3;
    CHECK_NEAR(amr::beamOffsets(single).front(), 0.0, kTolerance);
}

void testScansFollowTheTrajectory() {
    const GridMap map(20, 10, 0.5);
    const auto plan = amr::planAStar(map, {1, 1}, {18, 1});
    REQUIRE(plan.success());
    amr::MotionConfig config;
    config.model = amr::RobotModel::DiffDrive;
    config.initial_heading = kPi / 2;  // facing down the map: turns first
    const auto sim = amr::simulatePath(map, plan.path, config);
    LidarConfig lidar;
    lidar.beams = 4;
    lidar.max_range = 20.0;
    const auto scans = amr::scanTrajectory(map, sim.trajectory, lidar);
    REQUIRE(scans.size() == sim.trajectory.size());
    for (std::size_t i = 0; i < scans.size(); ++i) {
        CHECK_EQ(scans[i].time, sim.trajectory[i].state.time);
        CHECK_EQ(scans[i].origin.x, sim.trajectory[i].state.x);
        // Beam 2 (offset 0) points along the heading.
        CHECK_NEAR(scans[i].angles[2], sim.trajectory[i].state.theta, kTolerance);
    }
    // At t = 0 the robot at (0.75, 0.75) faces +y: the forward beam hits the
    // bottom border 5 - 0.75 = 4.25 m away; the backward beam the top border.
    CHECK_NEAR(scans.front().ranges[2], 4.25, kTolerance);
    CHECK_NEAR(scans.front().ranges[0], 0.75, kTolerance);
    // At the end it faces +x at (9.25, 0.75): 0.75 m to the right border.
    CHECK_NEAR(scans.back().ranges[2], 0.75, kTolerance);
    // Disabled LiDAR: no scans.
    CHECK(amr::scanTrajectory(map, sim.trajectory, LidarConfig{}).empty());
}

void testNoise() {
    const GridMap map(40, 40, 0.5);
    std::vector<amr::TrajectorySample> trajectory(2000);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        trajectory[i].state = {10.25, 10.25, 0.1 * static_cast<double>(i), 0.0};
    }
    LidarConfig lidar;
    lidar.beams = 4;
    lidar.max_range = 20.0;
    lidar.noise_stddev = 0.05;
    lidar.seed = 7;
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    // Errors against the exact (noise-free) range of each beam.
    double sum = 0.0;
    double sum_sq = 0.0;
    int n = 0;
    for (const auto& scan : scans) {
        for (std::size_t b = 0; b < scan.ranges.size(); ++b) {
            const double exact =
                amr::castRay(map, scan.origin, scan.angles[b], lidar.max_range).range;
            const double e = scan.ranges[b] - exact;
            sum += e;
            sum_sq += e * e;
            ++n;
        }
    }
    const double mean = sum / n;
    const double stddev = std::sqrt(sum_sq / n - mean * mean);
    CHECK(std::fabs(mean) < 0.003);               // 8000 samples: |mean| << 4 sigma/sqrt(n)
    CHECK(std::fabs(stddev - 0.05) < 0.003);
    // Same seed, same scans; different seed, different scans.
    const auto again = amr::scanTrajectory(map, trajectory, lidar);
    CHECK_EQ(again[123].ranges[1], scans[123].ranges[1]);
    lidar.seed = 8;
    CHECK(amr::scanTrajectory(map, trajectory, lidar)[123].ranges[1] != scans[123].ranges[1]);
    // Noise never produces a negative range or a range beyond the maximum.
    lidar.noise_stddev = 50.0;
    for (const auto& scan : amr::scanTrajectory(map, trajectory, lidar)) {
        for (const double range : scan.ranges) {
            CHECK(range >= 0.0 && range <= lidar.max_range);
        }
    }
}

void testValidation() {
    LidarConfig config;
    CHECK(amr::validateLidarConfig(config).empty());   // disabled
    config.beams = 72;
    CHECK(amr::validateLidarConfig(config).empty());
    config.beams = 4000;
    CHECK(!amr::validateLidarConfig(config).empty());
    config = LidarConfig{};
    config.beams = 8;
    config.fov = 0.0;
    CHECK(!amr::validateLidarConfig(config).empty());
    config.fov = 7.0;  // > 2 pi
    CHECK(!amr::validateLidarConfig(config).empty());
    config = LidarConfig{};
    config.beams = 8;
    config.max_range = -1.0;
    CHECK(!amr::validateLidarConfig(config).empty());
    config = LidarConfig{};
    config.beams = 8;
    config.noise_stddev = std::nan("");
    CHECK(!amr::validateLidarConfig(config).empty());
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Exact distances: faces, border, angled beam, max range", testAnalyticDistances},
        {"Rays through cell corners", testCornerRays},
        {"Sensor inside an obstacle", testSensorInsideObstacle},
        {"Exact vs. slab-test reference; never later than sampling", testAgainstBruteForce},
        {"Beam layout: full circle, partial FOV, single beam", testBeamLayout},
        {"Scans follow the trajectory and heading", testScansFollowTheTrajectory},
        {"Range noise: statistics, determinism, clamping", testNoise},
        {"Configuration validation", testValidation},
    });
}
