// Tests for the circular robot footprint: disc geometry, obstacle inflation,
// footprint-aware planning and disc collision checking in the simulator.
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::GridCoord;
using amr::GridMap;
using amr::MotionConfig;
using amr::NavigationStatus;
using amr::PlanStatus;
using amr::WorldPoint;

namespace {

/// Build a map from text rows: '.' = free, '#' = occupied. Row 0 is the top.
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

/// 5 x 5 map, 1 m cells, one obstacle covering the square [2, 3] x [2, 3].
GridMap centerObstacleMap() {
    return makeMap({".....", ".....", "..#..", ".....", "....."}, 1.0);
}

std::size_t occupiedCount(const GridMap& map) {
    std::size_t count = 0;
    for (std::size_t i = 0; i < map.cellCount(); ++i) {
        count += map.data()[i];
    }
    return count;
}

void testDiscGeometry() {
    const GridMap map = centerObstacleMap();
    // Side neighbor center (1.5, 2.5): 0.5 m from the obstacle's left edge.
    CHECK(!amr::discOverlapsObstacle(map, {1.5, 2.5}, 0.5));    // touching is allowed
    CHECK(amr::discOverlapsObstacle(map, {1.5, 2.5}, 0.5001));
    // Diagonal neighbor center (1.5, 1.5): sqrt(0.5) = 0.7071 m from the corner (2, 2).
    CHECK(!amr::discOverlapsObstacle(map, {1.5, 1.5}, 0.70));
    CHECK(amr::discOverlapsObstacle(map, {1.5, 1.5}, 0.71));
    // Map border counts as an obstacle: (0.5, 0.5) is 0.5 m from two borders.
    CHECK(!amr::discOverlapsObstacle(map, {0.5, 0.5}, 0.5));
    CHECK(amr::discOverlapsObstacle(map, {0.5, 0.5}, 0.51));
    CHECK(amr::discOverlapsObstacle(map, {4.6, 0.5}, 0.5));    // right border
    // Center inside the obstacle.
    CHECK(amr::discOverlapsObstacle(map, {2.5, 2.5}, 0.01));
}

void testSweptDiscIsExact() {
    const GridMap map = centerObstacleMap();
    // Horizontal pass 0.5 m above the obstacle's top edge (y = 2).
    CHECK(!amr::sweptDiscOverlapsObstacle(map, {0.6, 1.5}, {4.4, 1.5}, 0.5));
    CHECK(amr::sweptDiscOverlapsObstacle(map, {0.6, 1.5}, {4.4, 1.5}, 0.55));
    // Both endpoints are clear (distances 1.58 m and 0.5 m), but the segment passes
    // through the obstacle's corner (3, 2): an endpoint-only check would miss this.
    const WorldPoint a{1.5, 0.5};
    const WorldPoint b{3.5, 2.5};
    CHECK(!amr::discOverlapsObstacle(map, a, 0.3));
    CHECK(!amr::discOverlapsObstacle(map, b, 0.3));
    CHECK(amr::sweptDiscOverlapsObstacle(map, a, b, 0.3));
    // Diagonal pass near the corner (2, 2): the endpoints are 0.5 m from the
    // obstacle, the midpoint (1.75, 1.75) only sqrt(2) / 4 = 0.3536 m.
    CHECK(!amr::sweptDiscOverlapsObstacle(map, {1.5, 2.0}, {2.0, 1.5}, 0.35));
    CHECK(amr::sweptDiscOverlapsObstacle(map, {1.5, 2.0}, {2.0, 1.5}, 0.36));
}

void testInflationRings() {
    const GridMap map = centerObstacleMap();
    const GridMap same = amr::inflateObstacles(map, 0.0);
    CHECK_EQ(occupiedCount(same), static_cast<std::size_t>(1));

    // r = 0.5: neighbors only touch the obstacle and the border cells only touch
    // the border, so nothing is inflated.
    CHECK_EQ(occupiedCount(amr::inflateObstacles(map, 0.5)), static_cast<std::size_t>(1));

    // r = 0.6: the 4 side neighbors and the 16 border cells are inflated,
    // the 4 diagonal neighbors (0.707 m away) are not.
    const GridMap side = amr::inflateObstacles(map, 0.6);
    CHECK_EQ(occupiedCount(side), static_cast<std::size_t>(1 + 4 + 16));
    CHECK(side.isOccupied({2, 1}));
    CHECK(side.isOccupied({0, 4}));
    CHECK(side.isFree({1, 1}));

    // r = 0.75: diagonal neighbors are inflated too; everything is blocked.
    CHECK_EQ(occupiedCount(amr::inflateObstacles(map, 0.75)), static_cast<std::size_t>(25));
}

void testInflationIsMonotonic() {
    std::mt19937 rng(11);
    std::bernoulli_distribution occupied(0.15);
    for (int trial = 0; trial < 50; ++trial) {
        std::vector<std::uint8_t> cells(20 * 15);
        for (auto& c : cells) {
            c = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
        }
        const GridMap map(20, 15, 0.5, cells);
        GridMap previous = map;
        for (const double r : {0.1, 0.25, 0.3, 0.36, 0.6, 0.8}) {
            const GridMap inflated = amr::inflateObstacles(map, r);
            for (std::size_t i = 0; i < map.cellCount(); ++i) {
                CHECK(inflated.data()[i] >= previous.data()[i]);  // never un-blocks a cell
            }
            previous = inflated;
        }
    }
}

// The key guarantee: moving between centers of adjacent free cells of the inflated
// map never makes the disc overlap an obstacle on the real map.
void testPlannedPathsAreCollisionFree() {
    std::mt19937 rng(2026);
    std::uniform_int_distribution<int> size_dist(4, 45);
    std::uniform_real_distribution<double> density_dist(0.0, 0.3);
    std::uniform_real_distribution<double> radius_cells(0.05, 1.6);
    const double resolutions[] = {0.25, 0.5, 1.0};
    int successes = 0;
    for (int trial = 0; trial < 300; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        const double res = resolutions[trial % 3];
        std::bernoulli_distribution occupied(density_dist(rng));
        std::vector<std::uint8_t> cells(static_cast<std::size_t>(width) *
                                        static_cast<std::size_t>(height));
        for (auto& c : cells) {
            c = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
        }
        const GridMap map(width, height, res, std::move(cells));
        const double radius = radius_cells(rng) * res;
        const GridMap inflated = amr::inflateObstacles(map, radius);

        // Start and goal are drawn from the cells where the disc fits.
        std::vector<GridCoord> fits;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (inflated.isFree({x, y})) {
                    fits.push_back({x, y});
                }
            }
        }
        if (fits.empty()) {
            continue;
        }
        std::uniform_int_distribution<std::size_t> pick(0, fits.size() - 1);
        const GridCoord start = fits[pick(rng)];
        const GridCoord goal = fits[pick(rng)];
        const amr::PlanResult plan = amr::planWithFootprint(map, inflated, start, goal);
        if (!plan.success()) {
            continue;
        }
        ++successes;
        for (std::size_t i = 0; i < plan.path.size(); ++i) {
            const WorldPoint p = map.gridToWorld(plan.path[i]);
            CHECK(!amr::discOverlapsObstacle(map, p, radius));
            if (i > 0) {
                CHECK(!amr::sweptDiscOverlapsObstacle(map, map.gridToWorld(plan.path[i - 1]),
                                                      p, radius));
            }
        }
        MotionConfig config;
        config.robot_radius = radius;
        config.max_time = 5000.0;
        const amr::SimulationResult sim = amr::simulatePath(map, plan.path, config);
        CHECK(sim.status == NavigationStatus::GoalReached);
        CHECK_EQ(sim.collision_count, static_cast<std::size_t>(0));
    }
    CHECK(successes > 150);  // the sample must exercise many successful plans
    std::cout << "    (" << successes << " of 300 random requests planned and simulated)\n";
}

/// Wall across row 3 with a one-cell (1 m) door at x = 2; 5 x 7 cells of 1 m.
GridMap doorMap() {
    return makeMap({".....", ".....", ".....", "##.##", ".....", ".....", "....."}, 1.0);
}

void testFootprintStatuses() {
    const GridMap map = doorMap();

    // r = 0.5: the robot exactly fits the 1 m door (touching both sides is allowed).
    const GridMap fits = amr::inflateObstacles(map, 0.5);
    CHECK(amr::planWithFootprint(map, fits, {2, 1}, {2, 5}).success());

    // r = 0.6: the door is too narrow.
    const GridMap wide = amr::inflateObstacles(map, 0.6);
    CHECK(amr::planWithFootprint(map, wide, {2, 1}, {2, 5}).status == PlanStatus::NoPath);

    // Free start/goal cells where the disc does not fit (0.5 m from the map border)
    // are reported as such...
    CHECK(amr::planWithFootprint(map, wide, {0, 0}, {2, 5}).status ==
          PlanStatus::StartTooCloseToObstacle);
    CHECK(amr::planWithFootprint(map, wide, {2, 1}, {4, 5}).status ==
          PlanStatus::GoalTooCloseToObstacle);
    // ...while really occupied cells keep their original status.
    CHECK(amr::planWithFootprint(map, wide, {0, 3}, {2, 5}).status == PlanStatus::StartOccupied);
    CHECK(amr::planWithFootprint(map, wide, {2, 1}, {4, 3}).status == PlanStatus::GoalOccupied);
}

void testSimulatorDetectsDiscCollision() {
    // A point-robot path through the 1 m door, driven by a robot that is too wide.
    const GridMap map = doorMap();
    const amr::PlanResult plan = amr::planAStar(map, {2, 1}, {2, 5});
    REQUIRE(plan.success());
    MotionConfig config;
    config.robot_radius = 0.6;
    const amr::SimulationResult sim = amr::simulatePath(map, plan.path, config);
    CHECK(sim.status == NavigationStatus::Collision);
    CHECK_EQ(sim.collision_count, static_cast<std::size_t>(1));
    REQUIRE(!sim.trajectory.empty());
    const auto& last = sim.trajectory.back().state;
    CHECK(!amr::discOverlapsObstacle(map, {last.x, last.y}, 0.6));  // stopped while clear
    // Moving down x = 2.5, the disc first touches the door corners (2, 3) and (3, 3)
    // when 3 - y = sqrt(0.6^2 - 0.5^2): the robot must stop before that, and within
    // one 0.05 m step of it.
    const double contact_y = 3.0 - std::sqrt(0.6 * 0.6 - 0.5 * 0.5);
    CHECK(last.y < contact_y);
    CHECK(last.y > contact_y - 0.05 - 1e-9);

    // The same path is fine for a point robot and for a robot that fits.
    CHECK(amr::simulatePath(map, plan.path, MotionConfig{}).status == NavigationStatus::GoalReached);
    config.robot_radius = 0.5;
    CHECK(amr::simulatePath(map, plan.path, config).status == NavigationStatus::GoalReached);
}

void testRadiusValidation() {
    MotionConfig config;
    config.robot_radius = -0.1;
    CHECK(!amr::validateMotionConfig(config).empty());
    config.robot_radius = std::nan("");
    CHECK(!amr::validateMotionConfig(config).empty());
    config.robot_radius = 1000.0;
    CHECK(!amr::validateMotionConfig(config).empty());
    config.robot_radius = 0.0;
    CHECK(amr::validateMotionConfig(config).empty());
    config.robot_radius = 0.35;
    CHECK(amr::validateMotionConfig(config).empty());
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Disc geometry: touching vs. overlapping, corners, border", testDiscGeometry},
        {"Swept disc check is exact between samples", testSweptDiscIsExact},
        {"Inflation rings for r = 0, 0.5, 0.6, 0.75 cells", testInflationRings},
        {"Inflation grows monotonically with the radius", testInflationIsMonotonic},
        {"Planned paths keep the disc clear (300 random maps)", testPlannedPathsAreCollisionFree},
        {"Footprint statuses and door widths", testFootprintStatuses},
        {"Simulator detects disc collisions", testSimulatorDetectsDiscCollision},
        {"Robot radius validation", testRadiusValidation},
    });
}
