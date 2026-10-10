// Tests for point-robot kinematics and path-following simulation (TC12, TC14,
// plus the motion-side expectations of TC03, TC04 and TC05).
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "astar.hpp"
#include "grid_map.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::GridCoord;
using amr::GridMap;
using amr::MotionConfig;
using amr::NavigationStatus;
using amr::SimulationResult;
using amr::WorldPoint;

namespace {

constexpr double kResolution = 0.5;
constexpr double kTolerance = 1e-9;

GridMap makeMap(const std::vector<std::string>& rows) {
    const auto height = static_cast<int>(rows.size());
    const auto width = static_cast<int>(rows.front().size());
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(width, height, kResolution, std::move(cells));
}

/// A 20 x 20 map with obstacles that force several turns.
GridMap makeObstacleMap() {
    return makeMap({
        "....................",
        "....................",
        "................###.",
        "....##..........###.",
        "....##..............",
        "....##..............",
        "....##...#######....",
        "....##...#######....",
        "....##..............",
        "....##..............",
        "....##......##......",
        "....##......##......",
        "....##......##.###..",
        "....##......##.###..",
        "....##......##......",
        "......####..##......",
        "......####..##......",
        "............##......",
        "....................",
        "....................",
    });
}

/// True if p lies on segment a->b (collinear and between the endpoints).
bool onSegment(WorldPoint p, WorldPoint a, WorldPoint b) {
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double apx = p.x - a.x;
    const double apy = p.y - a.y;
    const double cross = abx * apy - aby * apx;
    const double dot = abx * apx + aby * apy;
    const double length_sq = abx * abx + aby * aby;
    return std::fabs(cross) <= 1e-9 && dot >= -1e-9 && dot <= length_sq + 1e-9;
}

/// TC14 invariants, shared by several tests.
void checkTrajectoryConsistency(const SimulationResult& sim, const MotionConfig& config) {
    REQUIRE(!sim.trajectory.empty());
    CHECK_NEAR(sim.trajectory.front().state.time, 0.0, kTolerance);
    for (std::size_t i = 0; i < sim.trajectory.size(); ++i) {
        const amr::RobotState& s = sim.trajectory[i].state;
        CHECK(std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.time));
        if (i == 0) {
            continue;
        }
        const amr::RobotState& prev = sim.trajectory[i - 1].state;
        const double dt = s.time - prev.time;
        CHECK(dt > 0.0);                                   // strictly increasing time
        CHECK_NEAR(dt, config.dt, 1e-9);                   // fixed timestep
        const double moved = std::hypot(s.x - prev.x, s.y - prev.y);
        CHECK(moved / dt <= config.speed * (1.0 + 1e-9));  // never faster than configured
    }
    CHECK_NEAR(sim.travelTime(), static_cast<double>(sim.steps) * config.dt, 1e-9);
    CHECK_EQ(sim.trajectory.size(), sim.steps + 1);
}

/// Every sample lies on the segment toward its target waypoint: no overshoot.
void checkNoOvershoot(const GridMap& map, const std::vector<GridCoord>& path,
                      const SimulationResult& sim) {
    for (const amr::TrajectorySample& sample : sim.trajectory) {
        const std::size_t k = sample.target_waypoint_index;
        REQUIRE(k < path.size());
        const WorldPoint p{sample.state.x, sample.state.y};
        if (k == 0) {
            CHECK(std::hypot(p.x - map.gridToWorld(path[0]).x,
                             p.y - map.gridToWorld(path[0]).y) <= kTolerance);
            continue;
        }
        if (!onSegment(p, map.gridToWorld(path[k - 1]), map.gridToWorld(path[k]))) {
            ::testfw::reportFailure(__FILE__, __LINE__,
                                    "sample at t=" + std::to_string(sample.state.time) +
                                        " is off the segment to waypoint " + std::to_string(k));
            return;
        }
    }
}

void testStepToward() {
    const auto normal = amr::stepToward({0.0, 0.0}, {1.0, 0.0}, 0.25);
    CHECK_NEAR(normal.position.x, 0.25, kTolerance);
    CHECK_NEAR(normal.position.y, 0.0, kTolerance);
    CHECK_NEAR(normal.distance, 0.25, kTolerance);
    CHECK(!normal.reached);

    const auto diagonal = amr::stepToward({0.0, 0.0}, {3.0, 4.0}, 1.0);  // 3-4-5 triangle
    CHECK_NEAR(diagonal.position.x, 0.6, kTolerance);
    CHECK_NEAR(diagonal.position.y, 0.8, kTolerance);

    const auto clamped = amr::stepToward({0.0, 0.0}, {0.1, 0.0}, 0.25);  // would overshoot
    CHECK(clamped.reached);
    CHECK_NEAR(clamped.position.x, 0.1, kTolerance);
    CHECK_NEAR(clamped.distance, 0.1, kTolerance);

    const auto zero = amr::stepToward({2.0, 3.0}, {2.0, 3.0}, 0.25);  // zero distance
    CHECK(zero.reached);
    CHECK(std::isfinite(zero.position.x) && std::isfinite(zero.position.y));
    CHECK_NEAR(zero.distance, 0.0, kTolerance);
}

void testMotionConfigValidation() {
    CHECK(amr::validateMotionConfig(MotionConfig{}).empty());
    MotionConfig c;
    c.speed = 0.0;
    CHECK(!amr::validateMotionConfig(c).empty());
    c = MotionConfig{};
    c.dt = -0.1;
    CHECK(!amr::validateMotionConfig(c).empty());
    c = MotionConfig{};
    c.goal_tolerance = -1.0;
    CHECK(!amr::validateMotionConfig(c).empty());
    c = MotionConfig{};
    c.goal_tolerance = 0.0;  // exact arrival is allowed
    CHECK(amr::validateMotionConfig(c).empty());
    c = MotionConfig{};
    c.max_time = std::nan("");
    CHECK(!amr::validateMotionConfig(c).empty());
    c = MotionConfig{};
    c.dt = 1e-9;  // would require 6e11 steps
    CHECK(!amr::validateMotionConfig(c).empty());

    const GridMap map(3, 1, kResolution);
    MotionConfig bad;
    bad.speed = -1.0;
    const auto sim = amr::simulatePath(map, {{0, 0}, {1, 0}}, bad);
    CHECK(sim.status == NavigationStatus::InvalidConfig);
    CHECK(sim.trajectory.empty());
}

void testSegmentCollisionCheck() {
    const GridMap map = makeMap({
        "...",
        ".#.",
        "...",
    });
    CHECK(amr::isSegmentCollisionFree(map, {0.25, 0.25}, {1.25, 0.25}));   // top row
    CHECK(!amr::isSegmentCollisionFree(map, {0.25, 0.75}, {1.25, 0.75}));  // into obstacle
    CHECK(!amr::isSegmentCollisionFree(map, {0.25, 0.25}, {1.25, 1.25}));  // diagonal crosses it
    CHECK(!amr::isSegmentCollisionFree(map, {0.25, 0.25}, {-0.25, 0.25})); // leaves the map
    CHECK(amr::isSegmentCollisionFree(map, {0.25, 0.25}, {0.25, 0.25}));   // zero length
    CHECK(!amr::isSegmentCollisionFree(map, {0.75, 0.75}, {0.75, 0.75}));  // inside obstacle
}

// TC12 - Robot motion: reaches the goal within tolerance, no overshoot, terminates.
void tc12RobotMotion() {
    const GridMap map = makeObstacleMap();
    const auto plan = amr::planAStar(map, {1, 1}, {18, 18});
    REQUIRE(plan.success());
    const MotionConfig config;  // 0.5 m/s, dt 0.1 s, tolerance 0.05 m
    const SimulationResult sim = amr::simulatePath(map, plan.path, config);

    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK_EQ(sim.collision_count, static_cast<std::size_t>(0));
    CHECK(sim.final_goal_error <= config.goal_tolerance);
    checkNoOvershoot(map, plan.path, sim);
    checkTrajectoryConsistency(sim, config);

    // 34 cells of 0.5 m at 0.05 m per step = exactly 10 steps per cell.
    const double path_m = amr::pathLengthMeters(map, plan.path);
    CHECK_NEAR(sim.distance_traveled, path_m, 1e-9);
    CHECK_EQ(sim.steps, static_cast<std::size_t>(plan.path_cost) * 10);
    CHECK_NEAR(sim.travelTime(), path_m / config.speed, 1e-9);

    // Starts at the start-cell center and ends on the goal-cell center.
    const WorldPoint start = map.gridToWorld({1, 1});
    const WorldPoint goal = map.gridToWorld({18, 18});
    CHECK_NEAR(sim.trajectory.front().state.x, start.x, kTolerance);
    CHECK_NEAR(sim.trajectory.front().state.y, start.y, kTolerance);
    CHECK_NEAR(sim.trajectory.back().state.x, goal.x, kTolerance);
    CHECK_NEAR(sim.trajectory.back().state.y, goal.y, kTolerance);
    CHECK_EQ(sim.trajectory.back().target_waypoint_index, plan.path.size() - 1);

    // Every waypoint is visited, in order.
    std::size_t next = 0;
    for (const auto& sample : sim.trajectory) {
        const WorldPoint w = map.gridToWorld(plan.path[next]);
        if (std::hypot(sample.state.x - w.x, sample.state.y - w.y) <= kTolerance) {
            if (++next == plan.path.size()) {
                break;
            }
        }
    }
    CHECK_EQ(next, plan.path.size());
}

// TC14 - Trajectory consistency for several speed/timestep combinations,
// including steps that do not divide the cell size and steps longer than a cell.
void tc14TrajectoryConsistency() {
    const GridMap map = makeObstacleMap();
    const auto plan = amr::planAStar(map, {0, 19}, {19, 0});
    REQUIRE(plan.success());
    const double configs[][2] = {{0.5, 0.1}, {0.7, 0.13}, {0.3, 0.05}, {2.0, 1.0}, {1.0, 0.017}};
    for (const auto& c : configs) {
        MotionConfig config;
        config.speed = c[0];
        config.dt = c[1];
        const SimulationResult sim = amr::simulatePath(map, plan.path, config);
        CHECK(sim.status == NavigationStatus::GoalReached);
        checkTrajectoryConsistency(sim, config);
        checkNoOvershoot(map, plan.path, sim);
        CHECK(sim.final_goal_error <= config.goal_tolerance);
    }

    // Steps that do not divide a cell: 0.7 * 0.13 = 0.091 m -> 6 steps per 0.5 m cell,
    // because leftover distance is not carried across waypoints.
    MotionConfig odd;
    odd.speed = 0.7;
    odd.dt = 0.13;
    const SimulationResult sim = amr::simulatePath(map, plan.path, odd);
    CHECK_EQ(sim.steps, static_cast<std::size_t>(plan.path_cost) * 6);
}

// TC03 (motion side) - the robot follows a narrow corridor without collision.
void tc03RobotFollowsCorridor() {
    const GridMap map = makeMap({
        ".......",
        "######.",
        ".......",
        ".######",
        ".......",
    });
    const auto plan = amr::planAStar(map, {0, 0}, {0, 4});
    REQUIRE(plan.success());
    const MotionConfig config;
    const SimulationResult sim = amr::simulatePath(map, plan.path, config);
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK_EQ(sim.collision_count, static_cast<std::size_t>(0));
    for (const auto& sample : sim.trajectory) {
        CHECK(map.isFree(map.worldToGrid({sample.state.x, sample.state.y})));
    }
    checkNoOvershoot(map, plan.path, sim);
}

// TC04 (motion side) - planning failure means no trajectory is generated.
void tc04NoTrajectoryWithoutPlan() {
    const GridMap map = makeMap({
        "..#..",
        "..#..",
        "..#..",
    });
    const auto plan = amr::planAStar(map, {0, 0}, {4, 0});
    CHECK(!plan.success());
    // The CLI only simulates successful plans; a default result is NOT_RUN and empty.
    const SimulationResult not_run;
    CHECK(not_run.status == NavigationStatus::NotRun);
    CHECK(not_run.trajectory.empty());
    CHECK_NEAR(not_run.travelTime(), 0.0, kTolerance);
    // Simulating the (empty) failed path is rejected rather than fabricating motion.
    const SimulationResult sim = amr::simulatePath(map, plan.path, MotionConfig{});
    CHECK(sim.status == NavigationStatus::InvalidPath);
    CHECK(sim.trajectory.empty());
}

// TC05 (motion side) - start equals goal: no movement required.
void tc05NoMovementWhenStartIsGoal() {
    const GridMap map(5, 5, kResolution);
    const auto plan = amr::planAStar(map, {2, 2}, {2, 2});
    REQUIRE(plan.success());
    const SimulationResult sim = amr::simulatePath(map, plan.path, MotionConfig{});
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK_EQ(sim.steps, static_cast<std::size_t>(0));
    REQUIRE(sim.trajectory.size() == 1);  // initial == final sample
    CHECK_NEAR(sim.travelTime(), 0.0, kTolerance);
    CHECK_NEAR(sim.distance_traveled, 0.0, kTolerance);
    CHECK_NEAR(sim.final_goal_error, 0.0, kTolerance);
}

void testZeroDistanceWaypointsAreSkipped() {
    const GridMap map(4, 1, kResolution);
    // Duplicate waypoints are not 4-connected moves, so they are invalid input...
    const auto dup = amr::simulatePath(map, {{0, 0}, {0, 0}, {1, 0}}, MotionConfig{});
    CHECK(dup.status == NavigationStatus::InvalidPath);
    // ...while a normal path never stalls: each step moves the robot.
    const auto sim = amr::simulatePath(map, {{0, 0}, {1, 0}, {2, 0}, {3, 0}}, MotionConfig{});
    CHECK(sim.status == NavigationStatus::GoalReached);
    for (std::size_t i = 1; i < sim.trajectory.size(); ++i) {
        const auto& a = sim.trajectory[i - 1].state;
        const auto& b = sim.trajectory[i].state;
        CHECK(std::hypot(b.x - a.x, b.y - a.y) > 0.0);
    }
}

void testTimeout() {
    const GridMap map(20, 1, kResolution);
    const auto plan = amr::planAStar(map, {0, 0}, {19, 0});  // 9.5 m -> 19 s at 0.5 m/s
    REQUIRE(plan.success());
    MotionConfig config;
    config.max_time = 5.0;
    const SimulationResult sim = amr::simulatePath(map, plan.path, config);
    CHECK(sim.status == NavigationStatus::Timeout);
    CHECK_EQ(sim.steps, static_cast<std::size_t>(50));
    CHECK(sim.trajectory.back().state.time <= config.max_time + kTolerance);
    CHECK(sim.final_goal_error > config.goal_tolerance);
    checkTrajectoryConsistency(sim, config);
}

void testCollisionIsDetected() {
    const GridMap map = makeMap({
        "..#..",
    });
    // A structurally valid path that runs straight through the obstacle.
    const std::vector<GridCoord> path{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
    const SimulationResult sim = amr::simulatePath(map, path, MotionConfig{});
    CHECK(sim.status == NavigationStatus::Collision);
    CHECK_EQ(sim.collision_count, static_cast<std::size_t>(1));
    REQUIRE(!sim.trajectory.empty());
    // The robot stops at its last safe pose, before entering the obstacle cell (x >= 1.0 m).
    const auto& last = sim.trajectory.back().state;
    CHECK(map.isFree(map.worldToGrid({last.x, last.y})));
    CHECK(last.x < 1.0);
    CHECK_NEAR(last.x, 0.95, 1e-9);

    // Starting inside an obstacle is reported immediately.
    const SimulationResult blocked = amr::simulatePath(map, {{2, 0}, {3, 0}}, MotionConfig{});
    CHECK(blocked.status == NavigationStatus::Collision);
    CHECK_EQ(blocked.steps, static_cast<std::size_t>(0));
}

void testInvalidPathsAreRejected() {
    const GridMap map(5, 5, kResolution);
    CHECK(amr::simulatePath(map, {}, MotionConfig{}).status == NavigationStatus::InvalidPath);
    CHECK(amr::simulatePath(map, {{0, 0}, {2, 0}}, MotionConfig{}).status ==
          NavigationStatus::InvalidPath);  // jump
    CHECK(amr::simulatePath(map, {{0, 0}, {1, 1}}, MotionConfig{}).status ==
          NavigationStatus::InvalidPath);  // diagonal
    CHECK(amr::simulatePath(map, {{4, 0}, {5, 0}}, MotionConfig{}).status ==
          NavigationStatus::InvalidPath);  // out of bounds
}

}  // namespace

int main() {
    return testfw::runAll({
        {"stepToward: normal, diagonal, clamped and zero-distance steps", testStepToward},
        {"Motion configuration validation", testMotionConfigValidation},
        {"Segment collision check", testSegmentCollisionCheck},
        {"TC12 Robot reaches goal, no overshoot, terminates", tc12RobotMotion},
        {"TC14 Trajectory consistency (5 speed/dt settings)", tc14TrajectoryConsistency},
        {"TC03 Robot follows a narrow corridor", tc03RobotFollowsCorridor},
        {"TC04 No trajectory without a plan", tc04NoTrajectoryWithoutPlan},
        {"TC05 No movement when start equals goal", tc05NoMovementWhenStartIsGoal},
        {"Robot never stalls on a valid path", testZeroDistanceWaypointsAreSkipped},
        {"Timeout stops the simulation", testTimeout},
        {"Collision is detected and stops the robot", testCollisionIsDetected},
        {"Invalid paths are rejected", testInvalidPathsAreRejected},
    });
}
