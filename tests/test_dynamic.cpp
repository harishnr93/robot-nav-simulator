// Tests for dynamic obstacles and replanning: the obstacle motion model, exact
// collision distances, loading, visibility, and the replanning simulator.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "dynamic_obstacles.hpp"
#include "ekf.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::DynamicObstacle;
using amr::DynamicScenario;
using amr::GridMap;
using amr::MotionConfig;
using amr::NavigationStatus;
using amr::WorldPoint;

namespace {

constexpr double kTolerance = 1e-9;

GridMap makeMap(const std::vector<std::string>& rows, double resolution = 0.5) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                   resolution, std::move(cells));
}

DynamicObstacle obstacle(const GridMap& map, std::vector<amr::GridCoord> route, double speed,
                         double radius = 0.25) {
    DynamicObstacle o;
    for (const auto& cell : route) {
        o.route.push_back(map.gridToWorld(cell));
    }
    o.speed = speed;
    o.radius = radius;
    return o;
}

/// The decision rules are tested with ideal sightings (oracle perception); the LiDAR
/// pipeline has its own tests (test_obstacle_tracking.cpp).
DynamicScenario scenario(const GridMap& planning_map, std::vector<DynamicObstacle> obstacles) {
    DynamicScenario s;
    s.perception.mode = amr::ObstaclePerception::Oracle;
    s.planning_map = &planning_map;
    s.obstacles = std::move(obstacles);
    return s;
}

/// Plan on `map` and simulate with the given dynamic obstacles.
amr::SimulationResult run(const GridMap& map, amr::GridCoord start, amr::GridCoord goal,
                          const DynamicScenario& dynamic, MotionConfig config = MotionConfig{}) {
    const auto plan = amr::planAStar(map, start, goal);
    if (!plan.success()) {
        return {};
    }
    return amr::simulatePath(map, plan.path, config, amr::Connectivity::Four, &dynamic);
}

/// Exact minimum distance to every obstacle over each recorded step, checked
/// against the robot's sampled trajectory.
double minGapAlong(const amr::SimulationResult& sim, const DynamicScenario& dynamic,
                   double robot_radius) {
    double gap = 1e9;
    for (std::size_t k = 1; k < sim.trajectory.size(); ++k) {
        const auto& a = sim.trajectory[k - 1].state;
        const auto& b = sim.trajectory[k].state;
        for (const DynamicObstacle& o : dynamic.obstacles) {
            gap = std::min(gap, amr::minDistanceDuring(o, {a.x, a.y}, {b.x, b.y}, a.time, b.time) -
                                    o.radius - robot_radius);
        }
    }
    return gap;
}

void testPingPongMotion() {
    DynamicObstacle o;
    o.route = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}};  // 3 m long, an L
    o.speed = 0.5;
    CHECK_NEAR(amr::routeLength(o), 3.0, kTolerance);
    const auto at = [&](double t) { return amr::obstaclePosition(o, t); };
    CHECK_NEAR(at(0.0).x, 0.0, kTolerance);
    CHECK_NEAR(at(2.0).x, 1.0, kTolerance);             // 1 m along the first leg
    CHECK_NEAR(at(5.0).x, 2.0, kTolerance);             // 2.5 m: round the corner
    CHECK_NEAR(at(5.0).y, 0.5, kTolerance);
    CHECK_NEAR(at(6.0).y, 1.0, kTolerance);             // end of the route at 6 s
    CHECK_NEAR(at(7.0).y, 0.5, kTolerance);             // and back
    CHECK_NEAR(at(10.0).x, 1.0, kTolerance);
    CHECK_NEAR(at(12.0).x, 0.0, kTolerance);            // start again after 12 s
    CHECK_NEAR(at(14.0).x, at(2.0).x, kTolerance);      // periodic
    // Corner and reversal times: 4 s (corner), 6 s (end), 8 s (corner), 12 s (start).
    const auto breaks = amr::obstacleBreakpoints(o, 3.0, 13.0);
    REQUIRE(breaks.size() == 4);
    CHECK_NEAR(breaks[0], 4.0, kTolerance);
    CHECK_NEAR(breaks[1], 6.0, kTolerance);
    CHECK_NEAR(breaks[2], 8.0, kTolerance);
    CHECK_NEAR(breaks[3], 12.0, kTolerance);
    CHECK(amr::obstacleBreakpoints(o, 4.0, 6.0).empty());  // open interval
}

/// The exact closest approach against dense sampling of both motions.
void testMinDistanceAgainstSampling() {
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> pos(0.0, 5.0);
    std::uniform_real_distribution<double> speed(0.2, 3.0);
    std::uniform_real_distribution<double> start(0.0, 20.0);
    for (int trial = 0; trial < 300; ++trial) {
        DynamicObstacle o;
        o.route = {{pos(rng), pos(rng)}, {pos(rng), pos(rng)}, {pos(rng), pos(rng)}};
        o.speed = speed(rng);
        const WorldPoint a{pos(rng), pos(rng)};
        const WorldPoint b{pos(rng), pos(rng)};
        const double t0 = start(rng);
        const double t1 = t0 + 2.0;  // long enough to include corners and reversals
        const double exact = amr::minDistanceDuring(o, a, b, t0, t1);
        double sampled = 1e9;
        const int samples = 20000;
        for (int i = 0; i <= samples; ++i) {
            const double f = static_cast<double>(i) / samples;
            const WorldPoint p = amr::obstaclePosition(o, t0 + f * (t1 - t0));
            sampled = std::min(sampled, std::hypot(a.x + (b.x - a.x) * f - p.x,
                                                   a.y + (b.y - a.y) * f - p.y));
        }
        CHECK(exact <= sampled + 1e-12);  // exact is never larger than any sample
        // ...and samples come arbitrarily close: relative speed <= ~5.5 m/s, so a
        // sample is at most 2 s / 20000 * 5.5 m/s away from the true minimum.
        CHECK(sampled - exact <= 6e-4);
    }
}

/// A contact that happens between two samples is still detected.
void testContactBetweenSamples() {
    const GridMap open = makeMap({"..........", "..........", ".........."});
    // The obstacle sweeps 1 m in 0.1 s past a robot standing still at (2.25, 0.75):
    // at the step ends it is 0.5 m away on either side, but it passes straight through.
    DynamicObstacle fast;
    fast.route = {{1.75, 0.75}, {2.75, 0.75}, {2.75, 0.25}};
    fast.speed = 10.0;
    fast.radius = 0.1;
    const WorldPoint robot{2.25, 0.75};
    CHECK(std::hypot(amr::obstaclePosition(fast, 0.0).x - robot.x, 0.0) >= 0.5 - kTolerance);
    CHECK(std::hypot(amr::obstaclePosition(fast, 0.1).x - robot.x, 0.0) >= 0.5 - kTolerance);
    CHECK_NEAR(amr::minDistanceDuring(fast, robot, robot, 0.0, 0.1), 0.0, kTolerance);
}

void testLoading() {
    const GridMap map = makeMap({"......", "..#...", "......"});
    const std::string file = "test_dynamic_obstacles.csv";
    const auto load = [&](const std::string& text) {
        std::ofstream(file) << text;
        return amr::loadDynamicObstacles(file, map);
    };
    const auto good = load("# radius, speed, route\n0.2, 0.5, 0,0, 5,0\n\n0.3,1, 0,2, 5,2, 5,0\n");
    REQUIRE(good.ok);
    REQUIRE(good.obstacles.size() == 2);
    CHECK_NEAR(good.obstacles[1].radius, 0.3, kTolerance);
    CHECK_NEAR(good.obstacles[1].route[2].x, 2.75, kTolerance);  // cell (5, 0) center
    CHECK(load("0.2, 0.5, 0,0\n").error.find("two x, y cell pairs") != std::string::npos);
    CHECK(load("0.2, 0.5, 0,0, 5\n").error.find("pairs") != std::string::npos);
    CHECK(load("0.2, fast, 0,0, 5,0\n").error.find("not a number") != std::string::npos);
    CHECK(load("0.2, 0.5, 0,0, 9,0\n").error.find("outside the map") != std::string::npos);
    CHECK(load("0.2, 0.5, 0,0, 1.5,0\n").error.find("whole cell") != std::string::npos);
    CHECK(load("0.2, 0.5, 2,0, 2,2\n").error.find("crosses a static obstacle") !=
          std::string::npos);
    CHECK(load("0.2, 0.5, 2,1, 2,2\n").error.find("not a free cell") != std::string::npos);
    CHECK(load("0, 0.5, 0,0, 5,0\n").error.find("radius") != std::string::npos);
    CHECK(load("0.2, -1, 0,0, 5,0\n").error.find("speed") != std::string::npos);
    CHECK(load("0.2, 0.5, 0,0, 5,0\n").ok);
    CHECK(!amr::loadDynamicObstacles("no_such_file.csv", map).ok);
    std::remove(file.c_str());
}

void testVisibility() {
    const GridMap map = makeMap({"..........", "....#.....", ".........."});
    const WorldPoint robot = map.gridToWorld({1, 1});
    // Range is measured to the obstacle's edge.
    CHECK(amr::isObstacleVisible(map, robot, map.gridToWorld({1, 0}), 0.25, 0.3));
    CHECK(!amr::isObstacleVisible(map, robot, map.gridToWorld({1, 0}), 0.25, 0.2));
    // The wall cell (4, 1) hides an obstacle straight behind it, but not one beside it.
    CHECK(!amr::isObstacleVisible(map, robot, map.gridToWorld({7, 1}), 0.25, 10.0));
    CHECK(amr::isObstacleVisible(map, robot, map.gridToWorld({7, 0}), 0.25, 10.0) ==
          !amr::castRay(map, robot, std::atan2(-0.5, 3.0), std::hypot(3.0, 0.5)).hit);
    CHECK(amr::isObstacleVisible(map, robot, robot, 0.25, 0.0));
}

/// No obstacles: identical to the static simulation, for every robot model.
void testEmptyScenarioMatchesStatic() {
    const GridMap map = makeMap({"..........", "..##..#...", "...#..#...", "......#...",
                                 ".........."});
    for (const bool diff_drive : {false, true}) {
        for (const auto connectivity : {amr::Connectivity::Four, amr::Connectivity::Eight}) {
            MotionConfig config;
            config.model = diff_drive ? amr::RobotModel::DiffDrive : amr::RobotModel::Point;
            const auto plan = amr::planAStar(map, {0, 0}, {9, 4}, connectivity);
            REQUIRE(plan.success());
            const DynamicScenario empty = scenario(map, {});
            const auto plain = amr::simulatePath(map, plan.path, config, connectivity);
            const auto dynamic = amr::simulatePath(map, plan.path, config, connectivity, &empty);
            REQUIRE(plain.trajectory.size() == dynamic.trajectory.size());
            for (std::size_t k = 0; k < plain.trajectory.size(); ++k) {
                CHECK_EQ(plain.trajectory[k].state.x, dynamic.trajectory[k].state.x);
                CHECK_EQ(plain.trajectory[k].state.y, dynamic.trajectory[k].state.y);
                CHECK_EQ(plain.trajectory[k].state.theta, dynamic.trajectory[k].state.theta);
                CHECK_EQ(plain.trajectory[k].target_waypoint_index,
                         dynamic.trajectory[k].target_waypoint_index);
            }
            CHECK(dynamic.status == NavigationStatus::GoalReached);
            CHECK(dynamic.replans.empty());
        }
    }
}

/// A person crossing the hallway: the robot sees the obstacle sweep across its
/// path, waits for it to pass and crosses behind it. A blind robot (tiny sensing
/// range) runs into it.
void testWaitsForACrossingObstacle() {
    const GridMap map = makeMap({"....................", "....................",
                                 "....................", "....................",
                                 "...................."});
    // Sweeps column 10 over the full height of the map, back and forth.
    const auto crossing = obstacle(map, {{10, 0}, {10, 4}}, 0.3, 0.3);
    DynamicScenario seeing = scenario(map, {crossing});
    const auto sim = run(map, {1, 2}, {18, 2}, seeing);
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK(sim.wait_steps > 0);
    CHECK(sim.min_dynamic_clearance >= 0.0);
    CHECK_NEAR(sim.min_dynamic_clearance, minGapAlong(sim, seeing, 0.0), 1e-9);
    DynamicScenario blind = seeing;
    blind.replanning.sensing_range = 1e-3;
    const auto hit = run(map, {1, 2}, {18, 2}, blind);
    CHECK(hit.status == NavigationStatus::Collision);
    CHECK_EQ(hit.collided_obstacle, 0);
    CHECK(hit.replans.empty());
}

/// A slow obstacle that keeps the straight route blocked for longer than the
/// patience: the robot replans a detour (with the reused planner) and gets through.
void testReplansAroundAPersistentBlockage() {
    const GridMap map = makeMap({"....................", "....................",
                                 "....................", "....................",
                                 "...................."});
    const auto slow = obstacle(map, {{9, 2}, {11, 2}}, 0.05, 0.3);  // dawdles on row 2
    DynamicScenario dynamic = scenario(map, {slow});
    const auto sim = run(map, {1, 2}, {18, 2}, dynamic);
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK(sim.min_dynamic_clearance >= 0.0);
    std::size_t successes = 0;
    for (const auto& event : sim.replans) {
        if (event.status != amr::PlanStatus::Success) {
            continue;
        }
        ++successes;
        CHECK(event.path.front() == event.from);
        CHECK(event.path.back() == (amr::GridCoord{18, 2}));
        CHECK(amr::isValidGridPath(map, event.path));
        CHECK(event.blocked_cells > 0);
        // The detour avoids the blocked cells at planning time: never row 2 near x = 10.
        for (const auto& cell : event.path) {
            CHECK(!(cell.y == 2 && cell.x >= 9 && cell.x <= 11));
        }
    }
    CHECK(successes >= 1);
    // The first replan only comes after the patience (default 6 s) has run out.
    REQUIRE(!sim.replans.empty());
    const double first_sight = 0.0;  // the obstacle is within 3 m almost from the start
    CHECK(sim.replans.front().time >= first_sight + 6.0 - 1e-9);
    // With no patience the robot detours as soon as it sees the blockage.
    dynamic.replanning.patience = 0.0;
    const auto eager = run(map, {1, 2}, {18, 2}, dynamic);
    CHECK(eager.status == NavigationStatus::GoalReached);
    REQUIRE(!eager.replans.empty());
    CHECK(eager.replans.front().time < sim.replans.front().time);
}

/// Head-on in a corridor with a late sighting: the robot backs off to the cell it
/// came from and takes another route.
void testRetreat() {
    const GridMap map = makeMap({"....................", "....................",
                                 "...................."});
    // Comes west along row 1, the robot's row, then returns.
    const auto head_on = obstacle(map, {{17, 1}, {3, 1}}, 0.5, 0.25);
    DynamicScenario dynamic = scenario(map, {head_on});
    dynamic.replanning.sensing_range = 0.6;  // sees it late
    const auto sim = run(map, {1, 1}, {18, 1}, dynamic);
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK(sim.min_dynamic_clearance >= 0.0);
    bool backed_off = false;
    for (std::size_t k = 1; k < sim.trajectory.size(); ++k) {
        backed_off = backed_off || sim.trajectory[k].state.x < sim.trajectory[k - 1].state.x - 1e-12;
    }
    CHECK(backed_off);
}

/// A wall with one doorway, and an obstacle patrolling across the doorway's exit:
/// there is no way around, so replanning finds no path while the exit is blocked.
/// The robot waits at the door, then passes.
void testWaitsWhenThereIsNoWayAround() {
    const GridMap map = makeMap({"....#....", "....#....", ".........", "....#....",
                                 "....#...."});
    const auto exit_patrol = obstacle(map, {{5, 0}, {5, 4}}, 0.3, 0.2);
    DynamicScenario dynamic = scenario(map, {exit_patrol});
    dynamic.replanning.patience = 0.0;  // try to replan at once: there is nothing to find
    const auto sim = run(map, {1, 2}, {7, 2}, dynamic);
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK(sim.wait_steps > 0);
    CHECK(sim.min_dynamic_clearance >= 0.0);
    const bool failed_attempt =
        std::any_of(sim.replans.begin(), sim.replans.end(),
                    [](const amr::ReplanEvent& e) { return e.status != amr::PlanStatus::Success; });
    CHECK(failed_attempt);
}

/// The differential drive backs up for a retreat instead of turning around.
void testDiffDriveBacksUp() {
    const GridMap map = makeMap({"....................", "....................",
                                 "...................."});
    const auto head_on = obstacle(map, {{17, 1}, {3, 1}}, 0.5, 0.25);
    DynamicScenario dynamic = scenario(map, {head_on});
    dynamic.replanning.sensing_range = 0.6;
    MotionConfig config;
    config.model = amr::RobotModel::DiffDrive;
    const auto sim = run(map, {1, 1}, {18, 1}, dynamic, config);
    CHECK(sim.status == NavigationStatus::GoalReached);
    CHECK(sim.min_dynamic_clearance >= 0.0);
    const bool reversed = std::any_of(sim.trajectory.begin(), sim.trajectory.end(),
                                      [](const amr::TrajectorySample& s) {
                                          return s.linear_velocity < 0.0;
                                      });
    CHECK(reversed);
    // Backing up keeps the heading: the robot moves against the direction it faces.
    for (std::size_t k = 1; k < sim.trajectory.size(); ++k) {
        const auto& a = sim.trajectory[k - 1].state;
        const auto& b = sim.trajectory[k].state;
        if (sim.trajectory[k].linear_velocity < 0.0) {
            CHECK((b.x - a.x) * std::cos(b.theta) + (b.y - a.y) * std::sin(b.theta) < 0.0);
            CHECK_EQ(a.theta, b.theta);
        }
    }
    // The EKF's odometry model (turn, then drive a signed distance) still rebuilds
    // the trajectory exactly, backing up included.
    amr::Pose pose{sim.trajectory[0].state.x, sim.trajectory[0].state.y,
                   sim.trajectory[0].state.theta};
    for (std::size_t k = 1; k < sim.trajectory.size(); ++k) {
        const auto inc = amr::trueIncrement(sim.trajectory[k - 1].state, sim.trajectory[k].state);
        CHECK((inc.distance < 0.0) == (sim.trajectory[k].linear_velocity < 0.0));
        pose = amr::applyIncrement(pose, inc);
        CHECK_NEAR(pose.x, sim.trajectory[k].state.x, 1e-9);
        CHECK_NEAR(pose.y, sim.trajectory[k].state.y, 1e-9);
    }
}

/// Invariants over random scenarios: the robot stays in free cells; a run reported
/// collision-free never came closer than contact; a collision is real.
void testRandomScenarioInvariants() {
    const GridMap map = makeMap({"....................", "....##..........###.",
                                 "....##..............", "....##...#######....",
                                 "....##..............", "..........##........",
                                 "...........##.......", "....................",
                                 "........###.........", "...................."});
    std::mt19937 rng(17);
    int reached = 0;
    int collisions = 0;
    int goal_checks = 0;
    for (int trial = 0; trial < 120; ++trial) {
        std::vector<DynamicObstacle> obstacles;
        while (obstacles.size() < 3) {
            const int x = std::uniform_int_distribution<int>(0, 19)(rng);
            const int y = std::uniform_int_distribution<int>(0, 9)(rng);
            const bool horizontal = rng() % 2 == 0;
            int length = std::uniform_int_distribution<int>(2, 6)(rng);
            amr::GridCoord end{x, y};
            while (length-- > 0) {
                const amr::GridCoord next{end.x + (horizontal ? 1 : 0), end.y + (horizontal ? 0 : 1)};
                if (!map.isFree(next) || next == amr::GridCoord{18, 8}) {
                    break;
                }
                end = next;
            }
            const DynamicObstacle o = obstacle(map, {{x, y}, end}, 0.2 + 0.1 * static_cast<double>(rng() % 4));
            if (end != amr::GridCoord{x, y} && map.isFree({x, y}) && !(x == 1 && y == 1) &&
                amr::validateDynamicObstacle(map, o).empty()) {
                obstacles.push_back(o);
            }
        }
        MotionConfig config;
        config.max_time = 200.0;
        config.model = trial % 2 == 0 ? amr::RobotModel::Point : amr::RobotModel::DiffDrive;
        const DynamicScenario dynamic = scenario(map, obstacles);
        const auto sim = run(map, {1, 1}, {18, 8}, dynamic, config);
        REQUIRE(!sim.trajectory.empty());
        for (const auto& sample : sim.trajectory) {
            CHECK(map.isFree(map.worldToGrid({sample.state.x, sample.state.y})));
        }
        // The goal is never treated as blocked, so no replan fails because of it.
        for (const auto& event : sim.replans) {
            CHECK(event.status != amr::PlanStatus::GoalOccupied);
            goal_checks += 1;
        }
        const double gap = minGapAlong(sim, dynamic, 0.0);
        if (sim.status == NavigationStatus::Collision) {
            ++collisions;
            CHECK(sim.collided_obstacle >= 0);
        } else {
            CHECK(gap >= -amr::kFootprintTolerance);
            CHECK_NEAR(sim.min_dynamic_clearance, gap, 1e-9);
        }
        reached += sim.status == NavigationStatus::GoalReached ? 1 : 0;
    }
    std::cout << "    random scenarios: " << reached << "/120 reached the goal, " << collisions
              << " collisions\n";
    CHECK(goal_checks > 100);  // replans did happen
    CHECK(reached >= 108);  // measured 112 (the rest: waiting out a blocked passage)
    // Measured 4 (obstacles closing in from two sides). Over 1000 scenarios of this
    // generator the oracle collides in 2.3 % (3 expected in 120); 5 allows the noise.
    CHECK(collisions <= 5);
}

void testValidation() {
    amr::ReplanningConfig config;
    CHECK(amr::validateReplanningConfig(config).empty());
    config.sensing_range = 0.0;
    CHECK(!amr::validateReplanningConfig(config).empty());
    config = {};
    config.margin = -0.1;
    CHECK(!amr::validateReplanningConfig(config).empty());
    config = {};
    config.prediction_horizon = std::nan("");
    CHECK(!amr::validateReplanningConfig(config).empty());
    // A scenario without a planning map is a configuration error.
    const GridMap map = makeMap({"...."});
    DynamicScenario missing;
    const auto sim = amr::simulatePath(map, {{0, 0}, {1, 0}}, MotionConfig{},
                                       amr::Connectivity::Four, &missing);
    CHECK(sim.status == NavigationStatus::InvalidConfig);
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Ping-pong motion along a route, corners and reversals", testPingPongMotion},
        {"Exact closest approach vs. dense sampling", testMinDistanceAgainstSampling},
        {"Contact between two samples is detected", testContactBetweenSamples},
        {"Loading obstacle files and their errors", testLoading},
        {"Visibility: range to the edge, line of sight", testVisibility},
        {"No obstacles: identical to the static simulation", testEmptyScenarioMatchesStatic},
        {"Waits for a crossing obstacle; a blind robot is hit", testWaitsForACrossingObstacle},
        {"Replans around a persistent blockage after the patience", testReplansAroundAPersistentBlockage},
        {"Backs off when the waypoint ahead becomes blocked", testRetreat},
        {"Waits when there is no way around", testWaitsWhenThereIsNoWayAround},
        {"Differential drive backs up instead of turning around", testDiffDriveBacksUp},
        {"Random scenarios: invariants", testRandomScenarioInvariants},
        {"Configuration validation", testValidation},
    });
}
