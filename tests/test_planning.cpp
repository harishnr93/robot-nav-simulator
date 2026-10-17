// Tests for planning with moving obstacles: safe interval path planning over space
// and time, turn-aware planning, the steppable navigator, livelock detection with a
// strategy switch, and several robots that all replan.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "astar.hpp"
#include "dynamic_obstacles.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "multi_robot.hpp"
#include "simulator.hpp"
#include "spacetime_planner.hpp"
#include "test_framework.hpp"

using amr::DynamicObstacle;
using amr::DynamicScenario;
using amr::GridCoord;
using amr::GridMap;
using amr::MotionConfig;
using amr::NavigationStatus;
using amr::ObstaclePrediction;
using amr::SpaceTimeConfig;
using amr::SpaceTimePlan;
using amr::WorldPoint;

namespace {

constexpr double kPi = 3.14159265358979323846;

GridMap makeMap(const std::vector<std::string>& rows, double resolution = 0.5) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()), resolution,
                   std::move(cells));
}

GridMap loadMap(const std::string& name) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(std::string(AMR_SOURCE_DIR) + "/maps/" + name, 0.5);
    if (!loaded.ok) {
        return makeMap({"."});
    }
    return loaded.map;
}

/// The true future of scripted obstacles, as predictions from time t on.
std::vector<ObstaclePrediction> truth(const std::vector<DynamicObstacle>& obstacles, double t, double dt,
                                      std::size_t steps) {
    std::vector<ObstaclePrediction> out;
    for (const DynamicObstacle& o : obstacles) {
        ObstaclePrediction p;
        p.radius = o.radius;
        for (std::size_t k = 0; k <= steps; ++k) {
            p.centers.push_back(amr::obstaclePosition(o, t + static_cast<double>(k) * dt));
        }
        out.push_back(std::move(p));
    }
    return out;
}

/// Where a timed plan puts the robot at time t (waits at cells, straight lines between).
WorldPoint planPosition(const GridMap& map, const SpaceTimePlan& plan, double t) {
    const auto& w = plan.waypoints;
    for (std::size_t i = 0; i + 1 < w.size(); ++i) {
        if (t <= w[i].depart) {
            return map.gridToWorld(w[i].cell);
        }
        if (t < w[i + 1].arrive) {
            const double s = (t - w[i].depart) / (w[i + 1].arrive - w[i].depart);
            const WorldPoint a = map.gridToWorld(w[i].cell), b = map.gridToWorld(w[i + 1].cell);
            return {a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s};
        }
    }
    return map.gridToWorld(w.back().cell);
}

int countTurns(const std::vector<GridCoord>& path) {
    int turns = 0;
    for (std::size_t k = 2; k < path.size(); ++k) {
        turns += (path[k].x - path[k - 1].x != path[k - 1].x - path[k - 2].x ||
                  path[k].y - path[k - 1].y != path[k - 1].y - path[k - 2].y)
                     ? 1
                     : 0;
    }
    return turns;
}

// ---------------------------------------------------------------------------------
// Space-time planning

/// Without obstacles and turn costs: the shortest path, at the robot's speed.
void testNoObstaclesIsAStar() {
    const GridMap map = loadMap("obstacles.csv");
    SpaceTimeConfig config;
    const SpaceTimePlan plan = amr::planSpaceTime(map, {1, 1}, 0.0, {18, 18}, {}, config);
    REQUIRE(plan.success);
    const amr::PlanResult reference = amr::planAStar(map, {1, 1}, {18, 18});
    CHECK_EQ(plan.waypoints.size(), reference.path.size());
    CHECK_NEAR(plan.arrival, static_cast<double>(reference.path_cost) * 0.5 / config.speed, 1e-9);
    const auto cells = amr::planCells(plan);
    for (std::size_t i = 1; i < cells.size(); ++i) {
        CHECK(amr::isAllowedMove(map, cells[i - 1], cells[i], amr::Connectivity::Four));
    }
    // Never waits: every departure is the arrival.
    for (const auto& w : plan.waypoints) {
        CHECK_NEAR(w.depart, w.arrive, 1e-9);
    }
    // Unreachable goal; start = goal.
    const GridMap walled = makeMap({"..#..", "..#..", "..#.."});
    CHECK(!amr::planSpaceTime(walled, {0, 0}, 0.0, {4, 0}, {}, config).success);
    const SpaceTimePlan here = amr::planSpaceTime(walled, {0, 0}, 0.0, {0, 0}, {}, config);
    CHECK(here.success && here.waypoints.size() == 1 && here.arrival == 0.0);
}

/// Turn-aware: every change of heading costs turning time; the fastest path to a
/// diagonal goal on an open map is an L (one turn), and the plan says how long.
void testTurnAwareFindsTheFastestPath() {
    const GridMap map = makeMap({"......", "......", "......", "......", "......", "......"});
    SpaceTimeConfig config;
    config.turn_aware = true;
    const SpaceTimePlan plan = amr::planSpaceTime(map, {0, 0}, 0.0, {5, 5}, {}, config);
    REQUIRE(plan.success);
    CHECK_EQ(countTurns(amr::planCells(plan)), 1);
    // 10 moves of 1 s, one 90 deg turn: ceil((pi/2 / 1.5 + 0.3) / 0.1) steps = 1.4 s.
    CHECK_NEAR(plan.arrival, 11.4, 1e-9);
    // Facing the goal's row already: no turn at the start.
    const SpaceTimePlan straight = amr::planSpaceTime(map, {0, 0}, 0.0, {5, 0}, {}, config);
    CHECK_NEAR(straight.arrival, 5.0, 1e-9);
    // Facing away: the turn round is paid first.
    const SpaceTimePlan behind = amr::planSpaceTime(map, {5, 0}, 0.0, {0, 0}, {}, config);
    CHECK(behind.arrival > 5.0 + 2.0);
    // On the obstacles map, turn-aware paths are never slower to drive.
    const GridMap obstacles = loadMap("obstacles.csv");
    const SpaceTimePlan aware = amr::planSpaceTime(obstacles, {1, 1}, 0.0, {18, 18}, {}, config);
    MotionConfig motion;
    motion.model = amr::RobotModel::DiffDrive;
    const auto drive = [&](const std::vector<GridCoord>& path) {
        return amr::simulatePath(obstacles, path, motion).travelTime();
    };
    const amr::PlanResult shortest = amr::planAStar(obstacles, {1, 1}, {18, 18});
    REQUIRE(aware.success && shortest.success());
    CHECK(drive(amr::planCells(aware)) <= drive(shortest.path) + 1e-9);
    // The plan predicts the simulated arrival closely.
    CHECK(std::fabs(drive(amr::planCells(aware)) - aware.arrival) < 0.05 * aware.arrival);
}

/// An obstacle patrols across the only corridor: the plan times the crossing (it
/// waits), and following it keeps clear of the obstacle at every step.
void testTimesACrossing() {
    const GridMap map = makeMap({"#####.#####", "...........", "#####.#####"});
    DynamicObstacle ob;
    ob.route = {map.gridToWorld({5, 0}), map.gridToWorld({5, 2})};
    ob.speed = 0.3;
    ob.radius = 0.2;
    SpaceTimeConfig config;
    config.clearance = 0.1;
    const auto predictions = truth({ob}, 0.0, config.dt, 200);
    const SpaceTimePlan plan = amr::planSpaceTime(map, {0, 1}, 0.0, {10, 1}, predictions, config);
    REQUIRE(plan.success);
    CHECK(plan.arrival >= 10.0 - 1e-9);
    double closest = 1e9;
    for (int i = 0; i * 0.02 <= plan.arrival; ++i) {
        const double t = i * 0.02;
        const WorldPoint r = planPosition(map, plan, t);
        const WorldPoint o = amr::obstaclePosition(ob, t);
        closest = std::min(closest, std::hypot(r.x - o.x, r.y - o.y) - ob.radius);
    }
    CHECK(closest > 0.0);
    // Blind to the obstacle: the straight run meets it.
    const SpaceTimePlan blind = amr::planSpaceTime(map, {0, 1}, 0.0, {10, 1}, {}, config);
    double blind_closest = 1e9;
    for (int i = 0; i * 0.02 <= blind.arrival; ++i) {
        const double t = i * 0.02;
        const WorldPoint r = planPosition(map, blind, t);
        const WorldPoint o = amr::obstaclePosition(ob, t);
        blind_closest = std::min(blind_closest, std::hypot(r.x - o.x, r.y - o.y) - ob.radius);
    }
    CHECK(blind_closest < 0.0);
}

/// An obstacle stands in a corridor, then leaves: the plan waits in place (a state
/// dominates later arrivals in the same cell and safe interval, so there is no
/// stepping away and back).
void testWaitsInPlace() {
    const GridMap map = makeMap({"........."});
    ObstaclePrediction p;
    p.radius = 0.2;
    const WorldPoint stand = map.gridToWorld({4, 0});
    for (int k = 0; k <= 200; ++k) {  // stands for 5 s, then leaves sideways at 1 m/s
        p.centers.push_back({stand.x, stand.y + std::max(0, k - 50) * 0.1});
    }
    SpaceTimeConfig config;
    const SpaceTimePlan plan = amr::planSpaceTime(map, {1, 0}, 0.0, {8, 0}, {p}, config);
    REQUIRE(plan.success);
    const auto cells = amr::planCells(plan);
    const std::set<std::pair<int, int>> distinct = [&] {
        std::set<std::pair<int, int>> s;
        for (const GridCoord& c : cells) {
            s.insert({c.x, c.y});
        }
        return s;
    }();
    CHECK_EQ(distinct.size(), cells.size());  // no cell visited twice
    CHECK(plan.arrival > 7.0 + 1.0);           // 7 moves of 1 s, plus the wait
}

/// start_time: the robot reaches the start cell later (it is between cells now).
void testStartTime() {
    const GridMap map = makeMap({"........"});
    SpaceTimeConfig config;
    config.start_time = 0.5;
    const SpaceTimePlan plan = amr::planSpaceTime(map, {1, 0}, 0.0, {6, 0}, {}, config);
    REQUIRE(plan.success);
    CHECK_NEAR(plan.waypoints.front().arrive, 0.5, 1e-9);
    CHECK_NEAR(plan.arrival, 5.5, 1e-9);
    // A start that is unsafe on arrival cannot be planned from.
    ObstaclePrediction p;
    p.radius = 0.2;
    p.centers.assign(201, map.gridToWorld({1, 0}));
    CHECK(!amr::planSpaceTime(map, {1, 0}, 0.0, {6, 0}, {p}, config).success);
}

// ---------------------------------------------------------------------------------
// Navigation

DynamicScenario scenarioFromFile(const GridMap& map, const std::string& file) {
    DynamicScenario s;
    s.planning_map = &map;
    s.obstacles = amr::loadDynamicObstacles(std::string(AMR_SOURCE_DIR) + "/maps/" + file, map).obstacles;
    return s;
}

/// Stepping a navigator by hand is simulatePath.
void testNavigatorMatchesSimulatePath() {
    const GridMap map = loadMap("obstacles.csv");
    const auto path = amr::planAStar(map, {1, 1}, {18, 18}).path;
    for (const auto model : {amr::RobotModel::Point, amr::RobotModel::DiffDrive}) {
        MotionConfig motion;
        motion.model = model;
        const DynamicScenario scenario = scenarioFromFile(map, "obstacles_dynamic.csv");
        const auto expected = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario);
        amr::Navigator navigator(map, path, motion, amr::Connectivity::Four, &scenario);
        while (!navigator.finished()) {
            navigator.step();
        }
        const auto& got = navigator.result();
        CHECK(got.status == expected.status);
        CHECK_EQ(got.steps, expected.steps);
        CHECK_EQ(got.trajectory.size(), expected.trajectory.size());
        CHECK_EQ(got.replans.size(), expected.replans.size());
        CHECK_NEAR(got.distance_traveled, expected.distance_traveled, 1e-12);
    }
}

/// Space-time planning in the simulator: the shipped moving-obstacle scenario, both
/// robot models, with perceived and with perfect predictions, faster than reactive.
void testSpaceTimeNavigation() {
    const GridMap map = loadMap("obstacles.csv");
    const auto path = amr::planAStar(map, {1, 1}, {18, 18}).path;
    for (const auto model : {amr::RobotModel::Point, amr::RobotModel::DiffDrive}) {
        MotionConfig motion;
        motion.model = model;
        DynamicScenario reactive = scenarioFromFile(map, "obstacles_dynamic.csv");
        const auto baseline = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &reactive);
        for (const bool perfect : {false, true}) {
            DynamicScenario scenario = scenarioFromFile(map, "obstacles_dynamic.csv");
            scenario.replanning.planner = amr::DynamicPlanner::SpaceTime;
            scenario.replanning.perfect_prediction = perfect;
            const auto sim = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario);
            CHECK(sim.success());
            CHECK(sim.min_dynamic_clearance > 0.0);
            CHECK(!sim.replans.empty());
            CHECK(sim.travelTime() <= baseline.travelTime() + 1e-9);
        }
    }
}

/// A benchmark scenario (planning_experiment spacetime, run 34) where the reactive
/// robot with the earlier settings (margin 0.25 m, horizon 2 s, no switch) shuttles
/// until the timeout; the livelock detector switches it to space-time planning and
/// it gets through. Without moving obstacles the detector stays quiet.
void testLivelockSwitch() {
    const GridMap map = loadMap("obstacles.csv");
    const auto path = amr::planAStar(map, {1, 1}, {18, 18}).path;
    DynamicScenario scenario;
    scenario.planning_map = &map;
    const auto obstacle = [](double r, double v, double x0, double y0, double x1, double y1) {
        DynamicObstacle o;
        o.radius = r;
        o.speed = v;
        o.route = {{x0, y0}, {x1, y1}};
        return o;
    };
    scenario.obstacles = {obstacle(0.334361, 0.210827, 4.25, 5.25, 5.75, 5.25),
                          obstacle(0.266309, 0.217234, 7.75, 4.25, 7.75, 5.75),
                          obstacle(0.297384, 0.220845, 9.25, 6.75, 9.75, 6.75)};
    scenario.perception.lidar.seed = 3585670826u;
    scenario.replanning.margin = 0.25;
    scenario.replanning.prediction_horizon = 2.0;
    scenario.replanning.livelock_window = 0.0;
    MotionConfig motion;
    motion.max_time = 300.0;
    const auto stuck = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario);
    CHECK(stuck.status == NavigationStatus::Timeout);
    CHECK_EQ(stuck.strategy_switches, 0);
    scenario.replanning.livelock_window = 15.0;
    const auto rescued = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario);
    CHECK(rescued.success());
    CHECK(rescued.strategy_switches >= 1);
    // Nothing in the way: progress all along, no switch.
    DynamicScenario empty;
    empty.planning_map = &map;
    empty.replanning.livelock_window = 5.0;
    const auto clear = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &empty);
    CHECK(clear.success());
    CHECK_EQ(clear.strategy_switches, 0);
}

/// A fast-obstacle benchmark scenario (perception_experiment dynamic --speeds 0.6,1.0,
/// run 156, constant-velocity prediction): evasive steps take the differential
/// drive off its path; the livelock switch then replanned from the ends of a
/// segment it was no longer on, and the straight drive there clipped a wall. Off the
/// path, the robot now only drives where the line is free, and otherwise gets back
/// onto the grid first.
void testNoWallContactOffThePath() {
    const GridMap map = loadMap("obstacles.csv");
    const auto path = amr::planAStar(map, {1, 1}, {18, 18}).path;
    const auto obstacle = [](double r, double v, double x0, double y0, double x1, double y1) {
        DynamicObstacle o;
        o.radius = r;
        o.speed = v;
        o.route = {{x0, y0}, {x1, y1}};
        return o;
    };
    DynamicScenario scenario;
    scenario.planning_map = &map;
    scenario.obstacles = {obstacle(0.33602085327806608, 0.93765891718315419, 2.25, 9.25, 2.25, 9.75),
                          obstacle(0.30740531164516466, 0.90323699974352112, 7.25, 9.25, 8.75, 9.25),
                          obstacle(0.34223086390631746, 0.74384629540952596, 9.25, 5.75, 9.75, 5.75)};
    scenario.perception.tracker.prediction = amr::MotionPrediction::ConstantVelocity;
    scenario.perception.lidar.beams = 360;
    scenario.perception.lidar.max_range = scenario.replanning.sensing_range;
    scenario.perception.lidar.noise_stddev = 0.02;
    scenario.perception.lidar.seed = 3998571021u;
    MotionConfig motion;
    motion.model = amr::RobotModel::DiffDrive;
    motion.max_time = 600.0;
    const auto sim = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario);
    CHECK(sim.success());
    CHECK(sim.strategy_switches >= 1);
    // Every step of the trajectory stays in free space.
    for (std::size_t k = 1; k < sim.trajectory.size(); ++k) {
        const auto& a = sim.trajectory[k - 1].state;
        const auto& b = sim.trajectory[k].state;
        CHECK(amr::isSegmentCollisionFree(map, {a.x, a.y}, {b.x, b.y}));
    }
}

void testValidation() {
    amr::ReplanningConfig config;
    CHECK(amr::validateReplanningConfig(config).empty());
    config.spacetime_horizon = 0.0;
    CHECK(!amr::validateReplanningConfig(config).empty());
    config = {};
    config.spacetime_period = -1.0;
    CHECK(!amr::validateReplanningConfig(config).empty());
    config = {};
    config.spacetime_horizon = std::nan("");
    CHECK(!amr::validateReplanningConfig(config).empty());
}

// ---------------------------------------------------------------------------------
// Several robots

amr::MultiRobotConfig robotsConfig(const GridMap& planning, amr::RobotModel model) {
    amr::MultiRobotConfig config;
    config.motion.model = model;
    config.motion.robot_radius = 0.2;
    config.motion.max_time = 300.0;
    config.scenario.planning_map = &planning;
    return config;
}

/// The smallest distance between the centers of two robots over their runs.
double closestRobots(const amr::MultiRobotResult& r) {
    double closest = 1e9;
    for (std::size_t i = 0; i < r.robots.size(); ++i) {
        for (std::size_t j = i + 1; j < r.robots.size(); ++j) {
            const auto& a = r.robots[i].trajectory;
            const auto& b = r.robots[j].trajectory;
            for (std::size_t k = 1; k < std::max(a.size(), b.size()); ++k) {
                const auto& p0 = a[std::min(k - 1, a.size() - 1)].state;
                const auto& p1 = a[std::min(k, a.size() - 1)].state;
                const auto& q0 = b[std::min(k - 1, b.size() - 1)].state;
                const auto& q1 = b[std::min(k, b.size() - 1)].state;
                closest = std::min(closest, amr::closestApproach({p0.x, p0.y}, {p1.x, p1.y}, {q0.x, q0.y},
                                                                 {q1.x, q1.y}));
            }
        }
    }
    return closest;
}

/// Symmetric swaps in an open room: two robots head-on, four through the middle.
/// Identical reactive point robots mirror each other and deadlock. The livelock
/// switch alone does not help two mirrored robots (both detect it at the same
/// instant and switch together); staggered windows break the symmetry. Space-time
/// planning passes directly. Nobody touches.
void testSymmetricSwaps() {
    const GridMap map = makeMap(std::vector<std::string>(12, "............"));
    const GridMap planning = amr::inflateObstacles(map, 0.2);
    const std::vector<amr::RobotTask> two{{{1, 6}, {10, 6}, 0.0}, {{10, 6}, {1, 6}, kPi}};
    const std::vector<amr::RobotTask> four{
        {{1, 1}, {10, 10}, 0.0}, {{10, 10}, {1, 1}, 0.0}, {{1, 10}, {10, 1}, 0.0}, {{10, 1}, {1, 10}, 0.0}};
    amr::MultiRobotConfig config = robotsConfig(planning, amr::RobotModel::Point);
    config.scenario.replanning.livelock_window = 0.0;
    const auto stuck = amr::simulateRobots(map, two, config);
    CHECK(stuck.planned);
    CHECK_EQ(stuck.timeouts, 2);
    CHECK_EQ(stuck.robot_collisions, 0);
    config.scenario.replanning.livelock_window = 15.0;
    config.stagger_livelock = false;
    const auto mirrored = amr::simulateRobots(map, two, config);
    CHECK_EQ(mirrored.timeouts, 2);
    REQUIRE(mirrored.robots.size() == 2);
    CHECK_EQ(mirrored.strategy_switches, 2);  // one escape each, at the same instant
    CHECK(mirrored.robots[0].trajectory.size() == mirrored.robots[1].trajectory.size());
    config.stagger_livelock = true;
    for (const auto& tasks : {two, four}) {
        const auto r = amr::simulateRobots(map, tasks, config);
        CHECK_EQ(r.reached, static_cast<int>(tasks.size()));
        CHECK_EQ(r.robot_collisions, 0);
        CHECK(r.strategy_switches >= 1);
        CHECK(closestRobots(r) >= 0.4 - 1e-9);
    }
    for (const auto model : {amr::RobotModel::Point, amr::RobotModel::DiffDrive}) {
        amr::MultiRobotConfig spacetime = robotsConfig(planning, model);
        spacetime.scenario.replanning.planner = amr::DynamicPlanner::SpaceTime;
        for (const auto& tasks : {two, four}) {
            const auto r = amr::simulateRobots(map, tasks, spacetime);
            CHECK_EQ(r.reached, static_cast<int>(tasks.size()));
            CHECK_EQ(r.robot_collisions, 0);
            CHECK(closestRobots(r) >= 0.4 - 1e-9);
        }
    }
}

/// Bookkeeping: robots that start in contact collide at once (with each other's
/// ids); an impossible task means nothing runs.
void testRobotsBookkeeping() {
    // Radius 0.6 on 1 m cells: robots on neighbouring cells touch from the start.
    // (Inflation treats the map border as a wall: the tasks stay inside.)
    const GridMap open = makeMap(std::vector<std::string>(8, "........"), 1.0);
    const GridMap open_planning = amr::inflateObstacles(open, 0.6);
    amr::MultiRobotConfig wide = robotsConfig(open_planning, amr::RobotModel::Point);
    wide.motion.robot_radius = 0.6;
    const auto touching = amr::simulateRobots(open, {{{2, 2}, {5, 2}, 0.0}, {{2, 3}, {5, 3}, 0.0}}, wide);
    CHECK(touching.planned);
    CHECK_EQ(touching.robot_collisions, 2);
    REQUIRE(touching.robots.size() == 2);
    CHECK_EQ(touching.robots[0].collided_robot, 1);
    CHECK_EQ(touching.robots[1].collided_robot, 0);
    const GridMap walled = makeMap({"..#..", "..#.."});
    const GridMap walled_planning = amr::inflateObstacles(walled, 0.2);
    const auto none =
        amr::simulateRobots(walled, {{{0, 0}, {4, 0}, 0.0}}, robotsConfig(walled_planning, amr::RobotModel::Point));
    CHECK(!none.planned);
    CHECK(none.robots.empty());
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Space-time: no obstacles is A*", testNoObstaclesIsAStar},
        {"Turn-aware planning finds the fastest path", testTurnAwareFindsTheFastestPath},
        {"Space-time: times a crossing", testTimesACrossing},
        {"Space-time: waits in place for a blocking obstacle", testWaitsInPlace},
        {"Space-time: start reached later", testStartTime},
        {"Navigator stepped by hand matches simulatePath", testNavigatorMatchesSimulatePath},
        {"Space-time planning in the simulator", testSpaceTimeNavigation},
        {"Livelock detection switches strategy", testLivelockSwitch},
        {"Off the path, no drive into a wall", testNoWallContactOffThePath},
        {"Configuration validation", testValidation},
        {"Symmetric swaps: deadlock, livelock switch, space-time", testSymmetricSwaps},
        {"Multi-robot bookkeeping", testRobotsBookkeeping},
    });
}
