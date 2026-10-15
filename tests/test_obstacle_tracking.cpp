// Tests for moving obstacles seen through the LiDAR alone: map subtraction and disc
// fitting, the multi-target tracker (two-point start, reversals, negative evidence,
// coasting), learned turning points and their prediction, and the simulator's LiDAR
// perception (with occlusion awareness).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "dynamic_obstacles.hpp"
#include "geometry2d.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "obstacle_tracking.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::GridMap;
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

GridMap obstaclesMap() {
    const auto loaded = amr::loadGridMapCsv(std::string(AMR_SOURCE_DIR) + "/maps/obstacles.csv", 0.5);
    CHECK(loaded.ok);
    return loaded.map;
}

/// A LiDAR world: the map plus discs (center, radius) at time t.
struct Disc {
    WorldPoint center;
    double radius;
};

amr::LidarScan scanAt(const GridMap& map, WorldPoint origin, const std::vector<Disc>& discs,
                      const amr::LidarConfig& lidar, amr::RangeErrorModel& errors, double time) {
    amr::LidarScan scan;
    scan.origin = origin;
    scan.time = time;
    errors.nextScan(time);
    for (const double a : amr::beamOffsets(lidar)) {
        const amr::RayHit wall = amr::castRay(map, origin, a, lidar.max_range);
        double r = wall.hit ? wall.range : 1e300;
        for (const Disc& d : discs) {
            r = std::min(r, amr::rayToCircle(d.center, d.radius, origin, a));
        }
        const bool hit = r <= lidar.max_range;
        scan.angles.push_back(a);
        scan.ranges.push_back(hit ? errors.measure(r) : lidar.max_range);
        scan.hits.push_back(hit);
    }
    return scan;
}

amr::LidarConfig lidar360(double noise = 0.02, std::uint32_t seed = 1) {
    amr::LidarConfig lidar;
    lidar.beams = 360;
    lidar.max_range = 3.0;
    lidar.noise_stddev = noise;
    lidar.seed = seed;
    return lidar;
}

/// Run the tracker on one obstacle seen from a fixed robot position; calls
/// `inspect(t, tracker)` after every scan.
template <typename Inspect>
void watch(const GridMap& map, WorldPoint robot, const amr::DynamicObstacle& obstacle, double seconds,
           amr::ObstacleTracker& tracker, const Inspect& inspect, double vanish_at = 1e300) {
    const amr::LidarConfig lidar = lidar360();
    amr::RangeErrorModel errors(lidar);
    for (int k = 0; k * 0.1 <= seconds + 1e-9; ++k) {
        const double t = 0.1 * k;
        std::vector<Disc> discs;
        if (t < vanish_at) {
            discs.push_back({amr::obstaclePosition(obstacle, t), obstacle.radius});
        }
        const amr::LidarScan scan = scanAt(map, robot, discs, lidar, errors, t);
        const auto detections = amr::detectMovingObstacles(map, scan, lidar.max_range, true, amr::DetectionConfig{});
        tracker.update(t, detections, map, scan, lidar.max_range);
        inspect(t, tracker);
    }
}

std::size_t confirmedTracks(const amr::ObstacleTracker& tracker) {
    return static_cast<std::size_t>(std::count_if(tracker.tracks().begin(), tracker.tracks().end(),
                                                  [](const amr::Track& t) { return t.confirmed; }));
}

// ---------------------------------------------------------------------------------
// Detection

void testStaticWorldHasNoDetections() {
    const GridMap map = obstaclesMap();
    const amr::LidarConfig lidar = lidar360(0.02, 3);
    amr::RangeErrorModel errors(lidar);
    std::mt19937 rng(5);
    std::uniform_int_distribution<int> cell(0, 19);
    int scans = 0;
    for (int k = 0; k < 400 && scans < 100; ++k) {
        const amr::GridCoord c{cell(rng), cell(rng)};
        if (!map.isFree(c)) {
            continue;
        }
        const amr::LidarScan scan = scanAt(map, map.gridToWorld(c), {}, lidar, errors, 0.1 * k);
        CHECK(amr::detectMovingObstacles(map, scan, lidar.max_range, true, amr::DetectionConfig{}).empty());
        ++scans;
    }
    CHECK_EQ(scans, 100);
}

void testDiscIsDetectedAndFitted() {
    const GridMap map = obstaclesMap();
    const WorldPoint robot = map.gridToWorld({7, 12});  // open space between two blocks
    for (const double radius : {0.2, 0.3, 0.45}) {
        const Disc disc{{robot.x + 1.4, robot.y + 0.3}, radius};
        amr::RangeErrorModel errors(lidar360(0.01));
        const auto scan = scanAt(map, robot, {disc}, lidar360(0.01), errors, 0.0);
        const auto detections = amr::detectMovingObstacles(map, scan, 3.0, true, amr::DetectionConfig{});
        REQUIRE(detections.size() == 1);
        const amr::Detection& d = detections.front();
        CHECK(d.radius_fitted);
        CHECK_NEAR(d.center.x, disc.center.x, 0.03);
        CHECK_NEAR(d.center.y, disc.center.y, 0.03);
        CHECK_NEAR(d.radius, radius, 0.03);
    }
    // A coarse LiDAR sees only a few points of the disc: the center still lands behind
    // the hits, close to the truth.
    amr::LidarConfig coarse = lidar360(0.0);
    coarse.beams = 72;
    amr::RangeErrorModel errors(coarse);
    const Disc disc{{robot.x + 1.8, robot.y}, 0.25};
    const auto scan = scanAt(map, robot, {disc}, coarse, errors, 0.0);
    const auto detections = amr::detectMovingObstacles(map, scan, 3.0, true, amr::DetectionConfig{});
    REQUIRE(detections.size() == 1);
    CHECK_NEAR(detections.front().center.x, disc.center.x, 0.08);
    CHECK_NEAR(detections.front().center.y, disc.center.y, 0.08);
}

void testFitDiscOnAnArc() {
    std::vector<WorldPoint> points;
    const WorldPoint c{3.0, 2.0};
    for (double a = 2.2; a <= 4.1; a += 0.1) {  // the side facing the origin
        points.push_back({c.x + 0.4 * std::cos(a), c.y + 0.4 * std::sin(a)});
    }
    const amr::Detection d = amr::fitDisc(points, {0.0, 0.0}, amr::DetectionConfig{});
    CHECK(d.radius_fitted);
    CHECK_NEAR(d.center.x, c.x, 1e-6);
    CHECK_NEAR(d.center.y, c.y, 1e-6);
    CHECK_NEAR(d.radius, 0.4, 1e-6);
}

/// A thin post in front of the disc splits its returns in two; they are one obstacle.
void testFragmentsAreMerged() {
    const GridMap map = makeMap({"....................", "....................", "....................",
                                 ".........#..........", "....................", "...................."},
                                0.1);
    const WorldPoint robot{0.35, 0.35};
    const WorldPoint post = map.gridToWorld({9, 3});
    // The disc sits right behind the post, on the line from the robot.
    const double a = std::atan2(post.y - robot.y, post.x - robot.x);
    const Disc disc{{robot.x + 1.35 * std::cos(a), robot.y + 1.35 * std::sin(a)}, 0.3};
    amr::RangeErrorModel errors(lidar360(0.0));
    const auto scan = scanAt(map, robot, {disc}, lidar360(0.0), errors, 0.0);
    const auto detections = amr::detectMovingObstacles(map, scan, 3.0, true, amr::DetectionConfig{});
    REQUIRE(detections.size() == 1);
    CHECK_NEAR(detections.front().center.x, disc.center.x, 0.06);
    CHECK_NEAR(detections.front().center.y, disc.center.y, 0.06);
}

// ---------------------------------------------------------------------------------
// Tracking

/// One obstacle patrolling a lane in plain view: one track throughout, the velocity
/// right again shortly after every reversal, and both turning points learned.
void testPatrolIsTrackedThroughReversals() {
    const GridMap map = obstaclesMap();
    amr::DynamicObstacle ob;
    ob.route = {map.gridToWorld({7, 10}), map.gridToWorld({7, 15})};
    ob.speed = 0.3;
    ob.radius = 0.3;
    amr::ObstacleTracker tracker;
    std::vector<int> ids;
    double worst_velocity = 0.0, worst_time = 0.0;
    const double period = 2.0 * amr::routeLength(ob) / ob.speed;
    watch(map, map.gridToWorld({9, 12}), ob, 50.0, tracker, [&](double t, const amr::ObstacleTracker& tr) {
        for (const amr::Track& track : tr.tracks()) {
            if (track.confirmed && std::find(ids.begin(), ids.end(), track.id) == ids.end()) {
                ids.push_back(track.id);
            }
        }
        // 1.5 s after a reversal (and away from the next), the velocity is right. (The
        // lane's south end is partly hidden: just after turning there, the fits are poor.)
        const double phase = std::fmod(t, period / 2.0);
        if (t > 3.0 && phase > 1.5 && phase < period / 2.0 - 0.3 && !tr.tracks().empty()) {
            const amr::WorldPoint a = amr::obstaclePosition(ob, t - 0.05);
            const amr::WorldPoint b = amr::obstaclePosition(ob, t + 0.05);
            const amr::Track& track = tr.tracks().front();
            const double e = std::hypot(track.x[2] - (b.x - a.x) / 0.1, track.x[3] - (b.y - a.y) / 0.1);
            if (e > worst_velocity) {
                worst_velocity = e;
                worst_time = t;
            }
        }
    });
    CHECK_EQ(ids.size(), 1u);
    std::cout << "    worst velocity error 1.5 s after a reversal: " << worst_velocity << " m/s at t = "
              << worst_time << " s\n";
    CHECK(worst_velocity < 0.15);
    REQUIRE(confirmedTracks(tracker) == 1);
    const amr::Track& track = tracker.tracks().front();
    REQUIRE(track.lane);
    CHECK(track.has_low_end && track.has_high_end);
    // The lane is 2.5 m long, centred on the track's lane center: ends at about +-1.25.
    CHECK_NEAR(track.high_end - track.low_end, 2.5, 0.15);
    // Prediction 10 s ahead stays within the learned ends.
    const auto path = tracker.predictPath(track, 0.1, 100);
    for (const auto& p : path) {
        const double s = (p.center.x - track.lane_center.x) * track.lane_axis.x +
                         (p.center.y - track.lane_center.y) * track.lane_axis.y;
        CHECK(s >= track.low_end - 0.06 && s <= track.high_end + 0.06);
    }
    // And it is close to where the obstacle really goes.
    const double now = 50.0;
    double worst = 0.0;
    for (std::size_t j = 0; j <= 20; ++j) {
        const WorldPoint truth = amr::obstaclePosition(ob, now + 0.1 * static_cast<double>(j));
        worst = std::max(worst, std::hypot(path[j].center.x - truth.x, path[j].center.y - truth.y));
    }
    CHECK(worst < 0.25);
}

/// An obstacle that turns round where the robot cannot see it: no turning point is
/// learned at that end (it would be a guess), only at the end that was watched.
void testUnwatchedReversalIsNotLearned() {
    // A corridor; the robot sees its west part only (a wall hides the east part).
    const GridMap map = makeMap({"####################", "#..................#", "#..................#",
                                 "#########.##########", "#........#.........#", "####################"});
    amr::DynamicObstacle ob;
    ob.route = {map.gridToWorld({2, 1}), map.gridToWorld({17, 1})};
    ob.speed = 0.6;
    ob.radius = 0.25;
    amr::ObstacleTracker tracker;
    // The robot below the gap at (9, 3) sees the corridor through it, not its ends.
    watch(map, map.gridToWorld({9, 4}), ob, 60.0, tracker, [](double, const amr::ObstacleTracker&) {});
    for (const amr::Track& track : tracker.tracks()) {
        CHECK(!track.has_low_end && !track.has_high_end);
    }
}

void testTwoPointStartOfAFastObstacle() {
    const GridMap map = obstaclesMap();
    amr::DynamicObstacle ob;
    ob.route = {map.gridToWorld({7, 9}), map.gridToWorld({7, 19})};
    ob.speed = 2.0;
    ob.radius = 0.3;
    amr::ObstacleTracker tracker;
    bool checked = false;
    watch(map, map.gridToWorld({10, 13}), ob, 1.0, tracker, [&](double, const amr::ObstacleTracker& tr) {
        for (const amr::Track& track : tr.tracks()) {
            if (track.hits == 2 && !checked) {
                checked = true;
                CHECK(track.confirmed);
                CHECK_NEAR(track.x[2], 0.0, 0.25);
                CHECK_NEAR(track.x[3], 2.0, 0.25);  // moving down the map (+y)
            }
        }
    });
    CHECK(checked);
}

/// Negative evidence: an obstacle that vanishes in plain view is dropped within a
/// few scans. One that goes behind a wall coasts (kept, possibly still there).
void testMissesAndCoasting() {
    const GridMap map = obstaclesMap();
    amr::DynamicObstacle ob;
    ob.route = {map.gridToWorld({7, 10}), map.gridToWorld({7, 15})};
    ob.speed = 0.3;
    ob.radius = 0.3;
    amr::ObstacleTracker vanished;
    double gone = -1.0;
    watch(map, map.gridToWorld({9, 12}), ob, 8.0, vanished, [&](double t, const amr::ObstacleTracker& tr) {
        if (t > 5.0 && gone < 0.0 && tr.tracks().empty()) {
            gone = t;
        }
    }, 5.0);
    CHECK(gone > 0.0 && gone <= 5.0 + 0.1 * 4);
    // Behind a wall: the obstacle crosses from the robot's view into the shadow of
    // the block at columns 4-5.
    amr::DynamicObstacle hidden;
    hidden.route = {map.gridToWorld({6, 8}), map.gridToWorld({1, 8})};
    hidden.speed = 0.4;
    hidden.radius = 0.25;
    amr::ObstacleTracker coasting;
    bool kept_while_hidden = false;
    watch(map, map.gridToWorld({8, 8}), hidden, 8.0, coasting, [&](double t, const amr::ObstacleTracker& tr) {
        const WorldPoint c = amr::obstaclePosition(hidden, t);
        const bool behind_wall = c.x < 2.0;  // west of the block, out of sight
        if (behind_wall && confirmedTracks(tr) == 1) {
            kept_while_hidden = true;
        }
    });
    CHECK(kept_while_hidden);
}

// ---------------------------------------------------------------------------------
// Simulator with LiDAR perception

void testSimulatorWithLidarPerception() {
    const GridMap map = obstaclesMap();
    const auto loaded = amr::loadDynamicObstacles(std::string(AMR_SOURCE_DIR) + "/maps/obstacles_dynamic.csv", map);
    REQUIRE(loaded.ok);
    amr::DynamicScenario scenario;
    scenario.planning_map = &map;
    scenario.obstacles = loaded.obstacles;
    REQUIRE(scenario.perception.mode == amr::ObstaclePerception::Lidar);  // the default
    const auto plan = amr::planAStar(map, {1, 1}, {18, 18});
    for (const auto model : {amr::RobotModel::Point, amr::RobotModel::DiffDrive}) {
        amr::MotionConfig motion;
        motion.model = model;
        const auto a = amr::simulatePath(map, plan.path, motion, amr::Connectivity::Four, &scenario);
        const auto b = amr::simulatePath(map, plan.path, motion, amr::Connectivity::Four, &scenario);
        CHECK(a.status == amr::NavigationStatus::GoalReached);
        CHECK(a.min_dynamic_clearance > 0.0);
        CHECK(a.tracks_created >= 3);  // all three obstacles were seen
        REQUIRE(a.trajectory.size() == b.trajectory.size());  // deterministic
        CHECK_EQ(a.trajectory.back().state.x, b.trajectory.back().state.x);
    }
}

/// Occlusion awareness: at a blind corner the robot holds until it can see round it,
/// and with nothing hidden there it still gets through.
void testOcclusionAwareCreepsAtBlindCorners() {
    // An L-shaped corridor: the robot drives east, then north round the corner.
    const GridMap map = makeMap({"#########...#", "#########...#", "#########...#", "#########...#",
                                 "#...........#", "#...........#", "#...........#", "#############"});
    const auto plan = amr::planAStar(map, {1, 5}, {10, 0});
    REQUIRE(plan.success());
    amr::DynamicScenario scenario;  // no obstacle at all: only what might be hidden
    scenario.planning_map = &map;
    amr::MotionConfig motion;
    const auto plain = amr::simulatePath(map, plan.path, motion, amr::Connectivity::Four, &scenario);
    scenario.perception.occlusion_aware = true;
    const auto aware = amr::simulatePath(map, plan.path, motion, amr::Connectivity::Four, &scenario);
    CHECK(plain.status == amr::NavigationStatus::GoalReached);
    CHECK(aware.status == amr::NavigationStatus::GoalReached);
    CHECK(aware.wait_steps > plain.wait_steps);  // it held at the corner
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Static world: map subtraction leaves nothing", testStaticWorldHasNoDetections},
        {"A disc is detected and fitted", testDiscIsDetectedAndFitted},
        {"Disc fit on an arc is exact", testFitDiscOnAnArc},
        {"Fragments of one obstacle are merged", testFragmentsAreMerged},
        {"A patrol is tracked through its reversals", testPatrolIsTrackedThroughReversals},
        {"A reversal out of sight is not learned", testUnwatchedReversalIsNotLearned},
        {"Two-point start of a fast obstacle", testTwoPointStartOfAFastObstacle},
        {"Negative evidence and coasting", testMissesAndCoasting},
        {"Simulator with LiDAR perception", testSimulatorWithLidarPerception},
        {"Occlusion-aware robot creeps at blind corners", testOcclusionAwareCreepsAtBlindCorners},
    });
}
