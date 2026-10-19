// Time-stepped simulation of a point robot following a planned grid path.
//
// The simulator:
//   1. checks the motion configuration and the path structure (non-empty,
//      consecutive waypoints are neighbors: straight, or also diagonal for
//      8-connectivity);
//   2. places the robot at the center of the first waypoint (t = 0);
//   3. each timestep, moves toward the current target waypoint (robot.hpp) and
//      collision-checks the movement segment against the occupancy grid;
//   4. advances to the next waypoint when the current one is reached;
//   5. stops at the goal, on the first collision, or when max_time is reached.
//
// Collision model:
//   * robot_radius == 0 (point robot): a segment is collision-free if every sampled
//     point on it lies in a free, in-bounds cell; for 8-connectivity it must also
//     not touch any occupied cell (catches a diagonal cutting an obstacle corner).
//   * robot_radius > 0 (disc robot): a step is collision-free if the disc swept
//     along it never overlaps an occupied cell or the map exterior (exact test,
//     see footprint.hpp). Touching is allowed.
// The check always uses the real map, never the inflated planning map, so it is an
// independent verification of the planned path.
//
// Dynamic obstacles and replanning (optional DynamicScenario, dynamic_obstacles.hpp):
//   * Each step the robot perceives the obstacles: by default with its own LiDAR
//     (a scan of the map and the obstacles; returns the map does not explain are
//     clustered into discs and tracked, obstacle_tracking.hpp), whose tracks give each
//     obstacle's predicted path over the prediction horizon (straight on, or bouncing
//     at the turning points learned so far) and its uncertainty. With the oracle, an
//     obstacle within sensing range and line of sight is seen at its true position,
//     with velocity = displacement / dt from two consecutive sightings, and predicted
//     straight on. Planning-map cells whose centers are closer than obstacle
//     radius + robot radius + margin to the predicted sweep count as blocked.
//   * If a blocked cell lies on the rest of the path, the robot replans to the goal
//     with the same AStarPlanner object (its buffers are reused): from its current
//     cell when it stands on a waypoint, from the waypoint it is driving to
//     otherwise, or, if that one is blocked, from the waypoint it came from (it
//     drives back first, unless backing up is itself predicted to end in contact:
//     then it heads for whichever end of its segment keeps the larger predicted gap).
//     If there is no path yet, it keeps following the old path
//     while its next waypoint is free, backs off to the previous waypoint if the one
//     ahead is blocked, waits on a waypoint whose successor is blocked, and tries
//     again next step.
//   * A look-ahead gate: the robot also waits instead of following the path when that
//     would come within the margin of a seen obstacle within the prediction horizon
//     (each tracked obstacle inflated by its prediction uncertainty) and standing is
//     safer. After waiting out the patience the margin is waived: any contact-free
//     motion is taken (an obstacle patrolling right next to the path never leaves).
//   * Optionally (PerceptionConfig::occlusion_aware) space hidden behind walls counts
//     as possibly occupied: the robot creeps round blind corners.
//   * Obstacles do not avoid the robot. Collisions with them are checked exactly
//     against their true motion during every step (moving, turning or waiting).
//
// Robot models (MotionConfig::model):
//   * Point: moves straight toward the next waypoint at constant speed.
//   * DiffDrive: pose (x, y, theta), inputs (v, omega), unicycle kinematics
//     (diff_drive.hpp). Turn-then-drive controller: if the heading differs from the
//     direction to the next waypoint by more than kDriveAlignment, the robot rotates
//     in place (v = 0) with the P heading controller, limited to max_angular_speed;
//     otherwise it drives straight to the waypoint (omega = 0) exactly like the
//     point robot. The center therefore stays on the planned path, so the
//     planner's footprint and corner guarantees still hold. Collinear waypoints
//     are passed without stopping.
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "dynamic_obstacles.hpp"
#include "grid_map.hpp"
#include "lidar_config.hpp"
#include "robot.hpp"
#include "tracking_config.hpp"
#include "types.hpp"

namespace amr {

enum class NavigationStatus {
    NotRun,         ///< simulation was not executed (e.g. planning failed)
    GoalReached,    ///< final position within goal tolerance, no collision
    Collision,      ///< a movement segment entered an occupied or out-of-bounds cell
    Timeout,        ///< max_time elapsed before reaching the goal
    InvalidPath,    ///< empty path, or consecutive waypoints not 4-connected
    InvalidConfig,  ///< motion parameters failed validateMotionConfig()
};

inline const char* toString(NavigationStatus status) {
    switch (status) {
        case NavigationStatus::NotRun:        return "NOT_RUN";
        case NavigationStatus::GoalReached:   return "GOAL_REACHED";
        case NavigationStatus::Collision:     return "COLLISION";
        case NavigationStatus::Timeout:       return "TIMEOUT";
        case NavigationStatus::InvalidPath:   return "INVALID_PATH";
        case NavigationStatus::InvalidConfig: return "INVALID_CONFIG";
    }
    return "UNKNOWN";
}

/// One recorded trajectory sample.
struct TrajectorySample {
    RobotState state;
    /// Index of the waypoint the robot is heading to after this sample.
    /// Equals the last waypoint index once the goal has been reached.
    std::size_t target_waypoint_index = 0;
    /// Differential drive: velocities applied during the step that ended at this
    /// sample (0 for the first sample).
    double linear_velocity = 0.0;   ///< m/s
    double angular_velocity = 0.0;  ///< rad/s
};

/// Planner for moving obstacles.
enum class DynamicPlanner {
    /// Replan shortest paths on the planning map with the cells about to be swept
    /// blocked; wait, retreat and gate (the rules below).
    Reactive,
    /// Plan over space and time (spacetime_planner.hpp) against the predicted motion of
    /// every perceived obstacle, waiting where the plan says; replan on every cell and
    /// at least every spacetime_period. Reactive rules take over when no timed plan
    /// is found; the look-ahead gate stays as a safety net.
    SpaceTime,
};

/// Sensing and replanning parameters for dynamic obstacles.
struct ReplanningConfig {
    double sensing_range = 3.0;  ///< obstacles are seen within this distance (to their edge), m
    double margin = 0.15;        ///< extra clearance around a seen obstacle when planning, m
    double prediction_horizon = 1.0;  ///< seconds of estimated obstacle motion blocked, s
    double patience = 6.0;       ///< how long the path may stay blocked before a detour, s
    /// When backing up to the previous waypoint would itself end in contact, head for
    /// whichever end of the current segment keeps the larger predicted gap.
    bool choose_retreat = true;
    /// How the robot plans around moving obstacles.
    DynamicPlanner planner = DynamicPlanner::Reactive;
    double spacetime_horizon = 20.0;  ///< SpaceTime: seconds of predicted motion planned against
    double spacetime_period = 0.0;    ///< SpaceTime: replan at least this often (on a cell), s (0: every step)
    /// SpaceTime: plan against the obstacles' true future motion (an omniscient upper
    /// bound for benchmarks), instead of the perceived and predicted one.
    bool perfect_prediction = false;
    /// Livelock detection: if the robot's remaining distance to the goal (along the
    /// static planning map) has not improved for this long, it is stuck, shuttling
    /// (livelock) or waiting (deadlock), and switches strategy: reactive <->
    /// space-time. The new strategy gets a full window before the next switch.
    /// 0 disables the detector.
    double livelock_window = 15.0;
};

/// How the robot perceives moving obstacles.
enum class ObstaclePerception {
    /// Their true positions when within sensing range and in line of sight of their
    /// centers; velocity from two consecutive sightings (the original shortcut).
    Oracle,
    /// Its own LiDAR only: returns the map does not explain are clustered into discs
    /// and tracked (obstacle_tracking.hpp).
    Lidar,
};

/// Perception of moving obstacles (default: the robot's own LiDAR).
struct PerceptionConfig {
    ObstaclePerception mode = ObstaclePerception::Lidar;
    /// Lidar mode. With beams = 0: 360 beams over a full circle, range = sensing range.
    LidarConfig lidar;
    DetectionConfig detection;
    TrackerConfig tracker;
    /// Radius added to a tracked obstacle for its prediction uncertainty (2 sigma),
    /// at most this much, m.
    double max_inflation = 0.1;
    /// Also inflate the cells blocked for replanning (not only the look-ahead gate).
    bool inflate_planning = false;
    /// Lidar mode, learned reversals: a lane track whose turning point ahead has not
    /// been seen yet is predicted both turning at the wall ahead and turning now,
    /// instead of running on through walls.
    bool turning_hypotheses = true;
    /// Occluded space as possibly occupied: free space hidden behind walls may hold an
    /// obstacle ("phantom") that moves at up to phantom_speed. Before each step, the
    /// robot looks from where the step would take it: if a phantom emerging from the
    /// space still hidden from there could reach it within phantom_reaction, the step
    /// waits (the robot creeps round corners, seeing more with every step). Space
    /// beyond the sensing range does not count: obstacles coming from there are seen
    /// in time. After waiting out the patience the robot commits and ignores phantoms
    /// for phantom_commit meters.
    bool occlusion_aware = false;
    double phantom_speed = 1.0;      ///< assumed top speed of unseen obstacles, m/s
    double phantom_radius = 0.3;     ///< m
    double phantom_reaction = 0.5;   ///< time the robot needs to react once one appears, s
    double phantom_commit = 1.5;     ///< m driven ignoring phantoms after the patience
};

/// Optional dynamic-obstacle scenario for simulatePath().
struct DynamicScenario {
    const GridMap* planning_map = nullptr;  ///< map A* replans on (the inflated map)
    std::vector<DynamicObstacle> obstacles;
    ReplanningConfig replanning;
    PerceptionConfig perception;
};

/// One replanning attempt.
struct ReplanEvent {
    double time = 0.0;             ///< s
    GridCoord from;                ///< cell the new path starts from
    PlanStatus status = PlanStatus::NoPath;
    std::vector<GridCoord> path;   ///< new path (empty unless status is Success)
    GridCost cost;                 ///< exact cost of the new path
    std::size_t expanded_nodes = 0;
    double planning_time_ms = 0.0;
    std::size_t blocked_cells = 0; ///< planning cells blocked by seen obstacles
};

/// Upper bound on validated replanning parameters.
constexpr double kMaxSensingRange = 1000.0;
constexpr double kMaxPredictionHorizon = 100.0;
/// Upper bound accepted for ReplanningConfig::livelock_window, s.
constexpr double kMaxLivelockWindow = 3600.0;

/// Empty string if the replanning parameters are valid, otherwise the reason.
std::string validateReplanningConfig(const ReplanningConfig& config);

struct SimulationResult {
    NavigationStatus status = NavigationStatus::NotRun;
    /// Samples at t = 0, dt, 2 dt, ... Includes the initial and final positions.
    /// For a single-waypoint path the one sample is both. Empty if not run.
    std::vector<TrajectorySample> trajectory;
    std::size_t steps = 0;                ///< timesteps executed
    std::size_t collision_count = 0;      ///< 0 or 1: the simulation stops at a collision
    double distance_traveled = 0.0;       ///< m
    double final_goal_error = 0.0;        ///< distance from final position to goal center, m
    // Differential drive only:
    double turn_time = 0.0;               ///< time spent rotating in place, s
    double total_rotation = 0.0;          ///< sum of |omega| * dt, rad
    std::size_t turns = 0;                ///< number of separate turn-in-place maneuvers
    // Dynamic obstacles only:
    std::vector<ReplanEvent> replans;     ///< every replanning attempt, in time order
    std::size_t wait_steps = 0;           ///< steps spent waiting for a blocked path
    int collided_obstacle = -1;           ///< index of the obstacle hit, -1 if none
    std::size_t tracks_created = 0;       ///< Lidar perception: tracks started over the run
    int strategy_switches = 0;            ///< livelocks detected (strategy switched)
    int collided_robot = -1;              ///< multi_robot: id of the robot hit, -1 if none
    double min_dynamic_clearance = 0.0;   ///< closest gap to any obstacle edge over the run, m

    bool success() const { return status == NavigationStatus::GoalReached; }
    /// final timestamp - initial timestamp, s (0 if no trajectory).
    double travelTime() const;
};

/// True if every point of segment a->b lies in a free, in-bounds cell
/// (sampled at intervals of at most resolution / kCollisionSamplesPerCell).
bool isSegmentCollisionFree(const GridMap& map, WorldPoint a, WorldPoint b);

/// Differential drive: the robot drives only when its heading is within this
/// angle of the direction to the next waypoint; otherwise it turns in place, rad.
constexpr double kDriveAlignment = 1e-6;

/// Number of collision samples per cell length along a segment.
constexpr int kCollisionSamplesPerCell = 20;

/// Simulate the robot following `path` (grid waypoints, start first). With a
/// dynamic scenario the robot also avoids moving obstacles by replanning.
SimulationResult simulatePath(const GridMap& map, const std::vector<GridCoord>& path,
                              const MotionConfig& config,
                              Connectivity connectivity = Connectivity::Four,
                              const DynamicScenario* dynamic = nullptr);

/// Another robot, as the others see it: where it is now (multi_robot.hpp).
struct MovingDisc {
    int id = 0;
    WorldPoint center;
    double radius = 0.0;
};

class NavigatorState;

/// One robot's navigation, one timestep at a time: simulatePath runs a navigator to
/// the end; several navigators stepped together are robots that see each other
/// (multi_robot.hpp). The map, path and scenario must outlive the navigator.
class Navigator {
public:
    Navigator(const GridMap& map, const std::vector<GridCoord>& path, const MotionConfig& config,
              Connectivity connectivity = Connectivity::Four, const DynamicScenario* dynamic = nullptr);
    ~Navigator();
    Navigator(Navigator&&) noexcept;
    Navigator& operator=(Navigator&&) noexcept;

    /// True once the run ended (goal, collision, timeout or an invalid request).
    bool finished() const;
    /// Decide and take one timestep. `others` (other robots, perceived like moving
    /// obstacles) are where they are at the start of the step; contact with them is
    /// not checked here (the caller knows how everyone moved).
    void step(const std::vector<MovingDisc>& others = {});
    /// End the run with a collision with another robot (`other` = its id).
    void collideWithRobot(int other);
    /// Where the robot is (center) and was at the start of the last step.
    WorldPoint position() const;
    WorldPoint previousPosition() const;
    /// The result so far (complete once finished()).
    const SimulationResult& result() const;

private:
    std::unique_ptr<NavigatorState> state_;
};

}  // namespace amr
