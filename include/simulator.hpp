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
#include <vector>

#include "grid_map.hpp"
#include "robot.hpp"
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

/// Simulate the robot following `path` (grid waypoints, start first).
SimulationResult simulatePath(const GridMap& map, const std::vector<GridCoord>& path,
                              const MotionConfig& config,
                              Connectivity connectivity = Connectivity::Four);

}  // namespace amr
