// Steering by the estimate: the differential-drive robot is controlled with its EKF
// estimate instead of its true pose, so localization error becomes path error.
//
// Each timestep:
//   1. control from the estimate: heading error e to the next waypoint (as seen from
//      the estimate); |e| > drive_alignment: turn in place (P control, limited to
//      max_angular_speed); otherwise drive at the speed (no farther than the waypoint
//      looks) while correcting the heading (omega = gain x e). A waypoint counts as
//      reached when the estimate is within one step of it; the run ends when the
//      estimate is within goal_tolerance of the goal;
//   2. the true robot moves exactly as commanded (unicycle kinematics), and its
//      swept disc (robot_radius) is checked against the real map: touching an
//      obstacle is a collision;
//   3. the wheel odometry measures the true increment with noise (the error model of
//      ekf.hpp, variances from the commanded increment) and the EKF predicts;
//   4. the LiDAR scans from the true pose and the EKF corrects.
// The EKF treats the motion as "turn, then drive" (ekf.hpp); with the small heading
// corrections made while driving this is an approximation (exact while turning in
// place or driving straight).
//
// Realistic actuators (ActuatorConfig, off by default). The wheels do not do exactly
// what is commanded:
//   - lag: the actual speed and turn rate follow the commands with a first-order lag
//     (time constant `lag`; exact over a step for a held command);
//   - speed_scale: the robot drives speed_scale x the commanded speed (wheel radius
//     error);
//   - turn_bias: a constant extra turn rate (unequal wheels, a slipping wheel), rad/s;
//   - rate_noise: a Gauss-Markov turn-rate disturbance with stationary std
//     rate_noise and correlation time noise_time (uneven floor).
// The odometry measures the motion that happened (with its own noise, as before).
//
// Path tracking (PathTracker):
//   - TurnThenDrive: as above (turn out large heading errors in place, then drive at
//     the next waypoint with heading corrections);
//   - LineFollow: follow the planned segment rather than aim at its end: turn in place
//     at corners, then omega = k_heading x heading error to the segment direction
//     - k_cross x cross-track error - k_integral x its integral (the integral
//     cancels a constant turn bias; clamped against windup). It slows down before
//     corners and the goal (stop_lead), so that a lagging drive stops where it should,
//     and aims at the waypoint once past the end of its segment.
// The cross-track error is the true center's distance to the planned polyline
// (ClosedLoopResult::cross_track), the error that the planning margin has to cover.
//
// Safety margin. The planner sees obstacles inflated by robot_radius + margin
// (footprint.hpp), so the planned path keeps that much clearance; the margin is what
// the localization error may eat before the true robot touches something. A fixed
// margin cannot cover an error that grows (dead reckoning); the covariance-aware rule
// (safety_sigmas) uses the EKF's own claimed uncertainty instead.
#pragma once

#include <cstdint>
#include <vector>

#include "ekf.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "simulator.hpp"

namespace amr {

/// How the true robot departs from the commands (see the header comment).
struct ActuatorConfig {
    double lag = 0.0;          ///< s, first-order lag of speed and turn rate (0 = none)
    double speed_scale = 1.0;  ///< actual / commanded speed
    double turn_bias = 0.0;    ///< rad/s, constant extra turn rate
    double rate_noise = 0.0;   ///< rad/s, std of the correlated turn-rate disturbance
    double noise_time = 1.0;   ///< s, its correlation time
    std::uint32_t seed = 0;

    bool ideal() const {
        return lag == 0.0 && speed_scale == 1.0 && turn_bias == 0.0 && rate_noise == 0.0;
    }
};

enum class PathTracker { TurnThenDrive, LineFollow };

struct LineFollowGains {
    double heading = 2.0;   ///< 1/s
    double cross = 3.0;     ///< rad/s per m of cross-track error
    double integral = 1.5;  ///< rad/s per m s
    double max_integral = 0.3;  ///< m s (anti-windup)
    /// s: slow down so as to reach the next corner (or the goal) no sooner than this
    /// plus a step; a lagging drive keeps rolling for about its time constant.
    double stop_lead = 0.3;
};

struct ClosedLoopConfig {
    MotionConfig motion;              ///< speed, dt, goal tolerance, limits, robot_radius
    LidarConfig lidar;                ///< beams = 0: no LiDAR (dead reckoning)
    EkfConfig ekf;                    ///< odometry noise and the filter's settings
    double drive_alignment = 0.15;    ///< rad: larger heading errors are turned out in place
    /// Covariance-aware safety (0 = off): do not drive a step whose estimated end
    /// position is closer to an obstacle than safety_sigmas x the claimed position
    /// std (minus the radius); hold (turning is still allowed) while the scans bring
    /// the uncertainty down, for at most safety_patience, then drive on for
    /// safety_commit before holding again.
    double safety_sigmas = 0.0;
    double safety_patience = 3.0;     ///< s held at most, then drive on for safety_commit m
    double safety_commit = 1.0;       ///< m
    ActuatorConfig actuator;          ///< ideal by default: the robot moves as commanded
    PathTracker tracker = PathTracker::TurnThenDrive;
    LineFollowGains line_follow;
    /// Control with the true pose instead of the estimate (perfect localization):
    /// isolates the tracking error of the actuators and the controller.
    bool steer_by_truth = false;
};

struct ClosedLoopResult {
    NavigationStatus status = NavigationStatus::NotRun;
    std::vector<TrajectorySample> trajectory;      ///< the true motion
    std::vector<LocalizationSample> localization;  ///< truth, dead reckoning, estimate per sample
    double final_goal_error = 0.0;                 ///< true distance to the goal at the end, m
    double min_clearance = 0.0;                    ///< closest the true disc came to an obstacle, m
    double max_error = 0.0;                        ///< largest true position error of the estimate, m
    double max_claimed_std = 0.0;                  ///< largest position std the EKF claimed, m
    std::size_t safety_holds = 0;                  ///< steps held by the safety rule
    std::vector<double> cross_track;               ///< per sample: true center to the planned path, m
    double max_cross_track = 0.0;                  ///< m
};

/// Follow `path` (grid waypoints, start first) steering by the EKF estimate.
ClosedLoopResult simulateWithEstimate(const GridMap& map, const std::vector<GridCoord>& path,
                                      const ClosedLoopConfig& config,
                                      Connectivity connectivity = Connectivity::Four);

/// Clearance of a disc center to the nearest occupied cell or map edge, minus nothing:
/// the distance from `p` to the closest obstacle surface, m.
double obstacleDistance(const GridMap& map, WorldPoint p, double search_radius);

}  // namespace amr
