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
// Safety margin. The planner sees obstacles inflated by robot_radius + margin
// (footprint.hpp), so the planned path keeps that much clearance; the margin is what
// the localization error may eat before the true robot touches something. A fixed
// margin cannot cover an error that grows (dead reckoning); the covariance-aware rule
// (safety_sigmas) uses the EKF's own claimed uncertainty instead.
#pragma once

#include <vector>

#include "ekf.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "simulator.hpp"

namespace amr {

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
};

/// Follow `path` (grid waypoints, start first) steering by the EKF estimate.
ClosedLoopResult simulateWithEstimate(const GridMap& map, const std::vector<GridCoord>& path,
                                      const ClosedLoopConfig& config,
                                      Connectivity connectivity = Connectivity::Four);

/// Clearance of a disc center to the nearest occupied cell or map edge, minus nothing:
/// the distance from `p` to the closest obstacle surface, m.
double obstacleDistance(const GridMap& map, WorldPoint p, double search_radius);

}  // namespace amr
