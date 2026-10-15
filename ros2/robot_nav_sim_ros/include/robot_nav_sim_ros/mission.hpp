// One navigation mission: the standalone simulator's pipeline (robot_simulator's
// main.cpp) as a library call, independent of ROS.
//
//   inflate the map by the robot radius -> plan with A* (planWithFootprint)
//   -> simulate the robot along the path (optionally among moving obstacles,
//      replanning) -> LiDAR scans along the trajectory -> EKF localization
//
// The ROS simulator node runs a mission whenever it has a map, a start and a goal,
// and then plays the result back in (scaled) real time.
#pragma once

#include <string>
#include <vector>

#include "astar.hpp"
#include "dynamic_obstacles.hpp"
#include "ekf.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace robot_nav_sim_ros {

struct MissionOptions {
    amr::MotionConfig motion;
    amr::Connectivity connectivity = amr::Connectivity::Four;
    amr::LidarConfig lidar;
    amr::EkfConfig ekf;
    std::vector<amr::DynamicObstacle> obstacles;  ///< empty: a static world
    amr::ReplanningConfig replanning;
    amr::PerceptionConfig perception;  ///< how moving obstacles are seen (default: LiDAR)
};

/// Empty string if the options are valid (the CLI's checks), otherwise the reason.
std::string validateMissionOptions(const MissionOptions& options);

struct Mission {
    amr::GridCoord start;
    amr::GridCoord goal;
    amr::GridMap planning_map;               ///< the map inflated by the robot radius
    amr::PlanResult plan;
    amr::SimulationResult sim;               ///< trajectory empty if planning failed
    std::vector<amr::LidarScan> scans;       ///< one per trajectory sample (LiDAR on)
    std::vector<amr::LocalizationSample> localization;  ///< one per sample (EKF on)

    bool planned() const { return plan.success(); }
    /// The path being followed at mission time t: the initial plan, or the latest
    /// successful replan (from the cell it was planned from).
    const std::vector<amr::GridCoord>& pathAt(double t) const;
};

/// Run a mission from `start` to `goal` (cells). Options must be valid.
Mission runMission(const amr::GridMap& map, amr::GridCoord start, amr::GridCoord goal,
                   const MissionOptions& options);

}  // namespace robot_nav_sim_ros
