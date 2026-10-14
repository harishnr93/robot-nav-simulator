#include "robot_nav_sim_ros/mission.hpp"

#include "footprint.hpp"

namespace robot_nav_sim_ros {

std::string validateMissionOptions(const MissionOptions& options) {
    if (std::string e = amr::validateMotionConfig(options.motion); !e.empty()) {
        return "invalid motion parameters: " + e;
    }
    if (std::string e = amr::validateLidarConfig(options.lidar); !e.empty()) {
        return "invalid LiDAR parameters: " + e;
    }
    if (std::string e = amr::validateEkfConfig(options.ekf); !e.empty()) {
        return "invalid EKF parameters: " + e;
    }
    if (options.ekf.enabled && options.motion.model != amr::RobotModel::DiffDrive) {
        return "the EKF needs the differential-drive model (robot_model: diff-drive)";
    }
    if (std::string e = amr::validateReplanningConfig(options.replanning); !e.empty()) {
        return "invalid replanning parameters: " + e;
    }
    return {};
}

const std::vector<amr::GridCoord>& Mission::pathAt(double t) const {
    const std::vector<amr::GridCoord>* current = &plan.path;
    for (const amr::ReplanEvent& event : sim.replans) {
        if (event.time > t + 1e-9) {
            break;
        }
        if (event.status == amr::PlanStatus::Success) {
            current = &event.path;
        }
    }
    return *current;
}

Mission runMission(const amr::GridMap& map, amr::GridCoord start, amr::GridCoord goal,
                   const MissionOptions& options) {
    Mission mission;
    mission.start = start;
    mission.goal = goal;
    mission.planning_map = amr::inflateObstacles(map, options.motion.robot_radius);
    mission.plan = amr::planWithFootprint(map, mission.planning_map, start, goal,
                                          options.connectivity);
    if (!mission.plan.success()) {
        return mission;
    }
    amr::DynamicScenario scenario;
    scenario.planning_map = &mission.planning_map;
    scenario.obstacles = options.obstacles;
    scenario.replanning = options.replanning;
    mission.sim = amr::simulatePath(map, mission.plan.path, options.motion, options.connectivity,
                                    options.obstacles.empty() ? nullptr : &scenario);
    mission.scans = amr::scanTrajectory(map, mission.sim.trajectory, options.lidar);
    mission.localization = amr::runLocalization(map, mission.sim.trajectory, mission.scans,
                                                options.lidar, options.ekf);
    return mission;
}

}  // namespace robot_nav_sim_ros
