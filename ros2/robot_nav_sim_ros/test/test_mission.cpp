// A mission is the standalone simulator's pipeline: same plan, same trajectory,
// scans and localization, moving obstacles with replanning, and the same checks.
#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "astar.hpp"
#include "dynamic_obstacles.hpp"
#include "footprint.hpp"
#include "robot_nav_sim_ros/mission.hpp"
#include "simulator.hpp"

namespace rns = robot_nav_sim_ros;

namespace {

std::string mapsDir() {
    const char* dir = std::getenv("AMR_MAPS_DIR");
    return dir != nullptr ? dir : "maps";
}

amr::GridMap obstaclesMap() {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(mapsDir() + "/obstacles.csv", 0.5);
    EXPECT_TRUE(loaded.ok) << loaded.error;
    return loaded.map;
}

}  // namespace

TEST(Mission, MatchesTheStandalonePipeline) {
    const amr::GridMap map = obstaclesMap();
    for (const bool diff_drive : {false, true}) {
        for (const double radius : {0.0, 0.2}) {
            rns::MissionOptions options;
            options.motion.model = diff_drive ? amr::RobotModel::DiffDrive : amr::RobotModel::Point;
            options.motion.robot_radius = radius;
            const rns::Mission mission = rns::runMission(map, {1, 1}, {18, 18}, options);
            // The same steps the CLI takes (main.cpp).
            const amr::GridMap inflated = amr::inflateObstacles(map, radius);
            const amr::PlanResult plan = amr::planWithFootprint(map, inflated, {1, 1}, {18, 18});
            ASSERT_TRUE(plan.success());
            const amr::SimulationResult sim = amr::simulatePath(map, plan.path, options.motion);
            ASSERT_TRUE(mission.planned());
            EXPECT_EQ(mission.plan.path.size(), plan.path.size());
            EXPECT_EQ(mission.plan.path_cost, plan.path_cost);
            ASSERT_EQ(mission.sim.trajectory.size(), sim.trajectory.size());
            for (std::size_t k = 0; k < sim.trajectory.size(); ++k) {
                EXPECT_EQ(mission.sim.trajectory[k].state.x, sim.trajectory[k].state.x);
                EXPECT_EQ(mission.sim.trajectory[k].state.theta, sim.trajectory[k].state.theta);
            }
            EXPECT_EQ(mission.sim.status, amr::NavigationStatus::GoalReached);
            EXPECT_TRUE(mission.scans.empty());
            EXPECT_TRUE(mission.localization.empty());
            for (std::size_t i = 0; i < map.cellCount(); ++i) {
                EXPECT_EQ(mission.planning_map.data()[i], inflated.data()[i]);
            }
        }
    }
}

TEST(Mission, SensorsLocalizationAndMovingObstacles) {
    const amr::GridMap map = obstaclesMap();
    rns::MissionOptions options;
    options.motion.model = amr::RobotModel::DiffDrive;
    options.lidar.beams = 36;
    options.ekf.enabled = true;
    const auto obstacles = amr::loadDynamicObstacles(mapsDir() + "/obstacles_dynamic.csv", map);
    ASSERT_TRUE(obstacles.ok) << obstacles.error;
    options.obstacles = obstacles.obstacles;
    ASSERT_EQ(rns::validateMissionOptions(options), "");
    const rns::Mission mission = rns::runMission(map, {1, 1}, {18, 18}, options);
    ASSERT_TRUE(mission.planned());
    const std::size_t n = mission.sim.trajectory.size();
    EXPECT_EQ(mission.scans.size(), n);
    EXPECT_EQ(mission.localization.size(), n);
    EXPECT_EQ(mission.sim.status, amr::NavigationStatus::GoalReached);
    // Replans happened, and pathAt follows them: the initial plan before the first,
    // the replanned path (starting where it was planned from) after it.
    ASSERT_FALSE(mission.sim.replans.empty());
    const amr::ReplanEvent* first = nullptr;
    for (const auto& event : mission.sim.replans) {
        if (event.status == amr::PlanStatus::Success) {
            first = &event;
            break;
        }
    }
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(&mission.pathAt(0.0), &mission.plan.path);
    const auto& replanned = mission.pathAt(first->time);
    EXPECT_EQ(replanned.front(), first->from);
    EXPECT_EQ(replanned.back(), (amr::GridCoord{18, 18}));
}

TEST(Mission, PlanningFailureLeavesNoTrajectory) {
    const amr::GridMap map = obstaclesMap();
    rns::MissionOptions options;
    // (4, 4) is an obstacle cell of the obstacles map.
    ASSERT_TRUE(map.isOccupied({4, 4}));
    const rns::Mission mission = rns::runMission(map, {1, 1}, {4, 4}, options);
    EXPECT_FALSE(mission.planned());
    EXPECT_EQ(mission.plan.status, amr::PlanStatus::GoalOccupied);
    EXPECT_TRUE(mission.sim.trajectory.empty());
    EXPECT_TRUE(mission.pathAt(0.0).empty());
}

TEST(Mission, Validation) {
    rns::MissionOptions options;
    EXPECT_EQ(rns::validateMissionOptions(options), "");
    options.ekf.enabled = true;  // needs the differential drive
    EXPECT_NE(rns::validateMissionOptions(options).find("differential-drive"), std::string::npos);
    options = {};
    options.motion.speed = -1.0;
    EXPECT_NE(rns::validateMissionOptions(options).find("motion"), std::string::npos);
    options = {};
    options.lidar.beams = 8;
    options.lidar.fov = 0.0;
    EXPECT_NE(rns::validateMissionOptions(options).find("LiDAR"), std::string::npos);
    options = {};
    options.replanning.sensing_range = 0.0;
    EXPECT_NE(rns::validateMissionOptions(options).find("replanning"), std::string::npos);
}
