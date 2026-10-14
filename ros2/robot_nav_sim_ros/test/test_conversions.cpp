// Frame conversions between the simulator (y down, clockwise headings, row 0 at the
// top) and ROS (y up, counter-clockwise, OccupancyGrid row 0 at the bottom).
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "lidar.hpp"
#include "robot_nav_sim_ros/conversions.hpp"

namespace rns = robot_nav_sim_ros;

namespace {

constexpr double kPi = 3.14159265358979323846;
const builtin_interfaces::msg::Time kStamp;

amr::GridMap testMap() {
    // 4 x 3 at 0.5 m; occupied: (0, 0) top-left and (3, 2) bottom-right.
    amr::GridMap map(4, 3, 0.5);
    map.setOccupied({0, 0}, true);
    map.setOccupied({3, 2}, true);
    return map;
}

}  // namespace

TEST(Conversions, OccupancyGridRowsAreFlipped) {
    const amr::GridMap map = testMap();
    const auto grid = rns::toOccupancyGrid(map, "map", kStamp);
    EXPECT_EQ(grid.info.width, 4u);
    EXPECT_EQ(grid.info.height, 3u);
    EXPECT_FLOAT_EQ(grid.info.resolution, 0.5f);
    EXPECT_DOUBLE_EQ(grid.info.origin.position.x, 0.0);
    EXPECT_DOUBLE_EQ(grid.info.origin.orientation.w, 1.0);
    // Simulator top-left (0, 0) is the last ROS row (top); bottom-right (3, 2) the first.
    EXPECT_EQ(grid.data[2 * 4 + 0], rns::kOccupiedValue);
    EXPECT_EQ(grid.data[0 * 4 + 3], rns::kOccupiedValue);
    EXPECT_EQ(grid.data[0], rns::kFreeValue);
    int occupied = 0;
    for (const int8_t v : grid.data) {
        occupied += v == rns::kOccupiedValue;
    }
    EXPECT_EQ(occupied, 2);
}

TEST(Conversions, OccupancyGridRoundTrip) {
    std::mt19937 rng(1);
    std::bernoulli_distribution occupied(0.3);
    amr::GridMap map(17, 9, 0.25);
    for (int y = 0; y < 9; ++y) {
        for (int x = 0; x < 17; ++x) {
            map.setOccupied({x, y}, occupied(rng));
        }
    }
    const amr::GridMap back = rns::fromOccupancyGrid(rns::toOccupancyGrid(map, "map", kStamp));
    ASSERT_EQ(back.width(), 17);
    ASSERT_EQ(back.height(), 9);
    EXPECT_DOUBLE_EQ(back.resolution(), 0.25);
    for (std::size_t i = 0; i < map.cellCount(); ++i) {
        EXPECT_EQ(back.data()[i], map.data()[i]) << "cell " << i;
    }
}

TEST(Conversions, UnknownAndHighCostCellsAreObstacles) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info.width = 4;
    grid.info.height = 1;
    grid.info.resolution = 1.0f;
    grid.data = {0, 49, 50, -1};
    const amr::GridMap map = rns::fromOccupancyGrid(grid);
    EXPECT_TRUE(map.isFree({0, 0}));
    EXPECT_TRUE(map.isFree({1, 0}));
    EXPECT_TRUE(map.isOccupied({2, 0}));
    EXPECT_TRUE(map.isOccupied({3, 0}));
}

TEST(Conversions, PointsAndHeadings) {
    const rns::MapGeometry g{4, 3, 0.5};  // 2.0 m x 1.5 m
    // The simulator's top-left cell center is near the ROS map's top edge.
    const auto p = rns::toRosPoint(g, {0.25, 0.25});
    EXPECT_DOUBLE_EQ(p.x, 0.25);
    EXPECT_DOUBLE_EQ(p.y, 1.25);
    const amr::WorldPoint back = rns::toSimPoint(g, p.x, p.y);
    EXPECT_DOUBLE_EQ(back.x, 0.25);
    EXPECT_DOUBLE_EQ(back.y, 0.25);
    // Facing "down the map" in the simulator (+y, heading +90 deg) is facing -y in ROS.
    EXPECT_NEAR(rns::toRosYaw(kPi / 2), -kPi / 2, 1e-12);
    EXPECT_NEAR(rns::toSimHeading(rns::toRosYaw(0.7)), 0.7, 1e-12);
    // A unit step along the simulator heading lands where the ROS yaw points.
    for (const double heading : {0.0, 0.4, 2.0, -1.3, kPi}) {
        const amr::WorldPoint a{1.0, 0.7};
        const amr::WorldPoint b{a.x + 0.1 * std::cos(heading), a.y + 0.1 * std::sin(heading)};
        const auto ra = rns::toRosPoint(g, a);
        const auto rb = rns::toRosPoint(g, b);
        const double yaw = rns::toRosYaw(heading);
        EXPECT_NEAR(rb.x - ra.x, 0.1 * std::cos(yaw), 1e-12);
        EXPECT_NEAR(rb.y - ra.y, 0.1 * std::sin(yaw), 1e-12);
    }
}

TEST(Conversions, Quaternions) {
    for (const double yaw : {0.0, 0.5, -2.9, kPi / 2, 3.1}) {
        EXPECT_NEAR(rns::quaternionToYaw(rns::yawToQuaternion(yaw)), yaw, 1e-12);
    }
}

TEST(Conversions, CellsFromRosPositions) {
    const rns::MapGeometry g{4, 3, 0.5};
    // ROS (0.3, 1.4): x in column 0, y 1.4 m up = 0.1 m below the top -> row 0.
    const auto cell = rns::toSimCell(g, 0.3, 1.4);
    ASSERT_TRUE(cell.has_value());
    EXPECT_EQ(cell->x, 0);
    EXPECT_EQ(cell->y, 0);
    const auto bottom = rns::toSimCell(g, 1.9, 0.1);
    ASSERT_TRUE(bottom.has_value());
    EXPECT_EQ(bottom->x, 3);
    EXPECT_EQ(bottom->y, 2);
    EXPECT_FALSE(rns::toSimCell(g, -0.1, 1.0).has_value());
    EXPECT_FALSE(rns::toSimCell(g, 1.0, 1.6).has_value());
}

TEST(Conversions, PathPosesFaceTheNextCell) {
    const rns::MapGeometry g{4, 3, 0.5};
    const auto path = rns::toRosPath(g, {{0, 0}, {1, 0}, {1, 1}}, "map", kStamp);
    ASSERT_EQ(path.poses.size(), 3u);
    EXPECT_NEAR(rns::quaternionToYaw(path.poses[0].pose.orientation), 0.0, 1e-12);  // east
    // (1, 0) -> (1, 1) is down the simulator map = -y (south) in ROS.
    EXPECT_NEAR(rns::quaternionToYaw(path.poses[1].pose.orientation), -kPi / 2, 1e-12);
    EXPECT_DOUBLE_EQ(path.poses[2].pose.position.y, 0.75);
}

TEST(Conversions, CovarianceSigns) {
    // A sample error e in the simulator frame maps to J e in ROS (J = diag(1, -1, -1));
    // the covariance must then be J P J^T: compare quadratic forms.
    const amr::Matrix3 p{0.04, 0.01, 0.002, 0.01, 0.09, -0.003, 0.002, -0.003, 0.01};
    const auto c = rns::toRosCovariance(p);
    const double e[3] = {0.3, -0.2, 0.5};
    const double j[3] = {e[0], -e[1], -e[2]};
    double sim_form = 0.0, ros_form = 0.0;
    const int idx[3] = {0, 1, 5};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            sim_form += e[a] * p[static_cast<std::size_t>(3 * a + b)] * e[b];
            ros_form += j[a] * c[static_cast<std::size_t>(idx[a] * 6 + idx[b])] * j[b];
        }
    }
    EXPECT_NEAR(ros_form, sim_form, 1e-15);
    EXPECT_DOUBLE_EQ(c[0 * 6 + 1], -0.01);   // x-y flips
    EXPECT_DOUBLE_EQ(c[1 * 6 + 5], -0.003);  // y-yaw keeps its sign (two flips)
    EXPECT_DOUBLE_EQ(c[0 * 6 + 5], -0.002);  // x-yaw flips
    EXPECT_DOUBLE_EQ(c[2 * 6 + 2], 0.0);     // z unused
}

/// Every ROS beam points at the same world location as the simulator beam it came
/// from, for full and partial fields of view and any heading.
TEST(Conversions, LaserScanBeamGeometry) {
    const rns::MapGeometry g{40, 30, 0.1};
    for (const auto& [beams, fov_deg] : std::vector<std::pair<int, double>>{{8, 360}, {72, 360}, {5, 90}, {1, 30}}) {
        amr::LidarConfig lidar;
        lidar.beams = beams;
        lidar.fov = fov_deg * kPi / 180.0;
        for (const double heading : {0.0, 0.9, -2.4}) {
            amr::LidarScan scan;
            scan.origin = {2.0, 1.5};
            scan.heading = heading;
            const auto offsets = amr::beamOffsets(lidar);
            for (std::size_t i = 0; i < offsets.size(); ++i) {
                scan.angles.push_back(heading + offsets[i]);
                scan.ranges.push_back(1.0 + 0.01 * static_cast<double>(i));  // distinguishable
                scan.hits.push_back(i % 7 != 3);
            }
            const auto msg = rns::toLaserScan(scan, lidar, "base_link", kStamp);
            ASSERT_EQ(msg.ranges.size(), offsets.size());
            const double robot_yaw = rns::toRosYaw(heading);
            const auto origin = rns::toRosPoint(g, scan.origin);
            std::size_t no_return = 0;
            for (std::size_t k = 0; k < msg.ranges.size(); ++k) {
                if (std::isinf(msg.ranges[k])) {
                    ++no_return;
                    continue;
                }
                // Ranges are unique (1 + 0.01 i): recover the simulator beam i.
                const auto i = static_cast<std::size_t>(std::lround((msg.ranges[k] - 1.0) / 0.01));
                ASSERT_LT(i, offsets.size());
                const double angle =
                    robot_yaw + msg.angle_min + static_cast<double>(k) * msg.angle_increment;
                const double r = msg.ranges[k];
                const auto sim_end = rns::toRosPoint(
                    g, {scan.origin.x + r * std::cos(scan.angles[i]), scan.origin.y + r * std::sin(scan.angles[i])});
                EXPECT_NEAR(origin.x + r * std::cos(angle), sim_end.x, 1e-5)
                    << beams << " beams, " << fov_deg << " deg, heading " << heading << ", ROS beam " << k;
                EXPECT_NEAR(origin.y + r * std::sin(angle), sim_end.y, 1e-5);
            }
            std::size_t misses = 0;
            for (const bool hit : scan.hits) {
                misses += hit ? 0 : 1;
            }
            EXPECT_EQ(no_return, misses);
        }
    }
    // No return is +infinity; the range limit is the LiDAR's.
    amr::LidarConfig lidar;
    lidar.beams = 2;
    lidar.max_range = 4.0;
    amr::LidarScan scan;
    scan.ranges = {4.0, 2.0};
    scan.hits = {false, true};
    scan.angles = {0.0, kPi};
    const auto msg = rns::toLaserScan(scan, lidar, "base_link", kStamp);
    EXPECT_FLOAT_EQ(msg.range_max, 4.0f);
    EXPECT_TRUE(std::isinf(msg.ranges[0]));
}
