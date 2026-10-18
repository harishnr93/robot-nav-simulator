// Conversions between the simulator's frame and ROS conventions.
//
// The simulator uses image conventions: grid x = column, grid y = row with row 0 at
// the top, world y pointing DOWN, and headings growing from +x toward +y (clockwise
// on screen). ROS (REP 103) uses a right-handed map frame: y UP, headings growing
// counter-clockwise, and nav_msgs/OccupancyGrid stores row 0 at the BOTTOM (the
// row at the origin). For a map of H rows at resolution s:
//
//     ROS x   = sim x
//     ROS y   = H s - sim y            (so the map spans [0, W s] x [0, H s] in both)
//     ROS yaw = -sim heading
//     OccupancyGrid row r = H - 1 - sim row
//
// The pose covariance transforms with J = diag(1, -1, -1): the x-y and x-yaw terms
// change sign, the y-yaw term does not. LiDAR beams keep their geometry: a scan in
// the robot frame lists ROS angles counter-clockwise, which reverses the
// simulator's beam order.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ekf.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "types.hpp"

namespace robot_nav_sim_ros {

/// Occupancy values written to OccupancyGrid (0..100; -1 = unknown).
constexpr int8_t kFreeValue = 0;
constexpr int8_t kOccupiedValue = 100;
/// Cells at or above this value are obstacles when reading an OccupancyGrid.
/// Unknown cells (-1) are treated as obstacles too: the planner never plans
/// through space nobody has seen.
constexpr int kOccupiedThreshold = 50;

/// Size and resolution of the map the conversions refer to.
struct MapGeometry {
    int width = 0;
    int height = 0;
    double resolution = 0.0;
};
MapGeometry geometryOf(const amr::GridMap& map);

geometry_msgs::msg::Point toRosPoint(const MapGeometry& g, amr::WorldPoint sim);
amr::WorldPoint toSimPoint(const MapGeometry& g, double ros_x, double ros_y);
double toRosYaw(double sim_heading);
double toSimHeading(double ros_yaw);

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);

/// Pose of a simulator position + heading in the ROS map frame.
geometry_msgs::msg::Pose toRosPose(const MapGeometry& g, amr::WorldPoint sim, double sim_heading);

/// Grid cell containing a ROS map-frame position, if it is on the map.
std::optional<amr::GridCoord> toSimCell(const MapGeometry& g, double ros_x, double ros_y);

nav_msgs::msg::OccupancyGrid toOccupancyGrid(const amr::GridMap& map, const std::string& frame_id,
                                             const builtin_interfaces::msg::Time& stamp);

/// Map from an OccupancyGrid (origin assumed at (0, 0) with no rotation; the map
/// server publishes it that way). Values >= kOccupiedThreshold and unknown (-1)
/// cells become occupied.
amr::GridMap fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid);

/// A cell path as nav_msgs/Path: cell centers, each pose facing the next cell.
nav_msgs::msg::Path toRosPath(const MapGeometry& g, const std::vector<amr::GridCoord>& cells,
                              const std::string& frame_id,
                              const builtin_interfaces::msg::Time& stamp);

/// A LiDAR scan in the robot frame. Beams with no return get +infinity (REP 117).
sensor_msgs::msg::LaserScan toLaserScan(const amr::LidarScan& scan, const amr::LidarConfig& lidar,
                                        const std::string& frame_id,
                                        const builtin_interfaces::msg::Time& stamp);

/// 6 x 6 pose covariance (x, y, z, roll, pitch, yaw) in the ROS frame from the
/// EKF's 3 x 3 covariance of (x, y, heading) in the simulator frame.
std::array<double, 36> toRosCovariance(const amr::Matrix3& sim);

}  // namespace robot_nav_sim_ros
