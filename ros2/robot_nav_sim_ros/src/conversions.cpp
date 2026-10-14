#include "robot_nav_sim_ros/conversions.hpp"

#include <cmath>
#include <limits>

#include "diff_drive.hpp"

namespace robot_nav_sim_ros {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

MapGeometry geometryOf(const amr::GridMap& map) {
    return {map.width(), map.height(), map.resolution()};
}

geometry_msgs::msg::Point toRosPoint(const MapGeometry& g, amr::WorldPoint sim) {
    geometry_msgs::msg::Point p;
    p.x = sim.x;
    p.y = g.height * g.resolution - sim.y;
    p.z = 0.0;
    return p;
}

amr::WorldPoint toSimPoint(const MapGeometry& g, double ros_x, double ros_y) {
    return {ros_x, g.height * g.resolution - ros_y};
}

double toRosYaw(double sim_heading) {
    return amr::wrapAngle(-sim_heading);
}

double toSimHeading(double ros_yaw) {
    return amr::wrapAngle(-ros_yaw);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
    // Yaw of a general quaternion (ZYX convention).
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Pose toRosPose(const MapGeometry& g, amr::WorldPoint sim, double sim_heading) {
    geometry_msgs::msg::Pose pose;
    pose.position = toRosPoint(g, sim);
    pose.orientation = yawToQuaternion(toRosYaw(sim_heading));
    return pose;
}

std::optional<amr::GridCoord> toSimCell(const MapGeometry& g, double ros_x, double ros_y) {
    const amr::WorldPoint p = toSimPoint(g, ros_x, ros_y);
    const int x = static_cast<int>(std::floor(p.x / g.resolution));
    const int y = static_cast<int>(std::floor(p.y / g.resolution));
    if (x < 0 || y < 0 || x >= g.width || y >= g.height) {
        return std::nullopt;
    }
    return amr::GridCoord{x, y};
}

nav_msgs::msg::OccupancyGrid toOccupancyGrid(const amr::GridMap& map, const std::string& frame_id,
                                             const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id;
    grid.header.stamp = stamp;
    grid.info.map_load_time = stamp;
    grid.info.resolution = static_cast<float>(map.resolution());
    grid.info.width = static_cast<uint32_t>(map.width());
    grid.info.height = static_cast<uint32_t>(map.height());
    grid.info.origin.orientation.w = 1.0;  // origin (0, 0): the map's lower-left corner
    grid.data.resize(map.cellCount());
    for (int y = 0; y < map.height(); ++y) {
        const int row = map.height() - 1 - y;  // ROS row 0 is the bottom
        for (int x = 0; x < map.width(); ++x) {
            grid.data[static_cast<std::size_t>(row * map.width() + x)] =
                map.isOccupied({x, y}) ? kOccupiedValue : kFreeValue;
        }
    }
    return grid;
}

amr::GridMap fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    amr::GridMap map(width, height, grid.info.resolution);
    for (int row = 0; row < height; ++row) {
        for (int x = 0; x < width; ++x) {
            const int8_t v = grid.data[static_cast<std::size_t>(row * width + x)];
            if (v < 0 || v >= kOccupiedThreshold) {
                map.setOccupied({x, height - 1 - row}, true);
            }
        }
    }
    return map;
}

nav_msgs::msg::Path toRosPath(const MapGeometry& g, const std::vector<amr::GridCoord>& cells,
                              const std::string& frame_id,
                              const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;
    const auto center = [&](const amr::GridCoord& c) {
        return amr::WorldPoint{(c.x + 0.5) * g.resolution, (c.y + 0.5) * g.resolution};
    };
    double heading = 0.0;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const amr::WorldPoint p = center(cells[i]);
        if (i + 1 < cells.size()) {
            const amr::WorldPoint q = center(cells[i + 1]);
            heading = std::atan2(q.y - p.y, q.x - p.x);
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose = toRosPose(g, p, heading);
        path.poses.push_back(pose);
    }
    return path;
}

sensor_msgs::msg::LaserScan toLaserScan(const amr::LidarScan& scan, const amr::LidarConfig& lidar,
                                        const std::string& frame_id,
                                        const builtin_interfaces::msg::Time& stamp) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    const std::size_t n = scan.ranges.size();
    msg.range_min = 0.0f;
    msg.range_max = static_cast<float>(lidar.max_range);
    const bool full_circle = lidar.fov >= 2.0 * kPi - 1e-12;
    // Simulator beam i points at heading + offset_i (clockwise positive); the same
    // direction has ROS angle -offset_i in the robot frame. Listing ROS angles
    // counter-clockwise from angle_min reverses the order.
    if (n == 1) {
        msg.angle_min = 0.0f;
        msg.angle_max = 0.0f;
        msg.angle_increment = 0.0f;
    } else if (full_circle) {
        // Offsets -pi + 2 pi i / n  ->  ROS angles -pi + 2 pi k / n with i = (n - k) mod n.
        msg.angle_min = static_cast<float>(-kPi);
        msg.angle_increment = static_cast<float>(2.0 * kPi / static_cast<double>(n));
        msg.angle_max = static_cast<float>(-kPi + 2.0 * kPi * static_cast<double>(n - 1) /
                                                      static_cast<double>(n));
    } else {
        // Offsets -fov/2 .. +fov/2  ->  ROS angles in the reverse order: i = n - 1 - k.
        msg.angle_min = static_cast<float>(-lidar.fov / 2.0);
        msg.angle_max = static_cast<float>(lidar.fov / 2.0);
        msg.angle_increment = static_cast<float>(lidar.fov / static_cast<double>(n - 1));
    }
    msg.ranges.resize(n);
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = n == 1 ? 0 : full_circle ? (n - k) % n : n - 1 - k;
        msg.ranges[k] = scan.hits[i] ? static_cast<float>(scan.ranges[i])
                                     : std::numeric_limits<float>::infinity();
    }
    return msg;
}

std::array<double, 36> toRosCovariance(const amr::Matrix3& sim) {
    // sim = [xx xy xt; yx yy yt; tx ty tt]; J = diag(1, -1, -1).
    std::array<double, 36> c{};
    const int idx[3] = {0, 1, 5};  // x, y, yaw in the 6 x 6 layout
    const double sign[3] = {1.0, -1.0, -1.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            c[static_cast<std::size_t>(idx[i] * 6 + idx[j])] = sign[i] * sign[j] * sim[static_cast<std::size_t>(3 * i + j)];
        }
    }
    return c;
}

}  // namespace robot_nav_sim_ros
