// map_server_node: loads a map CSV (the simulator's format) and publishes it as a
// latched nav_msgs/OccupancyGrid on /map.
//
// Parameters:
//   map_file    (string, required)  map CSV: 0 = free, 1 = occupied, row 0 = top
//   resolution  (double, 0.5)       cell size, m
//   frame_id    (string, "map")
#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "grid_map.hpp"
#include "robot_nav_sim_ros/conversions.hpp"

namespace {

class MapServer : public rclcpp::Node {
public:
    MapServer() : rclcpp::Node("map_server") {
        const std::string file = declare_parameter<std::string>("map_file", "");
        const double resolution = declare_parameter<double>("resolution", 0.5);
        const std::string frame = declare_parameter<std::string>("frame_id", "map");
        if (file.empty()) {
            throw std::runtime_error("parameter map_file is required");
        }
        const amr::MapLoadResult loaded = amr::loadGridMapCsv(file, resolution);
        if (!loaded.ok) {
            throw std::runtime_error(loaded.error);
        }
        // Transient local: late subscribers (RViz2, the simulator) still get the map.
        publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).reliable().transient_local());
        publisher_->publish(robot_nav_sim_ros::toOccupancyGrid(loaded.map, frame, now()));
        RCLCPP_INFO(get_logger(), "Published %s: %d x %d cells at %.3f m", file.c_str(),
                    loaded.map.width(), loaded.map.height(), resolution);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(std::make_shared<MapServer>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("map_server"), "%s", e.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
