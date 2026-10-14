// simulator_node: runs the navigation simulator on the map from /map and plays the
// result back in (scaled) real time for RViz2.
//
// Subscribes:
//   /map            nav_msgs/OccupancyGrid (latched)    the map to plan and drive on
//   /goal_pose      geometry_msgs/PoseStamped           new goal (RViz2 "2D Goal Pose")
//   /initialpose    geometry_msgs/PoseWithCovarianceStamped
//                                                       move the robot (RViz2 "2D Pose Estimate")
// Publishes (map frame unless noted):
//   /inflated_map         nav_msgs/OccupancyGrid (latched)  the map the planner uses
//   /plan                 nav_msgs/Path (latched)           the path being followed
//   /trajectory           nav_msgs/Path                     where the robot has been
//   /odom                 nav_msgs/Odometry                 true pose and velocity
//   /tf                   map -> base_link
//   /scan                 sensor_msgs/LaserScan (base_link) LiDAR (lidar_beams > 0)
//   /ekf/pose             geometry_msgs/PoseWithCovarianceStamped   EKF estimate (ekf)
//   /ekf/dead_reckoning   geometry_msgs/PoseStamped         odometry only (ekf)
//   /dynamic_obstacles    visualization_msgs/MarkerArray    moving obstacles
//   /navigation_status    std_msgs/String (latched)         PLANNING, NAVIGATING,
//                                                           GOAL_REACHED, NO_PATH ..., etc.
//
// Each mission is the standalone simulator's pipeline (mission.hpp). A new goal or
// initial pose starts a new mission from the robot's current cell. Coordinates are
// converted between the simulator's frame (y down) and ROS (y up) in conversions.hpp.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dynamic_obstacles.hpp"
#include "robot_nav_sim_ros/conversions.hpp"
#include "robot_nav_sim_ros/mission.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
using robot_nav_sim_ros::Mission;
using robot_nav_sim_ros::MissionOptions;

class SimulatorNode : public rclcpp::Node {
public:
    SimulatorNode()
        : rclcpp::Node("simulator"),
          start_{static_cast<int>(declare_parameter<int>("start_x", 1)),
                 static_cast<int>(declare_parameter<int>("start_y", 1))},
          goal_{static_cast<int>(declare_parameter<int>("goal_x", 18)),
                static_cast<int>(declare_parameter<int>("goal_y", 18))} {
        MissionOptions& o = options_;
        const std::string model = declare_parameter<std::string>("robot_model", "point");
        if (model != "point" && model != "diff-drive") {
            throw std::runtime_error("robot_model must be point or diff-drive");
        }
        o.motion.model = model == "point" ? amr::RobotModel::Point : amr::RobotModel::DiffDrive;
        o.motion.speed = declare_parameter<double>("speed", o.motion.speed);
        o.motion.dt = declare_parameter<double>("dt", o.motion.dt);
        o.motion.goal_tolerance = declare_parameter<double>("goal_tolerance", o.motion.goal_tolerance);
        o.motion.max_time = declare_parameter<double>("max_time", o.motion.max_time);
        o.motion.robot_radius = declare_parameter<double>("robot_radius", 0.0);
        o.motion.max_angular_speed =
            declare_parameter<double>("max_angular_speed", o.motion.max_angular_speed);
        o.motion.heading_gain = declare_parameter<double>("heading_gain", o.motion.heading_gain);
        o.motion.initial_heading =
            declare_parameter<double>("initial_heading_deg", 0.0) * kPi / 180.0;
        const int connectivity = static_cast<int>(declare_parameter<int>("connectivity", 4));
        if (connectivity != 4 && connectivity != 8) {
            throw std::runtime_error("connectivity must be 4 or 8");
        }
        o.connectivity = static_cast<amr::Connectivity>(connectivity);
        o.lidar.beams = static_cast<int>(declare_parameter<int>("lidar_beams", 0));
        o.lidar.fov = declare_parameter<double>("lidar_fov_deg", 360.0) * kPi / 180.0;
        o.lidar.max_range = declare_parameter<double>("lidar_range", o.lidar.max_range);
        o.lidar.noise_stddev = declare_parameter<double>("lidar_noise", 0.0);
        o.lidar.seed = static_cast<std::uint32_t>(declare_parameter<int>("lidar_seed", 0));
        o.ekf.enabled = declare_parameter<bool>("ekf", false);
        o.ekf.odometry.distance = declare_parameter<double>("odom_noise_dist", o.ekf.odometry.distance);
        o.ekf.odometry.turn = declare_parameter<double>("odom_noise_turn", o.ekf.odometry.turn);
        o.ekf.odometry.drift = declare_parameter<double>("odom_noise_drift", o.ekf.odometry.drift);
        o.ekf.odometry.seed = static_cast<std::uint32_t>(declare_parameter<int>("odom_seed", 0));
        o.ekf.range_sigma = declare_parameter<double>("ekf_range_sigma", o.ekf.range_sigma);
        obstacles_file_ = declare_parameter<std::string>("dynamic_obstacles_file", "");
        o.replanning.sensing_range = declare_parameter<double>("sensing_range", o.replanning.sensing_range);
        o.replanning.patience = declare_parameter<double>("replan_patience", o.replanning.patience);
        playback_rate_ = declare_parameter<double>("playback_rate", 1.0);
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
        if (const std::string e = robot_nav_sim_ros::validateMissionOptions(o); !e.empty()) {
            throw std::runtime_error(e);
        }
        if (!(playback_rate_ > 0.0)) {
            throw std::runtime_error("playback_rate must be positive");
        }
        heading_ = o.motion.initial_heading;

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        inflated_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("inflated_map", latched);
        plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", latched);
        status_pub_ = create_publisher<std_msgs::msg::String>("navigation_status", latched);
        trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
        ekf_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf/pose", 10);
        dead_reckoning_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf/dead_reckoning", 10);
        obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("dynamic_obstacles", 10);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", latched, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { onMap(*m); });
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { onGoal(*m); });
        initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { onInitialPose(*m); });

        const auto period = std::chrono::duration<double>(o.motion.dt / playback_rate_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this] { tick(); });
        setStatus("WAITING_FOR_MAP");
    }

private:
    void onMap(const nav_msgs::msg::OccupancyGrid& msg) {
        map_ = robot_nav_sim_ros::fromOccupancyGrid(msg);
        geometry_ = robot_nav_sim_ros::geometryOf(*map_);
        options_.obstacles.clear();
        if (!obstacles_file_.empty()) {
            const amr::DynamicObstacleLoadResult loaded = amr::loadDynamicObstacles(obstacles_file_, *map_);
            if (!loaded.ok) {
                RCLCPP_ERROR(get_logger(), "%s", loaded.error.c_str());
            } else {
                options_.obstacles = loaded.obstacles;
            }
        }
        RCLCPP_INFO(get_logger(), "Map received: %d x %d cells at %.3f m", map_->width(),
                    map_->height(), map_->resolution());
        // The first map starts the configured mission; a new map restarts from here.
        if (!position_) {
            position_ = map_->gridToWorld(start_);
            startMission(start_, goal_);
        } else {
            startMission(currentCell(), goal_);
        }
    }

    void onGoal(const geometry_msgs::msg::PoseStamped& msg) {
        if (!map_ || !acceptFrame(msg.header.frame_id)) {
            return;
        }
        const auto cell = robot_nav_sim_ros::toSimCell(geometry_, msg.pose.position.x, msg.pose.position.y);
        if (!cell) {
            RCLCPP_WARN(get_logger(), "Goal (%.2f, %.2f) is outside the map", msg.pose.position.x,
                        msg.pose.position.y);
            return;
        }
        goal_ = *cell;
        RCLCPP_INFO(get_logger(), "New goal: cell (%d, %d)", goal_.x, goal_.y);
        startMission(currentCell(), goal_);
    }

    void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
        if (!map_ || !acceptFrame(msg.header.frame_id)) {
            return;
        }
        const auto& p = msg.pose.pose;
        const auto cell = robot_nav_sim_ros::toSimCell(geometry_, p.position.x, p.position.y);
        if (!cell) {
            RCLCPP_WARN(get_logger(), "Initial pose is outside the map");
            return;
        }
        heading_ = robot_nav_sim_ros::toSimHeading(robot_nav_sim_ros::quaternionToYaw(p.orientation));
        position_ = map_->gridToWorld(*cell);
        trajectory_.poses.clear();
        RCLCPP_INFO(get_logger(), "Robot moved to cell (%d, %d)", cell->x, cell->y);
        startMission(*cell, goal_);
    }

    bool acceptFrame(const std::string& frame) {
        if (!frame.empty() && frame != map_frame_) {
            RCLCPP_WARN(get_logger(), "Ignoring a pose in frame '%s' (expected '%s')", frame.c_str(),
                        map_frame_.c_str());
            return false;
        }
        return true;
    }

    amr::GridCoord currentCell() const { return map_->worldToGrid(*position_); }

    void startMission(amr::GridCoord start, amr::GridCoord goal) {
        setStatus("PLANNING");
        MissionOptions o = options_;
        o.motion.initial_heading = heading_;
        mission_ = robot_nav_sim_ros::runMission(*map_, start, goal, o);
        sample_ = 0;
        published_path_ = nullptr;
        inflated_pub_->publish(robot_nav_sim_ros::toOccupancyGrid(mission_->planning_map, map_frame_, now()));
        if (!mission_->planned()) {
            plan_pub_->publish(robot_nav_sim_ros::toRosPath(geometry_, {}, map_frame_, now()));
            setStatus(std::string("NO_PATH: ") + amr::toString(mission_->plan.status));
            RCLCPP_WARN(get_logger(), "No path from (%d, %d) to (%d, %d): %s", start.x, start.y,
                        goal.x, goal.y, amr::toString(mission_->plan.status));
            return;
        }
        RCLCPP_INFO(get_logger(), "Planned %zu waypoints; simulated %.1f s (%s)",
                    mission_->plan.path.size(), mission_->sim.travelTime(),
                    amr::toString(mission_->sim.status));
        setStatus("NAVIGATING");
    }

    void tick() {
        if (!map_) {
            return;
        }
        const rclcpp::Time stamp = now();
        if (!mission_ || mission_->sim.trajectory.empty()) {
            publishPose(stamp, 0.0, 0.0);  // the robot stands where it is
            return;
        }
        const auto& trajectory = mission_->sim.trajectory;
        const std::size_t k = std::min(sample_, trajectory.size() - 1);
        const amr::TrajectorySample& s = trajectory[k];
        const amr::WorldPoint previous = *position_;
        position_ = amr::WorldPoint{s.state.x, s.state.y};
        if (options_.motion.model == amr::RobotModel::DiffDrive) {
            heading_ = s.state.theta;
        }
        // Velocity: the diff drive records it; for the point robot, from the step.
        double v = s.linear_velocity;
        double omega = s.angular_velocity;
        if (options_.motion.model == amr::RobotModel::Point) {
            v = std::hypot(position_->x - previous.x, position_->y - previous.y) / options_.motion.dt;
            omega = 0.0;
        }
        publishPose(stamp, k + 1 < trajectory.size() ? v : 0.0, k + 1 < trajectory.size() ? omega : 0.0);
        if (sample_ < trajectory.size()) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = map_frame_;
            pose.header.stamp = stamp;
            pose.pose = robot_nav_sim_ros::toRosPose(geometry_, *position_, heading_);
            trajectory_.poses.push_back(pose);
        }
        trajectory_.header.frame_id = map_frame_;
        trajectory_.header.stamp = stamp;
        trajectory_pub_->publish(trajectory_);
        const double t = s.state.time;
        const std::vector<amr::GridCoord>& path = mission_->pathAt(t);
        if (&path != published_path_) {
            plan_pub_->publish(robot_nav_sim_ros::toRosPath(geometry_, path, map_frame_, stamp));
            published_path_ = &path;
        }
        if (k < mission_->scans.size()) {
            scan_pub_->publish(robot_nav_sim_ros::toLaserScan(mission_->scans[k], options_.lidar,
                                                              robot_frame_, stamp));
        }
        if (k < mission_->localization.size()) {
            publishLocalization(mission_->localization[k], stamp);
        }
        publishObstacles(t, stamp);
        if (sample_ + 1 < trajectory.size()) {
            ++sample_;
        } else if (sample_ + 1 == trajectory.size()) {
            ++sample_;  // past the end: report once
            setStatus(amr::toString(mission_->sim.status));
            RCLCPP_INFO(get_logger(), "Navigation finished: %s", amr::toString(mission_->sim.status));
        }
    }

    void publishPose(const rclcpp::Time& stamp, double v, double omega) {
        if (!position_) {
            return;
        }
        const geometry_msgs::msg::Pose pose = robot_nav_sim_ros::toRosPose(geometry_, *position_, heading_);
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = map_frame_;
        odom.header.stamp = stamp;
        odom.child_frame_id = robot_frame_;
        odom.pose.pose = pose;
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = -omega;  // clockwise in the simulator, counter-clockwise in ROS
        odom_pub_->publish(odom);
        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = robot_frame_;
        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.rotation = pose.orientation;
        tf_->sendTransform(tf);
    }

    void publishLocalization(const amr::LocalizationSample& s, const rclcpp::Time& stamp) {
        geometry_msgs::msg::PoseWithCovarianceStamped estimate;
        estimate.header.frame_id = map_frame_;
        estimate.header.stamp = stamp;
        estimate.pose.pose = robot_nav_sim_ros::toRosPose(geometry_, {s.estimate.x, s.estimate.y},
                                                          s.estimate.theta);
        estimate.pose.covariance = robot_nav_sim_ros::toRosCovariance(s.covariance);
        ekf_pub_->publish(estimate);
        geometry_msgs::msg::PoseStamped dead;
        dead.header = estimate.header;
        dead.pose = robot_nav_sim_ros::toRosPose(geometry_, {s.odometry.x, s.odometry.y}, s.odometry.theta);
        dead_reckoning_pub_->publish(dead);
    }

    void publishObstacles(double t, const rclcpp::Time& stamp) {
        if (options_.obstacles.empty()) {
            return;
        }
        visualization_msgs::msg::MarkerArray markers;
        for (std::size_t i = 0; i < options_.obstacles.size(); ++i) {
            const amr::DynamicObstacle& o = options_.obstacles[i];
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp = stamp;
            m.ns = "dynamic_obstacles";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = robot_nav_sim_ros::toRosPoint(geometry_, amr::obstaclePosition(o, t));
            m.pose.position.z = 0.15;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = 2.0 * o.radius;
            m.scale.z = 0.3;
            m.color.r = 0.61f;
            m.color.g = 0.40f;
            m.color.b = 0.27f;
            m.color.a = 0.9f;
            markers.markers.push_back(m);
        }
        obstacles_pub_->publish(markers);
    }

    void setStatus(const std::string& status) {
        if (status == status_) {
            return;
        }
        status_ = status;
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }

    MissionOptions options_;
    amr::GridCoord start_;
    amr::GridCoord goal_;
    std::string obstacles_file_;
    double playback_rate_ = 1.0;
    std::string map_frame_;
    std::string robot_frame_;

    std::optional<amr::GridMap> map_;
    robot_nav_sim_ros::MapGeometry geometry_;
    std::optional<Mission> mission_;
    std::size_t sample_ = 0;
    std::optional<amr::WorldPoint> position_;
    double heading_ = 0.0;
    nav_msgs::msg::Path trajectory_;
    const std::vector<amr::GridCoord>* published_path_ = nullptr;
    std::string status_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dead_reckoning_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(std::make_shared<SimulatorNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("simulator"), "%s", e.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
