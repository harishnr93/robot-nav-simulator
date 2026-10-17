"""Launch the map server, the simulator node and RViz2.

Example:
    ros2 launch robot_nav_sim_ros sim.launch.py
    ros2 launch robot_nav_sim_ros sim.launch.py robot_model:=diff-drive lidar_beams:=36 ekf:=true
    ros2 launch robot_nav_sim_ros sim.launch.py \\
        dynamic_obstacles_file:=$(ros2 pkg prefix robot_nav_sim_ros)/share/robot_nav_sim_ros/maps/obstacles_dynamic.csv
    ros2 launch robot_nav_sim_ros sim.launch.py rviz:=false playback_rate:=5.0

In RViz2, "2D Goal Pose" sends a new goal and "2D Pose Estimate" moves the robot.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = Path(get_package_share_directory("robot_nav_sim_ros"))

# name: (default, type, description)
ARGUMENTS = {
    "map_file": (str(SHARE / "maps" / "obstacles.csv"), str, "map CSV (0 free, 1 occupied)"),
    "resolution": ("0.5", float, "cell size, m"),
    "start_x": ("1", int, "start cell column"),
    "start_y": ("1", int, "start cell row (row 0 = top of the CSV)"),
    "goal_x": ("18", int, "goal cell column"),
    "goal_y": ("18", int, "goal cell row"),
    "robot_model": ("point", str, "point or diff-drive"),
    "robot_radius": ("0.0", float, "footprint radius, m (0 = point robot)"),
    "connectivity": ("4", int, "4 or 8"),
    "speed": ("0.5", float, "m/s"),
    "lidar_beams": ("0", int, "simulated LiDAR beams (0 = off)"),
    "ekf": ("false", bool, "EKF localization (needs diff-drive)"),
    "dynamic_obstacles_file": ("", str, "moving obstacles file (empty = none)"),
    "playback_rate": ("1.0", float, "simulation seconds per wall-clock second"),
}


def generate_launch_description() -> LaunchDescription:
    declared = [DeclareLaunchArgument(name, default_value=default, description=text)
                for name, (default, _, text) in ARGUMENTS.items()]
    declared.append(DeclareLaunchArgument("rviz", default_value="true",
                                          description="start RViz2 with the bundled config"))

    def param(name):
        return ParameterValue(LaunchConfiguration(name), value_type=ARGUMENTS[name][1])

    map_server = Node(
        package="robot_nav_sim_ros", executable="map_server_node", name="map_server",
        parameters=[{"map_file": param("map_file"), "resolution": param("resolution")}],
        output="screen")
    simulator = Node(
        package="robot_nav_sim_ros", executable="simulator_node", name="simulator",
        parameters=[{name: param(name) for name in ARGUMENTS
                     if name not in ("map_file", "resolution")}],
        output="screen")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", str(SHARE / "config" / "sim.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")))
    return LaunchDescription([*declared, map_server, simulator, rviz])
