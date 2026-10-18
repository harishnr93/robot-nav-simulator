"""End-to-end: the map server and the simulator node, checked through their topics.

Runs the obstacles map with the differential drive, a 36-beam LiDAR, the EKF and the
sample moving obstacles at 20x speed, and checks what RViz2 would show: the map, the
plan, the robot reaching the goal (in ROS coordinates: y up), the scan, the EKF
estimate, the obstacle markers and TF. Then sends a new goal on /goal_pose, as RViz2's
"2D Goal Pose" tool does, and checks that the robot drives there.
"""

import math
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

MAPS = Path(get_package_share_directory("robot_nav_sim_ros")) / "maps"
RESOLUTION = 0.5
HEIGHT = 20  # obstacles.csv is 20 x 20 cells


def ros_center(cell_x, cell_y):
    """ROS map-frame position of a simulator cell center (y up)."""
    return (cell_x + 0.5) * RESOLUTION, HEIGHT * RESOLUTION - (cell_y + 0.5) * RESOLUTION


@pytest.mark.launch_test
def generate_test_description():
    map_server = launch_ros.actions.Node(
        package="robot_nav_sim_ros", executable="map_server_node",
        parameters=[{"map_file": str(MAPS / "obstacles.csv"), "resolution": RESOLUTION}])
    simulator = launch_ros.actions.Node(
        package="robot_nav_sim_ros", executable="simulator_node",
        parameters=[{"start_x": 1, "start_y": 1, "goal_x": 18, "goal_y": 18,
                     "robot_model": "diff-drive", "lidar_beams": 36, "ekf": True,
                     "dynamic_obstacles_file": str(MAPS / "obstacles_dynamic.csv"),
                     "playback_rate": 20.0}])
    return launch.LaunchDescription([map_server, simulator, launch_testing.actions.ReadyToTest()])


class TestSimulatorNodes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_checker")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.got = {"map": None, "plan": None, "status": [], "odom": None, "scan": None,
                   "ekf": None, "markers": None, "tf": None, "trajectory": None}

        def keep(key):
            def callback(msg):
                cls.got[key] = msg
            return callback

        n = cls.node
        n.create_subscription(OccupancyGrid, "map", keep("map"), latched)
        n.create_subscription(PathMsg, "plan", keep("plan"), latched)
        n.create_subscription(String, "navigation_status",
                              lambda m: cls.got["status"].append(m.data), latched)
        n.create_subscription(Odometry, "odom", keep("odom"), 10)
        n.create_subscription(PathMsg, "trajectory", keep("trajectory"), 10)
        n.create_subscription(LaserScan, "scan", keep("scan"), qos_profile_sensor_data)
        n.create_subscription(PoseWithCovarianceStamped, "ekf/pose", keep("ekf"), 10)
        n.create_subscription(MarkerArray, "dynamic_obstacles", keep("markers"), 10)
        n.create_subscription(TFMessage, "tf", keep("tf"), 10)
        cls.goal_pub = n.create_publisher(PoseStamped, "goal_pose", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, condition, timeout=60.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition():
                return True
        return False

    def test_1_map(self):
        self.assertTrue(self.spin_until(lambda: self.got["map"] is not None))
        m = self.got["map"]
        self.assertEqual((m.info.width, m.info.height), (20, HEIGHT))
        self.assertAlmostEqual(m.info.resolution, RESOLUTION)
        self.assertEqual(m.header.frame_id, "map")
        # Same number of obstacle cells as the CSV; the CSV's top row is the last ROS row.
        rows = [[int(v) for v in line.split(",")]
                for line in (MAPS / "obstacles.csv").read_text().splitlines()
                if line.strip() and not line.startswith("#")]
        self.assertEqual(sum(v == 100 for v in m.data), sum(map(sum, rows)))
        for x in range(20):
            self.assertEqual(m.data[(HEIGHT - 1) * 20 + x] == 100, rows[0][x] == 1)

    def test_2_plan_and_goal(self):
        self.assertTrue(self.spin_until(lambda: "GOAL_REACHED" in self.got["status"]),
                        f"statuses: {self.got['status']}")
        plan = self.got["plan"]
        self.assertGreater(len(plan.poses), 2)
        last = plan.poses[-1].pose.position
        self.assertAlmostEqual((last.x, last.y), ros_center(18, 18))
        self.spin_until(lambda: False, 0.5)  # drain odometry queued behind the status
        pose = self.got["odom"].pose.pose.position
        gx, gy = ros_center(18, 18)
        self.assertLess(math.hypot(pose.x - gx, pose.y - gy), 0.05)
        self.assertEqual(self.got["odom"].child_frame_id, "base_link")
        self.assertGreater(len(self.got["trajectory"].poses), 50)

    def test_3_sensors_and_tf(self):
        self.assertTrue(self.spin_until(lambda: all(self.got[k] is not None
                                                    for k in ("scan", "ekf", "markers", "tf"))))
        scan = self.got["scan"]
        self.assertEqual(len(scan.ranges), 36)
        self.assertEqual(scan.header.frame_id, "base_link")
        self.assertAlmostEqual(scan.angle_increment, 2 * math.pi / 36, places=5)
        ekf = self.got["ekf"].pose
        self.assertGreater(ekf.covariance[0], 0.0)    # var x
        self.assertGreater(ekf.covariance[35], 0.0)   # var yaw
        self.assertEqual(len(self.got["markers"].markers), 3)
        frames = {(t.header.frame_id, t.child_frame_id) for t in self.got["tf"].transforms}
        self.assertIn(("map", "base_link"), frames)

    def test_4_new_goal_from_rviz(self):
        # RViz2's "2D Goal Pose": a PoseStamped in the map frame. Cell (1, 18) is
        # bottom-left in the CSV (row 18), i.e. near y = 0.75 m in ROS.
        gx, gy = ros_center(1, 18)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.w = 1.0
        self.assertTrue(self.spin_until(lambda: self.goal_pub.get_subscription_count() > 0, 20.0))
        self.got["status"].clear()
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: "NAVIGATING" in self.got["status"], 20.0),
                        f"statuses: {self.got['status']}")
        self.assertTrue(self.spin_until(lambda: self.got["status"][-1:] == ["GOAL_REACHED"], 90.0),
                        f"statuses: {self.got['status']}")
        last = self.got["plan"].poses[-1].pose.position
        self.assertAlmostEqual((last.x, last.y), (gx, gy))
        self.spin_until(lambda: False, 0.5)  # let the final odometry arrive
        pose = self.got["odom"].pose.pose.position
        self.assertLess(math.hypot(pose.x - gx, pose.y - gy), 0.05)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
