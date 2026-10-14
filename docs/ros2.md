# ROS 2 integration

`ros2/robot_nav_sim_ros` wraps the simulator as two ROS 2 nodes. They publish the map, the
plan and the robot's state for RViz2. The standalone `robot_simulator` is unchanged: both
are built from the same sources (`cmake/amr_core_sources.cmake`), and a test checks that
a ROS mission matches the CLI's plan and trajectory exactly.

Tested with ROS 2 Humble on Ubuntu 22.04. The package uses only core ROS packages
(`rclcpp`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`),
so it also builds on later distributions.

## Build, test and launch

```bash
source /opt/ros/humble/setup.bash
scripts/build_ros2.sh --test          # colcon build + test into build_ros2/, install_ros2/
source install_ros2/setup.bash

ros2 launch robot_nav_sim_ros sim.launch.py                          # point robot, RViz2
ros2 launch robot_nav_sim_ros sim.launch.py robot_model:=diff-drive lidar_beams:=36 ekf:=true
ros2 launch robot_nav_sim_ros sim.launch.py robot_model:=diff-drive \
    dynamic_obstacles_file:=$(ros2 pkg prefix robot_nav_sim_ros)/share/robot_nav_sim_ros/maps/obstacles_dynamic.csv
ros2 launch robot_nav_sim_ros sim.launch.py rviz:=false playback_rate:=5.0   # headless
```

colcon builds from the repository root with separate build, install and log
directories. The standalone `build/` and the ROS build never interfere.

In RViz2:

- **2D Goal Pose** sends a new goal on `/goal_pose`. The robot replans from its
  current cell.
- **2D Pose Estimate** moves the robot on `/initialpose`.

## Nodes

### `map_server_node`

This node loads a map CSV and publishes it once, latched, on `/map`.

| Parameter | Default | Meaning |
|---|---|---|
| `map_file` | (required) | map CSV (0 free, 1 occupied) |
| `resolution` | 0.5 | cell size, m |
| `frame_id` | `map` | frame of the grid |

### `simulator_node`

This node waits for `/map`, then runs a *mission*: the same pipeline as
`robot_simulator`. The pipeline is: inflate, plan with the footprint, simulate (with
moving obstacles and replanning), LiDAR scans, then EKF. The node then plays the result
back on a wall timer at `dt / playback_rate`.

A new goal, initial pose or map starts a new mission from the robot's current cell.

| Parameter | Default | Matches the CLI's |
|---|---|---|
| `start_x`, `start_y`, `goal_x`, `goal_y` | 1, 1, 18, 18 | `--start`, `--goal` (cells, row 0 = top of the CSV) |
| `robot_model` | `point` | `--robot-model` (`point` or `diff-drive`) |
| `speed`, `dt`, `goal_tolerance`, `max_time` | CLI defaults | same names |
| `robot_radius`, `connectivity` | 0.0, 4 | `--robot-radius`, `--connectivity` |
| `max_angular_speed`, `heading_gain`, `initial_heading_deg` | CLI defaults | differential drive |
| `lidar_beams`, `lidar_fov_deg`, `lidar_range`, `lidar_noise`, `lidar_seed` | 0 (off), 360, … | `--lidar-*` |
| `ekf`, `odom_noise_dist`, `odom_noise_turn`, `odom_noise_drift`, `odom_seed`, `ekf_range_sigma` | off | `--ekf`, odometry noise |
| `dynamic_obstacles_file`, `sensing_range`, `replan_patience` | none | `--dynamic-obstacles`, replanning |
| `playback_rate` | 1.0 | simulation seconds per wall-clock second |
| `map_frame`, `robot_frame` | `map`, `base_link` | |

Invalid options are rejected at startup with the CLI's own messages, for example
"the EKF needs the differential-drive model".

## Topics

| Topic | Type | QoS | Content |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | latched | the map (0 free, 100 occupied) |
| `/inflated_map` | `nav_msgs/OccupancyGrid` | latched | the map the planner uses (footprint inflation) |
| `/plan` | `nav_msgs/Path` | latched | the path being followed; updated on every successful replan |
| `/trajectory` | `nav_msgs/Path` | reliable | where the robot has been |
| `/odom` | `nav_msgs/Odometry` | reliable | true pose and velocity (`map` → `base_link`) |
| `/tf` | `map` → `base_link` | | the true pose |
| `/scan` | `sensor_msgs/LaserScan` | sensor data | LiDAR in `base_link` (`lidar_beams > 0`) |
| `/ekf/pose` | `geometry_msgs/PoseWithCovarianceStamped` | reliable | EKF estimate and covariance (`ekf`) |
| `/ekf/dead_reckoning` | `geometry_msgs/PoseStamped` | reliable | odometry-only estimate (`ekf`) |
| `/dynamic_obstacles` | `visualization_msgs/MarkerArray` | reliable | moving obstacles as cylinders |
| `/navigation_status` | `std_msgs/String` | latched | `WAITING_FOR_MAP`, `PLANNING`, `NAVIGATING`, `NO_PATH: <reason>`, then the final status (`GOAL_REACHED`, `COLLISION`, `TIMEOUT`, …) |

Subscribed: `/map`, `/goal_pose` (`geometry_msgs/PoseStamped`), `/initialpose`
(`geometry_msgs/PoseWithCovarianceStamped`).

`config/sim.rviz` shows all of these. It uses the `map` fixed frame and a top-down
orthographic view, with the inflated map drawn as a semi-transparent costmap.

## Frames

The simulator's grid has row 0 at the top, `+y` down and clockwise headings. ROS
(REP 103) uses `+y` up and counter-clockwise yaw, and an `OccupancyGrid` has row 0 at the
bottom. `conversions.hpp` holds every conversion. Each one has a geometric unit test.

| Quantity | Simulator → ROS |
|---|---|
| position | `x_ros = x`, `y_ros = height · resolution − y` |
| heading | `yaw = −θ` |
| grid row | `row_ros = height − 1 − row` (origin at `(0, 0)`) |
| covariance (x, y, θ) | `J P Jᵀ`, `J = diag(1, −1, −1)`: x-y and x-θ flip sign, y-θ keeps it |
| angular velocity | `ω_ros = −ω` |
| LaserScan | full circle: ROS beam `k` = simulator beam `(n − k) mod n`, `angle_min = −π`; partial field of view: beam `k` = `n − 1 − k`, `angle_min = −fov/2`; no return = `+inf` (REP 117) |
| incoming map | values ≥ 50 and unknown (−1) are obstacles |

The start and goal parameters are simulator cells, the same numbers as in the CLI.
Goals from RViz2 are ROS positions. They are converted to the cell that contains them.

## Tests

`colcon test` (or `scripts/build_ros2.sh --test`) runs 21 tests:

| Suite | Checks |
|---|---|
| `test_conversions` (gtest, 9) | rows flipped and round trip on a random map; unknown and high-cost cells are obstacles; points, headings and step directions agree; quaternions; cells from ROS positions; path poses face the next cell; covariance quadratic form and signs; every LaserScan beam points at the same world location as its simulator beam (full and partial fields of view, 3 headings); misses are `+inf` |
| `test_mission` (gtest, 4) | a mission equals `planWithFootprint` + `simulatePath` step for step (point and differential drive, radius 0 and 0.2); sensors, EKF and moving obstacles, with replans and `pathAt` following them; a failed plan leaves no trajectory; option validation |
| `test_nodes.launch.py` (launch_testing, 5) | both nodes at 20× speed with the differential drive, a 36-beam LiDAR, the EKF and moving obstacles: `/map` matches the CSV with the top row flipped; the plan ends at the goal in ROS coordinates; `/navigation_status` reaches `GOAL_REACHED` and `/odom` is within 5 cm of the goal; scan, EKF covariance, 3 obstacle markers and the `map` → `base_link` transform; a `/goal_pose` as RViz2 sends it starts a new mission that reaches the new goal; clean shutdown |

## Limitations

- **Precomputed missions.** Each mission is simulated in full when it starts (1–3 ms on
  the sample maps), then played back. A new goal, pose or map interrupts playback and
  starts a new mission. Within a mission, the moving obstacles and the robot's reactions
  are those of the standalone simulator.
- **Snapping on restart.** A new mission starts from the center of the robot's current
  cell, with its current heading. A robot between two cells jumps up to half a cell.
  Moving-obstacle time restarts at 0.
- **No velocity commands.** The node does not take `/cmd_vel`. The robot follows the
  simulator's own controller, so it cannot be driven by Nav2.
- **Map origin.** An incoming `OccupancyGrid` must have its origin at `(0, 0)` with no
  rotation. `map_server_node` always publishes it that way.
- **Point-robot velocity.** `/odom` twist for the point robot is computed from successive
  positions (it has no heading of its own).
