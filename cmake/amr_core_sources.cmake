# Sources of the core library (map loading, planning, simulation, sensing, export),
# relative to the repository root. Shared by the standalone build (CMakeLists.txt)
# and the ROS 2 package (ros2/robot_nav_sim_ros), so both compile the same code.
set(AMR_CORE_SOURCES
    src/grid_map.cpp
    src/astar.cpp
    src/robot.cpp
    src/simulator.cpp
    src/footprint.cpp
    src/diff_drive.cpp
    src/heading_control.cpp
    src/lidar.cpp
    src/ekf.cpp
    src/surface_model.cpp
    src/particle_filter.cpp
    src/localization_planning.cpp
    src/closed_loop.cpp
    src/spacetime_planner.cpp
    src/multi_robot.cpp
    src/scenario_search.cpp
    src/statistics.cpp
    src/dynamic_obstacles.cpp
    src/obstacle_tracking.cpp
    src/ghost_cells.cpp
    src/ghost_perception.cpp
    src/geometry2d.cpp
    src/alignment.cpp
    src/csv_writer.cpp)
