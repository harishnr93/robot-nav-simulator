cmake_minimum_required(VERSION 3.16)
project(robot_nav_sim_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(AMR_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------------------------------------------------------------------
# The simulator core, compiled from the repository's own sources (the same list as
# the standalone build), so the ROS nodes run exactly the standalone code.
# ---------------------------------------------------------------------------
get_filename_component(AMR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
include(${AMR_ROOT}/cmake/amr_core_sources.cmake)
list(TRANSFORM AMR_CORE_SOURCES PREPEND "${AMR_ROOT}/")
add_library(amr_core_ros STATIC ${AMR_CORE_SOURCES})
target_include_directories(amr_core_ros PUBLIC ${AMR_ROOT}/include)
set_target_properties(amr_core_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# ROS-independent mission logic and the frame conversions (unit-tested).
# ---------------------------------------------------------------------------
add_library(robot_nav_sim_ros_lib STATIC src/mission.cpp src/conversions.cpp)
target_include_directories(robot_nav_sim_ros_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(robot_nav_sim_ros_lib PUBLIC amr_core_ros)
target_compile_options(robot_nav_sim_ros_lib PRIVATE ${AMR_WARNING_FLAGS})
ament_target_dependencies(robot_nav_sim_ros_lib PUBLIC
    rclcpp geometry_msgs nav_msgs sensor_msgs)

add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node robot_nav_sim_ros_lib)
target_compile_options(map_server_node PRIVATE ${AMR_WARNING_FLAGS})
ament_target_dependencies(map_server_node rclcpp nav_msgs)

add_executable(simulator_node src/simulator_node.cpp)
target_link_libraries(simulator_node robot_nav_sim_ros_lib)
target_compile_options(simulator_node PRIVATE ${AMR_WARNING_FLAGS})
ament_target_dependencies(simulator_node
    rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2_ros)

install(TARGETS map_server_node simulator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# The repository's sample maps and moving-obstacle files, for the launch file.
install(DIRECTORY ${AMR_ROOT}/maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)
    ament_add_gtest(test_conversions test/test_conversions.cpp)
    target_link_libraries(test_conversions robot_nav_sim_ros_lib)
    ament_add_gtest(test_mission test/test_mission.cpp
        ENV AMR_MAPS_DIR=${AMR_ROOT}/maps)
    target_link_libraries(test_mission robot_nav_sim_ros_lib)
    add_launch_test(test/test_nodes.launch.py TIMEOUT 180)
endif()

ament_package()
