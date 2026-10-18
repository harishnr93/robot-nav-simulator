cmake_minimum_required(VERSION 3.16)

project(RobotNavSimulator
    VERSION 1.0.0
    DESCRIPTION "Autonomous Mobile Robot Navigation Simulator"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(AMR_BUILD_TESTS "Build the automated tests" ON)
option(AMR_BUILD_BENCHMARKS "Build the in-process planner micro-benchmark" ON)
option(AMR_PYTHON_TESTS "Register the Python visualization tests (needs numpy, matplotlib, pillow)" ON)

set(AMR_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# ---------------------------------------------------------------------------
# Core library: map loading, A* planning, motion simulation, CSV export
# ---------------------------------------------------------------------------
include(cmake/amr_core_sources.cmake)  # AMR_CORE_SOURCES, shared with the ROS 2 package
add_library(amr_core STATIC ${AMR_CORE_SOURCES})
target_include_directories(amr_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_options(amr_core PRIVATE ${AMR_WARNING_FLAGS})

# ---------------------------------------------------------------------------
# Command-line executable (placed at build/robot_simulator)
# ---------------------------------------------------------------------------
add_executable(robot_simulator src/main.cpp)
target_link_libraries(robot_simulator PRIVATE amr_core)
target_compile_options(robot_simulator PRIVATE ${AMR_WARNING_FLAGS})

# ---------------------------------------------------------------------------
# P vs. PID heading-control experiment (run with: ./build/heading_experiment)
# ---------------------------------------------------------------------------
add_executable(heading_experiment src/heading_experiment.cpp)
target_link_libraries(heading_experiment PRIVATE amr_core)
target_compile_options(heading_experiment PRIVATE ${AMR_WARNING_FLAGS})

# Grid-alignment experiments (run with: ./build/alignment_experiment doorway|clutter|classify)
add_executable(alignment_experiment src/alignment_experiment.cpp)
target_link_libraries(alignment_experiment PRIVATE amr_core)
target_compile_options(alignment_experiment PRIVATE ${AMR_WARNING_FLAGS})

# LiDAR perception experiments (run with: ./build/perception_experiment ekf|grazing|dynamic)
add_executable(perception_experiment src/perception_experiment.cpp)
target_link_libraries(perception_experiment PRIVATE amr_core)
target_compile_options(perception_experiment PRIVATE ${AMR_WARNING_FLAGS})

# Localization experiments (run with: ./build/localization_experiment global|kidnap|...)
add_executable(localization_experiment src/localization_experiment.cpp)
target_link_libraries(localization_experiment PRIVATE amr_core)
target_compile_options(localization_experiment PRIVATE ${AMR_WARNING_FLAGS})

# Planning with moving obstacles (run with: ./build/planning_experiment turns|spacetime|margins|robots)
find_package(Threads REQUIRED)
add_executable(planning_experiment src/planning_experiment.cpp)
target_link_libraries(planning_experiment PRIVATE amr_core Threads::Threads)
target_compile_options(planning_experiment PRIVATE ${AMR_WARNING_FLAGS})

# "Why NO_PATH?" experiments (run with: ./build/ghost_experiment recovery|scaling)
add_executable(ghost_experiment src/ghost_experiment.cpp)
target_link_libraries(ghost_experiment PRIVATE amr_core)
target_compile_options(ghost_experiment PRIVATE ${AMR_WARNING_FLAGS})

# ---------------------------------------------------------------------------
# Micro-benchmark (run with: ./build/bench_planner [--quick])
# ---------------------------------------------------------------------------
if(AMR_BUILD_BENCHMARKS)
    add_executable(bench_planner benchmarks/bench_planner.cpp)
    target_link_libraries(bench_planner PRIVATE amr_core)
    target_compile_options(bench_planner PRIVATE ${AMR_WARNING_FLAGS})
    target_compile_definitions(bench_planner PRIVATE AMR_HAS_PLANNER_CLASS)
endif()

# ---------------------------------------------------------------------------
# Tests (run with: ctest --test-dir build --output-on-failure)
# ---------------------------------------------------------------------------
if(AMR_BUILD_TESTS)
    enable_testing()

    foreach(test_name test_grid_map test_astar test_robot test_footprint test_octile test_diff_drive test_heading_control test_lidar test_ekf test_correlated_errors test_obstacle_tracking test_localization test_dynamic test_ghost_cells test_ghost_perception test_alignment test_planning)
        add_executable(${test_name} tests/${test_name}.cpp)
        target_link_libraries(${test_name} PRIVATE amr_core)
        target_compile_options(${test_name} PRIVATE ${AMR_WARNING_FLAGS})
        target_compile_definitions(${test_name} PRIVATE
            AMR_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()

    # End-to-end CLI checks.
    set(CLI_OUT ${PROJECT_BINARY_DIR}/test_output)

    add_test(NAME cli_help COMMAND robot_simulator --help)

    add_test(NAME cli_plan_obstacles_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --output ${CLI_OUT}/obstacles)
    set_tests_properties(cli_plan_obstacles_map PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED")

    add_test(NAME cli_plan_maze_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/maze.csv
                --start 1 1 --goal 19 19 --output ${CLI_OUT}/maze)
    set_tests_properties(cli_plan_maze_map PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED")

    add_test(NAME cli_reference_bfs_matches
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/corridor.csv
                --start 1 1 --goal 18 18 --reference-bfs --output ${CLI_OUT}/reference)
    set_tests_properties(cli_reference_bfs_matches PROPERTIES
        PASS_REGULAR_EXPRESSION "Reference BFS: +68 steps \\(matches A\\*\\)")

    add_test(NAME cli_reference_bfs_no_path
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/tests/data/unreachable_goal.csv
                --start 0 0 --goal 4 4 --reference-bfs --output ${CLI_OUT}/reference_none)
    set_tests_properties(cli_reference_bfs_no_path PROPERTIES
        PASS_REGULAR_EXPRESSION "Reference BFS: +no path \\(matches A\\*\\)")

    add_test(NAME cli_footprint_fine_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles_fine.csv
                --resolution 0.25 --start 20 2 --goal 20 37 --robot-radius 0.2
                --reference-bfs --output ${CLI_OUT}/footprint_fine)
    set_tests_properties(cli_footprint_fine_map PROPERTIES
        PASS_REGULAR_EXPRESSION "Reference BFS: +45 steps \\(matches A\\*\\).*Navigation: +GOAL_REACHED")

    add_test(NAME cli_footprint_too_wide_for_gaps
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-radius 0.3 --output ${CLI_OUT}/footprint_wide)
    set_tests_properties(cli_footprint_too_wide_for_gaps PROPERTIES
        PASS_REGULAR_EXPRESSION "Planner status: NO_PATH")

    add_test(NAME cli_footprint_start_too_close
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 3 5 --goal 18 18 --robot-radius 0.3 --output ${CLI_OUT}/footprint_close)
    set_tests_properties(cli_footprint_start_too_close PROPERTIES
        PASS_REGULAR_EXPRESSION "Planner status: START_TOO_CLOSE_TO_OBSTACLE")

    add_test(NAME cli_rejects_negative_radius
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --robot-radius -0.1)
    set_tests_properties(cli_rejects_negative_radius PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid motion parameters: robot radius")

    add_test(NAME cli_octile_obstacles_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --connectivity 8 --reference-bfs
                --output ${CLI_OUT}/octile)
    set_tests_properties(cli_octile_obstacles_map PROPERTIES
        PASS_REGULAR_EXPRESSION "26 steps \\(8 diagonal\\), 14.66 m.*29.313708 cells \\(matches A\\*\\).*GOAL_REACHED")

    add_test(NAME cli_octile_maze_has_no_diagonals
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/maze.csv
                --start 1 1 --goal 19 19 --connectivity 8 --output ${CLI_OUT}/octile_maze)
    set_tests_properties(cli_octile_maze_has_no_diagonals PROPERTIES
        PASS_REGULAR_EXPRESSION "60 steps \\(0 diagonal\\)")

    add_test(NAME cli_octile_with_footprint
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles_fine.csv
                --resolution 0.25 --start 20 2 --goal 20 37 --robot-radius 0.2
                --connectivity 8 --reference-bfs --output ${CLI_OUT}/octile_footprint)
    set_tests_properties(cli_octile_with_footprint PROPERTIES
        PASS_REGULAR_EXPRESSION "matches A\\*.*Navigation: +GOAL_REACHED")

    add_test(NAME cli_rejects_bad_connectivity
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --connectivity 6)
    set_tests_properties(cli_rejects_bad_connectivity PROPERTIES
        PASS_REGULAR_EXPRESSION "--connectivity must be 4 or 8")

    add_test(NAME cli_diff_drive_obstacles_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-model diff-drive --output ${CLI_OUT}/diff_drive)
    set_tests_properties(cli_diff_drive_obstacles_map PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED.*travel time 38.50 s.*3 turns in place, 4.50 s turning")

    add_test(NAME cli_diff_drive_footprint_octile
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles_fine.csv
                --resolution 0.25 --start 20 2 --goal 20 37 --robot-radius 0.2
                --connectivity 8 --robot-model diff-drive --initial-heading 90
                --output ${CLI_OUT}/diff_drive_combined)
    set_tests_properties(cli_diff_drive_footprint_octile PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED.*collisions 0.*differential drive")

    add_test(NAME cli_rejects_bad_robot_model
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --robot-model tank)
    set_tests_properties(cli_rejects_bad_robot_model PROPERTIES
        PASS_REGULAR_EXPRESSION "--robot-model must be point or diff-drive")

    add_test(NAME cli_rejects_overshooting_heading_gain
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --robot-model diff-drive --heading-gain 20)
    set_tests_properties(cli_rejects_overshooting_heading_gain PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid motion parameters: heading gain")

    add_test(NAME heading_experiment_p_vs_pid
        COMMAND heading_experiment --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --output ${CLI_OUT}/heading)
    set_tests_properties(heading_experiment_p_vs_pid PROPERTIES
        PASS_REGULAR_EXPRESSION "step_disturbance  P +[0-9.]+% +never.*step_disturbance  PID +[0-9.]+% +[0-9.]+ s")

    add_test(NAME heading_experiment_rejects_bad_gain
        COMMAND heading_experiment --kp -1)
    set_tests_properties(heading_experiment_rejects_bad_gain PROPERTIES WILL_FAIL TRUE)

    add_test(NAME cli_lidar_scans
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-model diff-drive --lidar-beams 72
                --output ${CLI_OUT}/lidar)
    set_tests_properties(cli_lidar_scans PROPERTIES
        PASS_REGULAR_EXPRESSION "LiDAR: +72 beams over 360.00 deg, range 5.00 m, 386 scans.*closest 0.250 m")

    add_test(NAME cli_rejects_bad_lidar_settings
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --lidar-beams 8 --lidar-fov 400)
    set_tests_properties(cli_rejects_bad_lidar_settings PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid LiDAR parameters: LiDAR field of view")

    add_test(NAME cli_ekf_localization
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-model diff-drive --lidar-beams 72 --ekf
                --output ${CLI_OUT}/ekf)
    set_tests_properties(cli_ekf_localization PROPERTIES
        PASS_REGULAR_EXPRESSION "EKF estimate: +position error RMS 0.003 m.*Dead reckoning: +position error RMS 0.533 m")

    add_test(NAME cli_ekf_requires_diff_drive
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --ekf)
    set_tests_properties(cli_ekf_requires_diff_drive PROPERTIES
        PASS_REGULAR_EXPRESSION "--ekf requires --robot-model diff-drive")

    add_test(NAME cli_rejects_bad_ekf_settings
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --robot-model diff-drive --ekf --ekf-range-sigma 0)
    set_tests_properties(cli_rejects_bad_ekf_settings PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid EKF parameters: EKF range sigma")

    add_test(NAME cli_particle_filter_global
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-model diff-drive --lidar-beams 72 --ekf
                --particle-filter --output ${CLI_OUT}/particle_filter)
    set_tests_properties(cli_particle_filter_global PROPERTIES
        PASS_REGULAR_EXPRESSION "Particle filter: found the robot after 0.0 s")

    add_test(NAME cli_steer_by_estimate_with_localization_planning
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/hall.csv
                --start 6 8 --goal 38 22 --robot-model diff-drive --robot-radius 0.25
                --lidar-beams 72 --lidar-range 3 --ekf --connectivity 8
                --plan-for-localization 1 --steer-by-estimate --output ${CLI_OUT}/steering)
    set_tests_properties(cli_steer_by_estimate_with_localization_planning PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED")

    add_test(NAME cli_plan_for_localization_requires_8
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/hall.csv
                --start 6 8 --goal 38 22 --plan-for-localization 1)
    set_tests_properties(cli_plan_for_localization_requires_8 PROPERTIES WILL_FAIL TRUE)

    add_test(NAME cli_dynamic_replanning
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18
                --dynamic-obstacles ${PROJECT_SOURCE_DIR}/maps/obstacles_dynamic.csv
                --output ${CLI_OUT}/dynamic)
    set_tests_properties(cli_dynamic_replanning PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED.*Dynamic: +3 moving obstacle\\(s\\), sensing range 3.00 m. 2 replan\\(s\\)")

    add_test(NAME cli_rejects_bad_dynamic_obstacles
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18
                --dynamic-obstacles ${PROJECT_SOURCE_DIR}/tests/data/bad_dynamic_obstacles.csv)
    set_tests_properties(cli_rejects_bad_dynamic_obstacles PROPERTIES
        PASS_REGULAR_EXPRESSION "line 2: route segment 0-1 crosses a static obstacle")

    add_test(NAME cli_dynamic_spacetime
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-model diff-drive
                --dynamic-obstacles ${PROJECT_SOURCE_DIR}/maps/obstacles_dynamic.csv
                --dynamic-planner space-time --livelock-window 15 --output ${CLI_OUT}/dynamic_spacetime)
    set_tests_properties(cli_dynamic_spacetime PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +GOAL_REACHED.*planner space-time, livelock switch after 15.0 s: 0 switch")

    add_test(NAME cli_turn_aware
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --robot-model diff-drive --turn-aware
                --output ${CLI_OUT}/turn_aware)
    set_tests_properties(cli_turn_aware PROPERTIES
        PASS_REGULAR_EXPRESSION "travel time 37.00 s.*2 turns in place")

    add_test(NAME cli_turn_aware_requires_diff_drive
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --turn-aware)
    set_tests_properties(cli_turn_aware_requires_diff_drive PROPERTIES
        PASS_REGULAR_EXPRESSION "--turn-aware requires --robot-model diff-drive")

    add_test(NAME cli_rejects_bad_dynamic_planner
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 1 1 --goal 18 18 --dynamic-planner sideways)
    set_tests_properties(cli_rejects_bad_dynamic_planner PROPERTIES
        PASS_REGULAR_EXPRESSION "--dynamic-planner must be reactive or space-time")

    add_test(NAME cli_rejects_bad_spacetime_horizon
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --spacetime-horizon 0)
    set_tests_properties(cli_rejects_bad_spacetime_horizon PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid replanning parameters: space-time horizon")

    add_test(NAME planning_experiment_turns
        COMMAND planning_experiment turns --runs 20 --output ${CLI_OUT}/planning_turns)
    set_tests_properties(planning_experiment_turns PROPERTIES
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        PASS_REGULAR_EXPRESSION "time saved per route")

    add_test(NAME cli_rejects_bad_replanning_settings
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --sensing-range 0)
    set_tests_properties(cli_rejects_bad_replanning_settings PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid replanning parameters: sensing range")

    add_test(NAME ghost_experiment_example
        COMMAND ghost_experiment example --seed 23 --density 0.08 --output ${CLI_OUT}/ghost)
    set_tests_properties(ghost_experiment_example PROPERTIES
        PASS_REGULAR_EXPRESSION "status FOUND, 2 cell\\(s\\), 8 optimal explanation\\(s\\)")

    add_test(NAME alignment_experiment_example
        COMMAND alignment_experiment example --output ${CLI_OUT}/alignment)
    set_tests_properties(alignment_experiment_example PROPERTIES
        PASS_REGULAR_EXPRESSION "example 0: angle 0 deg.*passable.*example 1: angle 0 deg.*NO_PATH")

    add_test(NAME cli_navigation_timeout
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/corridor.csv
                --start 1 1 --goal 18 18 --max-time 2 --output ${CLI_OUT}/timeout)
    set_tests_properties(cli_navigation_timeout PROPERTIES
        PASS_REGULAR_EXPRESSION "Navigation: +TIMEOUT")

    add_test(NAME cli_rejects_invalid_motion_parameters
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv
                --start 1 1 --goal 2 2 --speed -1)
    set_tests_properties(cli_rejects_invalid_motion_parameters PROPERTIES
        PASS_REGULAR_EXPRESSION "invalid motion parameters")

    add_test(NAME cli_reports_unreachable_goal
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/tests/data/unreachable_goal.csv
                --start 0 0 --goal 4 4 --output ${CLI_OUT}/unreachable)
    set_tests_properties(cli_reports_unreachable_goal PROPERTIES
        PASS_REGULAR_EXPRESSION "Planner status: NO_PATH.*Navigation: +NOT_RUN")

    add_test(NAME cli_rejects_occupied_start
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/obstacles.csv
                --start 4 5 --goal 18 18 --output ${CLI_OUT}/occupied_start)
    set_tests_properties(cli_rejects_occupied_start PROPERTIES
        PASS_REGULAR_EXPRESSION "Planner status: START_OCCUPIED")

    add_test(NAME cli_rejects_malformed_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/tests/data/nonrectangular.csv
                --start 0 0 --goal 1 1 --output ${CLI_OUT}/malformed)
    set_tests_properties(cli_rejects_malformed_map PROPERTIES
        PASS_REGULAR_EXPRESSION "must be rectangular")

    add_test(NAME cli_rejects_missing_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/does_not_exist.csv
                --start 0 0 --goal 1 1 --output ${CLI_OUT}/missing)
    set_tests_properties(cli_rejects_missing_map PROPERTIES
        PASS_REGULAR_EXPRESSION "does not exist")

    add_test(NAME cli_rejects_bad_arguments
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/empty.csv --start 1 x)
    set_tests_properties(cli_rejects_bad_arguments PROPERTIES WILL_FAIL TRUE)

    # Python visualization/animation tests. The project virtual environment
    # (.venv) is used when present; otherwise an active virtual environment or the
    # system Python. Override with -DPython3_EXECUTABLE=/path/to/python.
    if(AMR_PYTHON_TESTS)
        if(NOT Python3_EXECUTABLE AND EXISTS "${PROJECT_SOURCE_DIR}/.venv/bin/python")
            set(Python3_EXECUTABLE "${PROJECT_SOURCE_DIR}/.venv/bin/python"
                CACHE FILEPATH "Python interpreter for the visualization tests")
        endif()
        find_package(Python3 COMPONENTS Interpreter)
        if(Python3_Interpreter_FOUND)
            execute_process(
                COMMAND ${Python3_EXECUTABLE} -c "import numpy, matplotlib, PIL"
                RESULT_VARIABLE AMR_PYTHON_DEPS_RESULT
                OUTPUT_QUIET ERROR_QUIET)
        endif()
        if(Python3_Interpreter_FOUND AND AMR_PYTHON_DEPS_RESULT EQUAL 0)
            message(STATUS "Python tests enabled with ${Python3_EXECUTABLE}")
            add_test(NAME python_visualization
                COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/test_python_visualization.py)
            set_tests_properties(python_visualization PROPERTIES
                ENVIRONMENT "AMR_SOURCE_DIR=${PROJECT_SOURCE_DIR};AMR_SIMULATOR=$<TARGET_FILE:robot_simulator>;MPLBACKEND=Agg")
        else()
            message(STATUS "Python tests skipped: no interpreter with numpy, matplotlib "
                           "and pillow (activate .venv or set -DPython3_EXECUTABLE)")
        endif()
    endif()
endif()
