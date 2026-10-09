cmake_minimum_required(VERSION 3.16)

project(RobotNavSimulator
    VERSION 0.2.0
    DESCRIPTION "Autonomous Mobile Robot Navigation Simulator"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(AMR_BUILD_TESTS "Build the automated tests" ON)

set(AMR_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# ---------------------------------------------------------------------------
# Core library: map loading, A* planning, CSV export
# ---------------------------------------------------------------------------
add_library(amr_core STATIC
    src/grid_map.cpp
    src/astar.cpp
    src/csv_writer.cpp)
target_include_directories(amr_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_options(amr_core PRIVATE ${AMR_WARNING_FLAGS})

# ---------------------------------------------------------------------------
# Command-line executable (placed at build/robot_simulator)
# ---------------------------------------------------------------------------
add_executable(robot_simulator src/main.cpp)
target_link_libraries(robot_simulator PRIVATE amr_core)
target_compile_options(robot_simulator PRIVATE ${AMR_WARNING_FLAGS})

# ---------------------------------------------------------------------------
# Tests (run with: ctest --test-dir build --output-on-failure)
# ---------------------------------------------------------------------------
if(AMR_BUILD_TESTS)
    enable_testing()

    foreach(test_name test_grid_map test_astar)
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
        PASS_REGULAR_EXPRESSION "Planner status: SUCCESS")

    add_test(NAME cli_plan_maze_map
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/maps/maze.csv
                --start 1 1 --goal 19 19 --output ${CLI_OUT}/maze)
    set_tests_properties(cli_plan_maze_map PROPERTIES
        PASS_REGULAR_EXPRESSION "Planner status: SUCCESS")

    add_test(NAME cli_reports_unreachable_goal
        COMMAND robot_simulator --map ${PROJECT_SOURCE_DIR}/tests/data/unreachable_goal.csv
                --start 0 0 --goal 4 4 --output ${CLI_OUT}/unreachable)
    set_tests_properties(cli_reports_unreachable_goal PROPERTIES
        PASS_REGULAR_EXPRESSION "Planner status: NO_PATH")

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
endif()
