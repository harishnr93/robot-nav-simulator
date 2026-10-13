// CSV export of planning results.
//
// path.csv columns:
//   waypoint_index,grid_x,grid_y,world_x,world_y
// planning_metrics.csv (one header row, one data row):
//   status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,
//   planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m,
//   reference_bfs_cost,robot_radius_m,inflation_time_ms,connectivity,diagonal_steps,
//   path_cost_cells
// path_length_steps, path_length_m, diagonal_steps and path_cost_cells are left
// empty when planning failed. path_cost_cells is the exact path cost in cell units
// (straight moves + diagonal moves * sqrt(2)), printed with 6 decimals.
// reference_bfs_cost is empty unless the reference check was requested, and
// kReferenceNoPath (-1) when the reference search found no path. Otherwise it is the
// reference cost: an integer number of steps (BFS) for 4-connectivity, and the cost
// in cell units with 6 decimals (exact Dijkstra) for 8-connectivity.
// trajectory.csv columns:
//   time,x,y,target_waypoint_index
// plus, for the differential-drive model only:
//   theta,linear_velocity,angular_velocity
// (heading in rad; velocities applied during the step that ended at the sample)
// simulation_metrics.csv (one header row, one data row):
//   status,success,navigation_success,steps,travel_time_s,distance_traveled_m,
//   final_goal_error_m,collision_count,speed_mps,dt_s,goal_tolerance_m,max_time_s,
//   robot_radius_m,robot_model,turns,turn_time_s,total_rotation_rad,final_heading_rad
// (the last four are empty for the point robot)
// inflated_map.csv (only for robot_radius_m > 0): the planning map, in the same
// format as input maps (rows of comma-separated 0/1, row 0 first).
// navigation_success = planning succeeded AND the robot reached the goal without
// collision or timeout. Distance/time fields are empty when the simulation did not run.
#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace amr {

/// reference_bfs_cost value meaning "the reference search found no path".
constexpr int kReferenceNoPath = -1;

/// Outcome of the optional reference shortest-path check.
struct ReferenceCost {
    bool computed = false;         ///< false: the check was not run
    std::optional<GridCost> cost;  ///< std::nullopt: the reference found no path
};

/// Writes path.csv. An empty path produces a header-only file.
/// Returns false and sets error on I/O failure.
bool writePathCsv(const std::filesystem::path& file, const GridMap& map,
                  const std::vector<GridCoord>& path, std::string& error);

/// Writes planning_metrics.csv. Returns false and sets error on I/O failure.
/// robot_radius / inflation_time_ms describe the obstacle inflation used for planning.
bool writePlanningMetricsCsv(const std::filesystem::path& file, const GridMap& map,
                             GridCoord start, GridCoord goal, const PlanResult& result,
                             const ReferenceCost& reference, double robot_radius,
                             double inflation_time_ms, Connectivity connectivity,
                             std::string& error);

/// Writes a map in the input map format (rows of comma-separated 0/1).
bool writeGridMapCsv(const std::filesystem::path& file, const GridMap& map, std::string& error);

/// Writes trajectory.csv. An empty trajectory produces a header-only file.
/// with_heading adds the differential-drive columns.
bool writeTrajectoryCsv(const std::filesystem::path& file,
                        const std::vector<TrajectorySample>& trajectory, bool with_heading,
                        std::string& error);

/// Writes simulation_metrics.csv. `planning_succeeded` feeds navigation_success.
bool writeSimulationMetricsCsv(const std::filesystem::path& file, bool planning_succeeded,
                               const MotionConfig& config, const SimulationResult& result,
                               std::string& error);

}  // namespace amr
