// CSV export of planning results.
//
// path.csv columns:
//   waypoint_index,grid_x,grid_y,world_x,world_y
// planning_metrics.csv (one header row, one data row):
//   status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,
//   planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m,
//   reference_bfs_cost
// path_length_steps and path_length_m are left empty when planning failed.
// reference_bfs_cost is empty unless the BFS check was requested, and
// kReferenceNoPath (-1) when the reference BFS found no path.
// trajectory.csv columns:
//   time,x,y,target_waypoint_index
// simulation_metrics.csv (one header row, one data row):
//   status,success,navigation_success,steps,travel_time_s,distance_traveled_m,
//   final_goal_error_m,collision_count,speed_mps,dt_s,goal_tolerance_m,max_time_s
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

/// reference_bfs_cost value meaning "the reference BFS found no path".
constexpr int kReferenceNoPath = -1;

/// Writes path.csv. An empty path produces a header-only file.
/// Returns false and sets error on I/O failure.
bool writePathCsv(const std::filesystem::path& file, const GridMap& map,
                  const std::vector<GridCoord>& path, std::string& error);

/// Writes planning_metrics.csv. Returns false and sets error on I/O failure.
/// reference_bfs_cost: std::nullopt if not computed, kReferenceNoPath, or a cost.
bool writePlanningMetricsCsv(const std::filesystem::path& file, const GridMap& map,
                             GridCoord start, GridCoord goal, const PlanResult& result,
                             std::optional<int> reference_bfs_cost, std::string& error);

/// Writes trajectory.csv. An empty trajectory produces a header-only file.
bool writeTrajectoryCsv(const std::filesystem::path& file,
                        const std::vector<TrajectorySample>& trajectory, std::string& error);

/// Writes simulation_metrics.csv. `planning_succeeded` feeds navigation_success.
bool writeSimulationMetricsCsv(const std::filesystem::path& file, bool planning_succeeded,
                               const MotionConfig& config, const SimulationResult& result,
                               std::string& error);

}  // namespace amr
