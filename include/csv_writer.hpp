// CSV export of planning results.
//
// path.csv columns:
//   waypoint_index,grid_x,grid_y,world_x,world_y
// planning_metrics.csv (one header row, one data row):
//   status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,
//   planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m
// path_length_steps and path_length_m are left empty when planning failed.
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// Writes path.csv. An empty path produces a header-only file.
/// Returns false and sets error on I/O failure.
bool writePathCsv(const std::filesystem::path& file, const GridMap& map,
                  const std::vector<GridCoord>& path, std::string& error);

/// Writes planning_metrics.csv. Returns false and sets error on I/O failure.
bool writePlanningMetricsCsv(const std::filesystem::path& file, const GridMap& map,
                             GridCoord start, GridCoord goal, const PlanResult& result,
                             std::string& error);

}  // namespace amr
