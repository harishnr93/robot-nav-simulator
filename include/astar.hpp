// A* path planning on a four-connected occupancy grid.
//
// Model: moves up/down/left/right, uniform cost 1 per move,
// Manhattan-distance heuristic (admissible and consistent for this model,
// so the first time the goal is expanded its cost is optimal).
//
// Deterministic tie-breaking: open-set entries are ordered by
//   (lowest f, then lowest h, then lowest y, then lowest x).
// A node's predecessor is only replaced by a strictly cheaper one.
// The same map and request therefore always yield the same path.
#pragma once

#include <optional>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// |dx| + |dy| between two cells.
int manhattanDistance(GridCoord a, GridCoord b);

/// Checks bounds and occupancy of start and goal. Returns Success if both are valid.
PlanStatus validatePlanRequest(const GridMap& map, GridCoord start, GridCoord goal);

/// Plan a shortest collision-free path from start to goal.
PlanResult planAStar(const GridMap& map, GridCoord start, GridCoord goal);

/// True if the path is non-empty, every cell is free and in bounds,
/// and consecutive cells are 4-connected neighbors.
bool isValidGridPath(const GridMap& map, const std::vector<GridCoord>& path);

/// Sum of Euclidean distances between consecutive cell centers, in meters.
double pathLengthMeters(const GridMap& map, const std::vector<GridCoord>& path);

/// Reference breadth-first search returning only the shortest path cost.
/// Used for validating A* optimality in tests; not used for normal planning.
/// Returns std::nullopt if start/goal are invalid or the goal is unreachable.
std::optional<int> bfsShortestPathCost(const GridMap& map, GridCoord start, GridCoord goal);

}  // namespace amr
