// Shared value types used across the simulator.
//
// Coordinate conventions (used consistently by every module):
//   * Grid coordinate (x, y): x = column index, y = row index.
//   * The grid origin (0, 0) is the upper-left cell.
//   * Positive x points right; positive y points DOWN.
//   * World coordinates are in meters with the same axis orientation.
//   * A cell's world position is its center:
//         world_x = (grid_x + 0.5) * resolution
//         world_y = (grid_y + 0.5) * resolution
#pragma once

#include <cstddef>
#include <ostream>
#include <vector>

namespace amr {

/// Integer cell index in the occupancy grid (x = column, y = row).
struct GridCoord {
    int x = 0;
    int y = 0;
};

inline bool operator==(const GridCoord& a, const GridCoord& b) {
    return a.x == b.x && a.y == b.y;
}

inline bool operator!=(const GridCoord& a, const GridCoord& b) {
    return !(a == b);
}

inline std::ostream& operator<<(std::ostream& os, const GridCoord& c) {
    return os << '(' << c.x << ", " << c.y << ')';
}

/// Continuous position in meters, same axis orientation as GridCoord.
struct WorldPoint {
    double x = 0.0;
    double y = 0.0;
};

/// Outcome of a planning request. Only Success carries a path.
enum class PlanStatus {
    Success,
    StartOutOfBounds,
    GoalOutOfBounds,
    StartOccupied,
    GoalOccupied,
    NoPath,
};

inline const char* toString(PlanStatus status) {
    switch (status) {
        case PlanStatus::Success:          return "SUCCESS";
        case PlanStatus::StartOutOfBounds: return "START_OUT_OF_BOUNDS";
        case PlanStatus::GoalOutOfBounds:  return "GOAL_OUT_OF_BOUNDS";
        case PlanStatus::StartOccupied:    return "START_OCCUPIED";
        case PlanStatus::GoalOccupied:     return "GOAL_OCCUPIED";
        case PlanStatus::NoPath:           return "NO_PATH";
    }
    return "UNKNOWN";
}

/// Result of a single A* planning request.
struct PlanResult {
    PlanStatus status = PlanStatus::NoPath;
    /// Ordered cells from start to goal (both included). Empty unless Success.
    std::vector<GridCoord> path;
    /// Path cost in grid steps (uniform cost 1 per move). 0 unless Success.
    int path_cost = 0;
    /// Number of nodes removed from the open set and expanded (goal included).
    std::size_t expanded_nodes = 0;
    /// Wall-clock planning time measured with std::chrono::steady_clock.
    double planning_time_ms = 0.0;

    bool success() const { return status == PlanStatus::Success; }
};

}  // namespace amr
