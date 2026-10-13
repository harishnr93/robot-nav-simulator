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

#include <cmath>
#include <cstddef>
#include <cstdint>
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

/// Grid moves allowed for planning: 4 = up/down/left/right (cost 1 each);
/// 8 = additionally the four diagonals (cost sqrt(2) each, no corner cutting).
enum class Connectivity {
    Four = 4,
    Eight = 8,
};

/// Exact path cost in cell units: straight * 1 + diagonal * sqrt(2).
/// Kept as two integers so costs can be compared exactly (see compareGridCost).
struct GridCost {
    std::int64_t straight = 0;
    std::int64_t diagonal = 0;

    double cells() const { return static_cast<double>(straight) +
                                  static_cast<double>(diagonal) * std::sqrt(2.0); }
};

inline bool operator==(const GridCost& a, const GridCost& b) {
    return a.straight == b.straight && a.diagonal == b.diagonal;
}

/// Outcome of a planning request. Only Success carries a path.
enum class PlanStatus {
    Success,
    StartOutOfBounds,
    GoalOutOfBounds,
    StartOccupied,
    GoalOccupied,
    StartTooCloseToObstacle,  ///< start is free, but the robot footprint does not fit there
    GoalTooCloseToObstacle,   ///< goal is free, but the robot footprint does not fit there
    NoPath,
};

inline const char* toString(PlanStatus status) {
    switch (status) {
        case PlanStatus::Success:          return "SUCCESS";
        case PlanStatus::StartOutOfBounds: return "START_OUT_OF_BOUNDS";
        case PlanStatus::GoalOutOfBounds:  return "GOAL_OUT_OF_BOUNDS";
        case PlanStatus::StartOccupied:    return "START_OCCUPIED";
        case PlanStatus::GoalOccupied:     return "GOAL_OCCUPIED";
        case PlanStatus::StartTooCloseToObstacle: return "START_TOO_CLOSE_TO_OBSTACLE";
        case PlanStatus::GoalTooCloseToObstacle:  return "GOAL_TOO_CLOSE_TO_OBSTACLE";
        case PlanStatus::NoPath:           return "NO_PATH";
    }
    return "UNKNOWN";
}

/// Result of a single A* planning request.
struct PlanResult {
    PlanStatus status = PlanStatus::NoPath;
    /// Ordered cells from start to goal (both included). Empty unless Success.
    std::vector<GridCoord> path;
    /// Number of moves (grid steps) on the path. 0 unless Success. With
    /// 4-connectivity every move costs 1, so this is also the path cost.
    int path_cost = 0;
    /// How many of those moves are diagonal (8-connectivity only).
    int diagonal_moves = 0;
    /// Number of nodes removed from the open set and expanded (goal included).
    std::size_t expanded_nodes = 0;
    /// Wall-clock planning time measured with std::chrono::steady_clock.
    double planning_time_ms = 0.0;

    bool success() const { return status == PlanStatus::Success; }
    /// Exact path cost: (path_cost - diagonal_moves) * 1 + diagonal_moves * sqrt(2).
    GridCost cost() const { return {path_cost - diagonal_moves, diagonal_moves}; }
};

}  // namespace amr
