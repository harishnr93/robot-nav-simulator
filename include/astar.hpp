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
//
// Memory per cell (see AStarPlanner): one state byte (visited, closed, and the
// direction to the parent in 2 bits) plus a 4-byte cost that is only written for
// cells the search reaches; untouched cells cost one byte.
//
// Open set: every move changes f = g + h by exactly 0 or +2 (g grows by 1, h
// changes by +-1), and the heuristic is consistent, so the open set only ever
// holds two f values, F and F + 2. It is stored as two std::priority_queue
// min-heaps (one per f value) of packed 64-bit keys (h << 32 | cell index): a single
// integer comparison implements the (h, y, x) tie-break, and the pop order is
// exactly (f, h, y, x).
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// |dx| + |dy| between two cells.
int manhattanDistance(GridCoord a, GridCoord b);

/// Checks bounds and occupancy of start and goal. Returns Success if both are valid.
PlanStatus validatePlanRequest(const GridMap& map, GridCoord start, GridCoord goal);

/// Reusable A* planner.
///
/// The search buffers are kept between plan() calls and reset in time proportional
/// to the number of cells the previous search touched, not to the map size, so
/// repeated planning (benchmarks, replanning) avoids a per-request
/// O(width x height) set-up. Results are identical to planAStar().
/// Not thread-safe: use one planner per thread.
class AStarPlanner {
public:
    PlanResult plan(const GridMap& map, GridCoord start, GridCoord goal);

private:
    /// std::priority_queue min-heap whose storage is kept between searches.
    class KeyHeap : public std::priority_queue<std::uint64_t, std::vector<std::uint64_t>,
                                               std::greater<std::uint64_t>> {
    public:
        void clear() { c.clear(); }  // keeps the vector's capacity
    };

    void prepare(std::size_t cell_count);
    void search(const GridMap& map, GridCoord start, GridCoord goal, PlanResult& result);

    std::unique_ptr<int[]> cost_;          // g(n); only meaningful where state_ is visited
    std::vector<std::uint8_t> state_;      // per-cell flags; all zero between plans
    std::vector<int> touched_;             // cells whose state_ must be cleared
    KeyHeap open_now_;                     // keys (h << 32 | index) of entries with f == F
    KeyHeap open_next_;                    // keys of entries with f == F + 2
    std::size_t capacity_ = 0;             // cells allocated in cost_ / state_
};

/// Plan a shortest collision-free path from start to goal (single request).
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
