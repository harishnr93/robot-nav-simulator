// A* path planning on a 4- or 8-connected occupancy grid.
//
// 4-connected (default): moves up/down/left/right, uniform cost 1 per move,
// Manhattan-distance heuristic (admissible and consistent for this model,
// so the first time the goal is expanded its cost is optimal).
//
// 8-connected: additionally the four diagonal moves, cost sqrt(2) each. A diagonal
// move is only allowed when both orthogonal cells it passes are free (no cutting
// across obstacle corners). Heuristic: octile distance
//     h = (max(dx, dy) - min(dx, dy)) * 1 + min(dx, dy) * sqrt(2),
// the exact path length on an obstacle-free 8-connected grid, which is admissible
// and consistent. Costs are kept EXACT as integer pairs (straight, diagonal) and
// compared with integer arithmetic (compareGridCost), so there is no rounding and
// tie-breaking stays fully deterministic.
//
// Deterministic tie-breaking: open-set entries are ordered by
//   (lowest f, then lowest h, then lowest y, then lowest x).
// A node's predecessor is only replaced by a strictly cheaper one.
// The same map and request therefore always yield the same path.
//
// Memory per cell (see AStarPlanner): one state byte (visited, closed, and the
// direction to the parent in 3 bits) plus a 4-byte cost that is only written for
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

/// Octile distance between two cells, exactly: (max - min) straight + min diagonal.
GridCost octileDistance(GridCoord a, GridCoord b);

/// Exact comparison of a.straight + a.diagonal * sqrt(2) with the same for b:
/// returns -1, 0 or +1. Uses only integer arithmetic (valid while the squared
/// differences fit in 64 bits, i.e. for any map up to GridMap::kMaxDimension).
int compareGridCost(const GridCost& a, const GridCost& b);

/// True if moving from cell `from` to the adjacent cell `to` is allowed: `to` is
/// free, the move is one step in each axis at most, and a diagonal move does not
/// cut a corner (both orthogonal cells it passes are free). With Four, only
/// straight moves are allowed.
bool isAllowedMove(const GridMap& map, GridCoord from, GridCoord to, Connectivity connectivity);

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
    PlanResult plan(const GridMap& map, GridCoord start, GridCoord goal,
                    Connectivity connectivity = Connectivity::Four);

private:
    /// 8-connected open-set entry: exact f and h, then the cell index.
    struct OctileEntry {
        GridCost f;
        GridCost h;
        int index;
    };
    /// Min-heap order: lowest f, then lowest h, then lowest index (= lowest y, x).
    struct OctileEntryGreater {
        bool operator()(const OctileEntry& a, const OctileEntry& b) const;
    };
    /// std::priority_queue whose storage is kept between searches.
    class OctileHeap : public std::priority_queue<OctileEntry, std::vector<OctileEntry>,
                                                  OctileEntryGreater> {
    public:
        void clear() { c.clear(); }
    };

    /// std::priority_queue min-heap whose storage is kept between searches.
    class KeyHeap : public std::priority_queue<std::uint64_t, std::vector<std::uint64_t>,
                                               std::greater<std::uint64_t>> {
    public:
        void clear() { c.clear(); }  // keeps the vector's capacity
    };

    void prepare(std::size_t cell_count, Connectivity connectivity);
    void search(const GridMap& map, GridCoord start, GridCoord goal, PlanResult& result);
    void searchOctile(const GridMap& map, GridCoord start, GridCoord goal, PlanResult& result);

    std::unique_ptr<int[]> cost_;          // g(n) (8-conn: its straight part); valid if visited
    std::unique_ptr<int[]> cost_diagonal_; // 8-conn: diagonal part of g(n); valid if visited
    std::vector<std::uint8_t> state_;      // per-cell flags; all zero between plans
    std::vector<int> touched_;             // cells whose state_ must be cleared
    KeyHeap open_now_;                     // keys (h << 32 | index) of entries with f == F
    KeyHeap open_next_;                    // keys of entries with f == F + 2
    OctileHeap open_octile_;               // 8-connected open set
    std::size_t capacity_ = 0;             // cells allocated in cost_ / state_
    std::size_t diagonal_capacity_ = 0;    // cells allocated in cost_diagonal_
};

/// Plan a shortest collision-free path from start to goal (single request).
PlanResult planAStar(const GridMap& map, GridCoord start, GridCoord goal,
                     Connectivity connectivity = Connectivity::Four);

/// True if the path is non-empty, every cell is free and in bounds, and every
/// consecutive pair is an allowed move (isAllowedMove) for the connectivity.
bool isValidGridPath(const GridMap& map, const std::vector<GridCoord>& path,
                     Connectivity connectivity = Connectivity::Four);

/// Exact cost of a path whose consecutive cells are adjacent (straight or diagonal).
GridCost gridPathCost(const std::vector<GridCoord>& path);

/// Sum of Euclidean distances between consecutive cell centers, in meters.
double pathLengthMeters(const GridMap& map, const std::vector<GridCoord>& path);

/// Reference breadth-first search returning only the shortest path cost.
/// Used for validating A* optimality in tests; not used for normal planning.
/// Returns std::nullopt if start/goal are invalid or the goal is unreachable.
std::optional<int> bfsShortestPathCost(const GridMap& map, GridCoord start, GridCoord goal);

/// Reference shortest-path cost for either connectivity, for validation only:
/// BFS for Four, an independent exact Dijkstra search for Eight.
/// Returns std::nullopt if start/goal are invalid or the goal is unreachable.
std::optional<GridCost> referenceShortestPathCost(const GridMap& map, GridCoord start,
                                                  GridCoord goal, Connectivity connectivity);

}  // namespace amr
