// "Why NO_PATH?": the minimum set of occupied cells ("ghosts") whose removal would
// let the disc robot reach its goal.
//
// Problem. The planner plans on the inflated map: a free cell is blocked when its
// center is closer than the robot radius to an occupied cell or to the map border
// (inflateObstacles(), footprint.hpp). Each cell c therefore has a cover C(c): the
// set of occupied cells that block it (C(c) = {c} for an occupied cell and a point
// robot). Removing a set R of occupied cells frees exactly the cells whose cover lies
// inside R (border-blocked cells are never freed). The goal is reachable after the
// removal iff some path start -> goal uses only freed or free cells, i.e. iff
// R contains the union of the covers along that path. The minimum explanation is
// therefore the path whose cover union has the smallest total weight:
//
//     minimize  w(C(p0) u C(p1) u ... u C(pn))  over grid paths p0 = start .. pn = goal
//
// with w(o) = 1 (fewest cells) or w(o) = -log P(o is a ghost) - ... any positive cost
// (most likely explanation, see ghostWeight()). For an 8-connected diagonal move the
// two side cells must be free as well, so their covers join the union.
//
// This is minimum constraint removal (MCR, Hauser 2014) on a lattice with identical,
// overlapping obstacle footprints: removing one cell may free nothing, because its
// footprint overlaps its neighbours'. MCR is NP-hard in general.
//
// Exact algorithm. Best-first search over labels (cell, S), S = union of covers so
// far, ordered by f = max(w(S), H(cell)):
//   * H(c) is the minimax ("bottleneck") cover weight from c to the goal: every
//     path from c must cross a cell whose cover weighs at least H(c), and the union
//     contains that cover, so f never overestimates; it is consistent, so the first
//     label popped at the goal is optimal.
//   * Dominance: a label (c, S) is discarded if another label at c has S' subset of S
//     (it costs no more and every continuation is at least as cheap).
//   * After the first optimal label, the search continues while f equals the
//     optimum to list every optimal explanation (up to max_solutions); with
//     dominance these are exactly the inclusion-minimal optimal sets reachable.
// The worst case is exponential (labels are sets); limits stop the search and
// report the best lower bound reached.
#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

struct GhostQuery {
    double robot_radius = 0.0;                  ///< m (0 = point robot)
    Connectivity connectivity = Connectivity::Four;
    /// Removal cost per cell (index = map.index(cell)); only occupied cells are read.
    /// Empty: every occupied cell costs 1 (minimum number of cells).
    std::vector<double> weights;
    double max_cost = std::numeric_limits<double>::infinity();  ///< give up above this cost
    std::size_t max_labels = 20'000'000;        ///< give up after creating this many labels
    std::size_t max_solutions = 1;              ///< optimal explanations to list
};

enum class GhostStatus {
    PathExists,    ///< the goal is already reachable: nothing to remove
    Found,         ///< optimal explanation(s) found
    Impossible,    ///< unreachable even with every occupied cell removed (map border, bounds)
    LimitReached,  ///< max_cost or max_labels reached before an explanation was found
};

struct GhostResult {
    GhostStatus status = GhostStatus::Impossible;
    double cost = 0.0;                              ///< total weight of an optimal explanation
    std::vector<std::vector<GridCoord>> solutions;  ///< optimal sets, each sorted (y, x)
    std::vector<GridCoord> path;                    ///< a path that the first solution opens
    double lower_bound = 0.0;    ///< proven lower bound on the optimum (= cost when Found)
    std::size_t labels = 0;      ///< labels created
    std::size_t expanded = 0;    ///< labels expanded
    double time_ms = 0.0;
};

std::string toString(GhostStatus status);

/// Exact minimum-weight ghost set for start -> goal. Start and goal must be in bounds.
GhostResult findGhostCells(const GridMap& map, GridCoord start, GridCoord goal,
                           const GhostQuery& query);

/// Cover of every cell: the occupied cells (map indices, sorted) that block it on the
/// inflated map, and whether the map border blocks it. Exposed for tests.
struct CellCovers {
    std::vector<std::vector<int>> covers;  ///< per cell index
    std::vector<bool> border_blocked;      ///< per cell index
};
CellCovers computeCellCovers(const GridMap& map, double robot_radius);

/// Reference for tests: smallest number of removed cells (at most max_k), found by
/// trying every subset of the occupied cells in order of size and planning on the
/// inflated remainder with planWithFootprint(). Returns -1 if none up to max_k.
/// All minimum subsets are appended to `solutions` when it is not null.
int referenceGhostCount(const GridMap& map, GridCoord start, GridCoord goal, double robot_radius,
                        Connectivity connectivity, int max_k,
                        std::vector<std::vector<GridCoord>>* solutions = nullptr);

/// Removal cost of an occupied cell believed occupied with probability p_occupied:
/// log(p / (1 - p)), the log-odds. Minimizing the sum of log-odds of the removed
/// cells maximizes the probability that exactly those cells are ghosts (cells
/// independent, the others real). Clamped to [kMinGhostWeight, kMaxGhostWeight].
double ghostWeight(double p_occupied);
constexpr double kMinGhostWeight = 1e-3;
constexpr double kMaxGhostWeight = 50.0;

}  // namespace amr
