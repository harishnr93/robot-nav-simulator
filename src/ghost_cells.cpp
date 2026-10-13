#include "ghost_cells.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>

#include "footprint.hpp"

namespace amr {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

/// Distance from p to the square [x0, x1] x [y0, y1], computed exactly as the
/// footprint check does (footprint.cpp), so covers match inflateObstacles().
double distanceToSquare(WorldPoint p, double x0, double y0, double x1, double y1) {
    const double dx = std::max({x0 - p.x, 0.0, p.x - x1});
    const double dy = std::max({y0 - p.y, 0.0, p.y - y1});
    return std::hypot(dx, dy);
}

/// Sorted-vector union.
std::vector<int> unite(const std::vector<int>& a, const std::vector<int>& b) {
    std::vector<int> out;
    out.reserve(a.size() + b.size());
    std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

/// 64-bit signature for a fast "not a subset" test.
std::uint64_t signature(const std::vector<int>& set) {
    std::uint64_t s = 0;
    for (const int v : set) {
        s |= std::uint64_t{1} << (static_cast<std::uint64_t>(v) * 0x9E3779B97F4A7C15ULL >> 58);
    }
    return s;
}

bool isSubset(const std::vector<int>& small, std::uint64_t small_sig,
              const std::vector<int>& big, std::uint64_t big_sig) {
    if (small.size() > big.size() || (small_sig & ~big_sig) != 0) {
        return false;
    }
    return std::includes(big.begin(), big.end(), small.begin(), small.end());
}

struct Label {
    int cell;
    int parent;  // label index, -1 for the start
    std::vector<int> set;
    std::uint64_t sig;
    double cost;
    bool alive;
};

struct HeapEntry {
    double f;
    int distance;  // grid steps to the goal (Chebyshev or Manhattan)
    int label;
    /// Lowest f first; among equal f, the label nearest the goal (goal-directed
    /// search across the plateaus that unit weights create), then the oldest.
    bool operator>(const HeapEntry& o) const {
        if (f != o.f) {
            return f > o.f;
        }
        if (distance != o.distance) {
            return distance > o.distance;
        }
        return label > o.label;
    }
};

std::vector<GridCoord> sortedCells(const GridMap& map, const std::vector<int>& indices) {
    std::vector<GridCoord> cells;
    cells.reserve(indices.size());
    for (const int i : indices) {
        cells.push_back(map.coordFromIndex(static_cast<std::size_t>(i)));
    }
    return cells;  // map indices are row-major, so already sorted by (y, x)
}

}  // namespace

std::string toString(GhostStatus status) {
    switch (status) {
        case GhostStatus::PathExists:   return "PATH_EXISTS";
        case GhostStatus::Found:        return "FOUND";
        case GhostStatus::Impossible:   return "IMPOSSIBLE";
        case GhostStatus::LimitReached: return "LIMIT_REACHED";
    }
    return "UNKNOWN";
}

double ghostWeight(double p_occupied) {
    const double p = std::clamp(p_occupied, 1e-12, 1.0 - 1e-12);
    return std::clamp(std::log(p / (1.0 - p)), kMinGhostWeight, kMaxGhostWeight);
}

CellCovers computeCellCovers(const GridMap& map, double robot_radius) {
    CellCovers result;
    result.covers.resize(map.cellCount());
    result.border_blocked.assign(map.cellCount(), false);
    if (robot_radius <= 0.0) {
        for (std::size_t i = 0; i < map.cellCount(); ++i) {
            if (map.data()[i] != GridMap::kFree) {
                result.covers[i] = {static_cast<int>(i)};
            }
        }
        return result;
    }
    // Same rule as the footprint check: distance < radius - tolerance.
    const double limit = robot_radius - kFootprintTolerance;
    const double res = map.resolution();
    const double map_w = map.width() * res;
    const double map_h = map.height() * res;
    const int reach = static_cast<int>(std::ceil(robot_radius / res)) + 1;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const GridCoord c{x, y};
            const std::size_t index = map.index(c);
            const WorldPoint p = map.gridToWorld(c);
            result.border_blocked[index] =
                p.x < limit || p.y < limit || map_w - p.x < limit || map_h - p.y < limit;
            std::vector<int>& cover = result.covers[index];
            for (int oy = std::max(0, y - reach); oy <= std::min(map.height() - 1, y + reach); ++oy) {
                for (int ox = std::max(0, x - reach); ox <= std::min(map.width() - 1, x + reach);
                     ++ox) {
                    if (map.isOccupied({ox, oy}) &&
                        distanceToSquare(p, ox * res, oy * res, (ox + 1) * res, (oy + 1) * res) <
                            limit) {
                        cover.push_back(static_cast<int>(map.index({ox, oy})));
                    }
                }
            }
            std::sort(cover.begin(), cover.end());
        }
    }
    return result;
}

GhostResult findGhostCells(const GridMap& map, GridCoord start, GridCoord goal,
                           const GhostQuery& query) {
    const auto t_begin = std::chrono::steady_clock::now();
    GhostResult result;
    const auto finish = [&]() {
        result.time_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t_begin).count();
        return result;
    };
    if (!map.inBounds(start) || !map.inBounds(goal)) {
        return finish();
    }
    const CellCovers covers = computeCellCovers(map, query.robot_radius);
    const auto weight = [&](int obstacle) {
        return query.weights.empty() ? 1.0 : query.weights[static_cast<std::size_t>(obstacle)];
    };
    const auto setWeight = [&](const std::vector<int>& set) {
        double w = 0.0;
        for (const int o : set) {
            w += weight(o);
        }
        return w;
    };
    const std::size_t n = map.cellCount();
    std::vector<double> cover_weight(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        cover_weight[i] = covers.border_blocked[i] ? kInfinity : setWeight(covers.covers[i]);
    }
    const bool eight = query.connectivity == Connectivity::Eight;
    const int dirs = eight ? 8 : 4;
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    // Neighbour q of p in direction d, with the side cells of a diagonal move.
    const auto neighbour = [&](GridCoord p, int d, GridCoord& q, GridCoord& s1, GridCoord& s2) {
        q = {p.x + dx[d], p.y + dy[d]};
        s1 = {p.x + dx[d], p.y};
        s2 = {p.x, p.y + dy[d]};
        return map.inBounds(q);
    };

    // H: bottleneck cover weight from each cell to the goal (moves are symmetric).
    std::vector<double> h(n, kInfinity);
    {
        using Entry = std::pair<double, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        const int g = static_cast<int>(map.index(goal));
        h[static_cast<std::size_t>(g)] = cover_weight[static_cast<std::size_t>(g)];
        open.push({h[static_cast<std::size_t>(g)], g});
        while (!open.empty()) {
            const auto [value, index] = open.top();
            open.pop();
            if (value > h[static_cast<std::size_t>(index)]) {
                continue;
            }
            const GridCoord q = map.coordFromIndex(static_cast<std::size_t>(index));
            for (int d = 0; d < dirs; ++d) {
                GridCoord p, s1, s2;
                if (!neighbour(q, d, p, s1, s2)) {
                    continue;
                }
                // Moving p -> q crosses q (and, diagonally, the side cells).
                double crossing = value;
                if (d >= 4) {
                    crossing = std::max({crossing, cover_weight[map.index(s1)],
                                         cover_weight[map.index(s2)]});
                }
                const double candidate = std::max(crossing, cover_weight[map.index(q)]);
                if (candidate < h[map.index(p)]) {
                    h[map.index(p)] = candidate;
                    open.push({candidate, static_cast<int>(map.index(p))});
                }
            }
        }
    }
    const std::size_t start_index = map.index(start);
    const double start_bound = std::max(h[start_index], cover_weight[start_index]);
    if (!std::isfinite(start_bound)) {
        result.status = GhostStatus::Impossible;  // the border blocks every route
        return finish();
    }
    if (start_bound == 0.0) {
        result.status = GhostStatus::PathExists;
        return finish();
    }

    std::vector<Label> labels;
    std::vector<std::vector<int>> at_cell(n);  // alive labels per cell
    std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> open;
    // Insert a label unless dominated; remove the labels it dominates.
    const auto insert = [&](int cell, int parent, std::vector<int> set, double cost) {
        const std::uint64_t sig = signature(set);
        std::vector<int>& here = at_cell[static_cast<std::size_t>(cell)];
        for (const int other : here) {
            const Label& l = labels[static_cast<std::size_t>(other)];
            if (isSubset(l.set, l.sig, set, sig)) {
                return;
            }
        }
        here.erase(std::remove_if(here.begin(), here.end(),
                                  [&](int other) {
                                      Label& l = labels[static_cast<std::size_t>(other)];
                                      if (isSubset(set, sig, l.set, l.sig)) {
                                          l.alive = false;
                                          return true;
                                      }
                                      return false;
                                  }),
                   here.end());
        const int id = static_cast<int>(labels.size());
        labels.push_back({cell, parent, std::move(set), sig, cost, true});
        here.push_back(id);
        const GridCoord c = map.coordFromIndex(static_cast<std::size_t>(cell));
        const int ddx = std::abs(c.x - goal.x);
        const int ddy = std::abs(c.y - goal.y);
        open.push({std::max(cost, h[static_cast<std::size_t>(cell)]),
                   eight ? std::max(ddx, ddy) : ddx + ddy, id});
    };
    {
        std::vector<int> s0 = covers.covers[start_index];
        const double c0 = setWeight(s0);
        insert(static_cast<int>(start_index), -1, std::move(s0), c0);
    }

    const int goal_index = static_cast<int>(map.index(goal));
    double best = kInfinity;
    while (!open.empty()) {
        const HeapEntry top = open.top();
        open.pop();
        if (!labels[static_cast<std::size_t>(top.label)].alive) {
            continue;
        }
        // A copy: inserting labels below may reallocate `labels`.
        const Label label = labels[static_cast<std::size_t>(top.label)];
        result.lower_bound = std::max(result.lower_bound, top.f);
        if (std::isfinite(best) && top.f > best + 1e-9 * std::max(1.0, best)) {
            break;  // every optimal explanation has been listed
        }
        if (top.f > query.max_cost || labels.size() >= query.max_labels) {
            result.status = GhostStatus::LimitReached;
            return finish();
        }
        if (label.cell == goal_index) {
            if (!std::isfinite(best)) {
                best = label.cost;
                // Path of the first optimal solution.
                for (int l = top.label; l >= 0; l = labels[static_cast<std::size_t>(l)].parent) {
                    result.path.push_back(map.coordFromIndex(
                        static_cast<std::size_t>(labels[static_cast<std::size_t>(l)].cell)));
                }
                std::reverse(result.path.begin(), result.path.end());
            }
            result.solutions.push_back(sortedCells(map, label.set));
            if (result.solutions.size() >= query.max_solutions) {
                break;
            }
            continue;
        }
        ++result.expanded;
        const GridCoord p = map.coordFromIndex(static_cast<std::size_t>(label.cell));
        for (int d = 0; d < dirs; ++d) {
            GridCoord q, s1, s2;
            if (!neighbour(p, d, q, s1, s2) || covers.border_blocked[map.index(q)]) {
                continue;
            }
            std::vector<int> set = unite(label.set, covers.covers[map.index(q)]);
            if (d >= 4) {
                if (covers.border_blocked[map.index(s1)] || covers.border_blocked[map.index(s2)]) {
                    continue;
                }
                set = unite(unite(set, covers.covers[map.index(s1)]), covers.covers[map.index(s2)]);
            }
            const double cost = set.size() == label.set.size() ? label.cost : setWeight(set);
            if (cost > query.max_cost) {
                continue;
            }
            insert(static_cast<int>(map.index(q)), top.label, std::move(set), cost);
        }
        result.labels = labels.size();
    }
    result.labels = labels.size();
    if (std::isfinite(best)) {
        result.status = GhostStatus::Found;
        result.cost = best;
        result.lower_bound = best;
    } else {
        result.status = GhostStatus::Impossible;
    }
    return finish();
}

int referenceGhostCount(const GridMap& map, GridCoord start, GridCoord goal, double robot_radius,
                        Connectivity connectivity, int max_k,
                        std::vector<std::vector<GridCoord>>* solutions) {
    std::vector<GridCoord> occupied;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (map.isOccupied({x, y})) {
                occupied.push_back({x, y});
            }
        }
    }
    const auto feasible = [&](const std::vector<int>& removed) {
        GridMap edited = map;
        for (const int i : removed) {
            edited.setOccupied(occupied[static_cast<std::size_t>(i)], false);
        }
        return planWithFootprint(edited, inflateObstacles(edited, robot_radius), start, goal,
                                 connectivity)
            .success();
    };
    const int m = static_cast<int>(occupied.size());
    for (int k = 0; k <= std::min(max_k, m); ++k) {
        bool any = false;
        std::vector<int> pick(static_cast<std::size_t>(k));
        for (int i = 0; i < k; ++i) {
            pick[static_cast<std::size_t>(i)] = i;
        }
        while (true) {
            if (feasible(pick)) {
                any = true;
                if (solutions == nullptr) {
                    break;
                }
                std::vector<GridCoord> cells;
                for (const int i : pick) {
                    cells.push_back(occupied[static_cast<std::size_t>(i)]);
                }
                solutions->push_back(cells);
            }
            // Next combination in lexicographic order.
            int i = k - 1;
            while (i >= 0 && pick[static_cast<std::size_t>(i)] == m - k + i) {
                --i;
            }
            if (i < 0) {
                break;
            }
            ++pick[static_cast<std::size_t>(i)];
            for (int j = i + 1; j < k; ++j) {
                pick[static_cast<std::size_t>(j)] = pick[static_cast<std::size_t>(j - 1)] + 1;
            }
        }
        if (any) {
            return k;
        }
    }
    return -1;
}

}  // namespace amr
