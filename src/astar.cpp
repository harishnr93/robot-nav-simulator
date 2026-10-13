#include "astar.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <utility>

namespace amr {

namespace {

constexpr int kStepCost = 1;

/// Neighbor order: up, right, down, left. (Order does not affect the result
/// because the open set has a total ordering; it is fixed for readability.)
constexpr std::array<GridCoord, 4> kNeighborOffsets{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

// Per-cell state byte used by AStarPlanner.
constexpr std::uint8_t kVisited = 1U << 0;   // cost_ holds this cell's best g so far
constexpr std::uint8_t kClosed = 1U << 1;    // cell has been expanded
constexpr int kParentShift = 2;              // bits 2-4: neighbor direction (0-7)
constexpr std::uint8_t kParentMask = 7U << kParentShift;  // that led to this cell

/// 8-connected move order: the 4 straight moves (same order as kNeighborOffsets),
/// then the diagonals. Directions 0-3 are straight, 4-7 diagonal.
constexpr std::array<GridCoord, 8> kOctileOffsets{
    {{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {1, -1}, {1, 1}, {-1, 1}, {-1, -1}}};
constexpr int kFirstDiagonal = 4;

std::uint8_t visitedFrom(int direction) {
    return static_cast<std::uint8_t>(kVisited | (direction << kParentShift));
}

/// Open-set key: ordering by key == ordering by (h, index) == ordering by (h, y, x).
/// h <= width + height and index < width * height both fit in 32 bits.
std::uint64_t openKey(int h, int index) {
    return (static_cast<std::uint64_t>(h) << 32) | static_cast<std::uint32_t>(index);
}

int keyIndex(std::uint64_t key) {
    return static_cast<int>(key & 0xFFFFFFFFU);
}



}  // namespace

int manhattanDistance(GridCoord a, GridCoord b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

GridCost octileDistance(GridCoord a, GridCoord b) {
    const int dx = std::abs(a.x - b.x);
    const int dy = std::abs(a.y - b.y);
    return {std::max(dx, dy) - std::min(dx, dy), std::min(dx, dy)};
}

int compareGridCost(const GridCost& a, const GridCost& b) {
    // sign(ds + dd * sqrt(2)). sqrt(2) is irrational, so the value is zero only
    // when ds == dd == 0; with mixed signs compare ds^2 with 2 * dd^2.
    const std::int64_t ds = a.straight - b.straight;
    const std::int64_t dd = a.diagonal - b.diagonal;
    if (ds == 0 && dd == 0) {
        return 0;
    }
    if (ds >= 0 && dd >= 0) {
        return 1;
    }
    if (ds <= 0 && dd <= 0) {
        return -1;
    }
    const std::int64_t straight_sq = ds * ds;
    const std::int64_t diagonal_sq = 2 * dd * dd;
    if (ds > 0) {  // ds > 0, dd < 0
        return straight_sq > diagonal_sq ? 1 : -1;
    }
    return diagonal_sq > straight_sq ? 1 : -1;  // ds < 0, dd > 0
}

bool isAllowedMove(const GridMap& map, GridCoord from, GridCoord to, Connectivity connectivity) {
    const int dx = to.x - from.x;
    const int dy = to.y - from.y;
    if (!map.isFree(to) || std::max(std::abs(dx), std::abs(dy)) != 1) {
        return false;
    }
    if (dx == 0 || dy == 0) {
        return true;  // straight move
    }
    return connectivity == Connectivity::Eight && map.isFree({from.x + dx, from.y}) &&
           map.isFree({from.x, from.y + dy});
}

bool AStarPlanner::OctileEntryGreater::operator()(const OctileEntry& a,
                                                   const OctileEntry& b) const {
    if (const int c = compareGridCost(a.f, b.f); c != 0) {
        return c > 0;
    }
    if (const int c = compareGridCost(a.h, b.h); c != 0) {
        return c > 0;
    }
    return a.index > b.index;
}

PlanStatus validatePlanRequest(const GridMap& map, GridCoord start, GridCoord goal) {
    if (!map.inBounds(start)) {
        return PlanStatus::StartOutOfBounds;
    }
    if (!map.inBounds(goal)) {
        return PlanStatus::GoalOutOfBounds;
    }
    if (map.isOccupied(start)) {
        return PlanStatus::StartOccupied;
    }
    if (map.isOccupied(goal)) {
        return PlanStatus::GoalOccupied;
    }
    return PlanStatus::Success;
}

void AStarPlanner::prepare(std::size_t cell_count, Connectivity connectivity) {
    // Clear only what the previous search touched; everything else is still zero.
    for (const int index : touched_) {
        state_[static_cast<std::size_t>(index)] = 0;
    }
    touched_.clear();
    open_now_.clear();
    open_next_.clear();
    open_octile_.clear();
    if (connectivity == Connectivity::Eight && cell_count > diagonal_capacity_) {
        cost_diagonal_.reset(new int[cell_count]);  // uninitialized, like cost_
        diagonal_capacity_ = cell_count;
    }
    if (cell_count > capacity_) {
        // cost_ is deliberately left uninitialized: it is only read for cells whose
        // state is kVisited, and those were written first. Untouched memory pages
        // are therefore never faulted in.
        cost_.reset(new int[cell_count]);
        state_.assign(cell_count, 0);
        capacity_ = cell_count;
    }
}

void AStarPlanner::search(const GridMap& map, GridCoord start, GridCoord goal,
                          PlanResult& result) {
    const int width = map.width();
    const int height = map.height();
    const std::uint8_t* occupancy = map.data();
    const int index_offsets[4] = {-width, 1, width, -1};  // same order as kNeighborOffsets
    const int start_index = start.y * width + start.x;
    const int goal_index = goal.y * width + goal.x;

    // Two-bucket open set (see astar.hpp). current_f is F, the smallest f in the
    // open set. Keys are unique (a cell is re-pushed only with a strictly smaller
    // f), so the pop order does not depend on the heap layout.
    int current_f = manhattanDistance(start, goal);
    const auto push = [&](int f, int h, int index) {
        assert(f == current_f || f == current_f + 2);
        (f == current_f ? open_now_ : open_next_).push(openKey(h, index));
    };

    state_[static_cast<std::size_t>(start_index)] = kVisited;
    cost_[start_index] = 0;
    touched_.push_back(start_index);
    push(current_f, current_f, start_index);

    while (true) {
        if (open_now_.empty()) {
            if (open_next_.empty()) {
                break;
            }
            open_now_.swap(open_next_);  // every f == F entry is gone: F += 2
            current_f += 2;
        }
        const int entry_index = keyIndex(open_now_.top());
        open_now_.pop();

        // No decrease-key: a cell may be queued several times. Only the first
        // (cheapest) pop is processed; later, stale entries are skipped here.
        std::uint8_t& state = state_[static_cast<std::size_t>(entry_index)];
        if (state & kClosed) {
            continue;
        }
        state = static_cast<std::uint8_t>(state | kClosed);
        ++result.expanded_nodes;

        if (entry_index == goal_index) {
            result.status = PlanStatus::Success;
            result.path_cost = cost_[entry_index];
            result.path.resize(static_cast<std::size_t>(result.path_cost) + 1);
            int index = goal_index;
            for (auto it = result.path.rbegin(); it != result.path.rend(); ++it) {
                *it = GridCoord{index % width, index / width};
                if (index != start_index) {
                    const int direction = (state_[static_cast<std::size_t>(index)] & kParentMask) >>
                                          kParentShift;
                    index -= index_offsets[direction];
                }
            }
            return;
        }

        const int y = entry_index / width;
        const int x = entry_index - y * width;
        const bool in_bounds[4] = {y > 0, x < width - 1, y < height - 1, x > 0};
        const int tentative_g = cost_[entry_index] + kStepCost;
        for (int direction = 0; direction < 4; ++direction) {
            if (!in_bounds[direction]) {
                continue;
            }
            const int neighbor = entry_index + index_offsets[direction];
            if (occupancy[neighbor] != GridMap::kFree) {
                continue;
            }
            std::uint8_t& neighbor_state = state_[static_cast<std::size_t>(neighbor)];
            if (neighbor_state & kClosed) {
                continue;
            }
            if (!(neighbor_state & kVisited)) {
                touched_.push_back(neighbor);
            } else if (tentative_g >= cost_[neighbor]) {
                continue;  // only a strictly cheaper path replaces the parent
            }
            neighbor_state = visitedFrom(direction);
            cost_[neighbor] = tentative_g;
            const int h = std::abs(x + kNeighborOffsets[direction].x - goal.x) +
                          std::abs(y + kNeighborOffsets[direction].y - goal.y);
            push(tentative_g + h, h, neighbor);
        }
    }
    result.status = PlanStatus::NoPath;
}

void AStarPlanner::searchOctile(const GridMap& map, GridCoord start, GridCoord goal,
                                PlanResult& result) {
    const int width = map.width();
    const int height = map.height();
    const std::uint8_t* occupancy = map.data();
    int index_offsets[8];
    for (int d = 0; d < 8; ++d) {
        index_offsets[d] = kOctileOffsets[d].y * width + kOctileOffsets[d].x;
    }
    const int start_index = start.y * width + start.x;
    const int goal_index = goal.y * width + goal.x;
    const auto free_at = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < width && y < height &&
               occupancy[y * width + x] == GridMap::kFree;
    };

    state_[static_cast<std::size_t>(start_index)] = kVisited;
    cost_[start_index] = 0;
    cost_diagonal_[start_index] = 0;
    touched_.push_back(start_index);
    const GridCost start_h = octileDistance(start, goal);
    open_octile_.push({start_h, start_h, start_index});

    while (!open_octile_.empty()) {
        const int entry_index = open_octile_.top().index;
        open_octile_.pop();
        std::uint8_t& state = state_[static_cast<std::size_t>(entry_index)];
        if (state & kClosed) {
            continue;  // stale duplicate (no decrease-key)
        }
        state = static_cast<std::uint8_t>(state | kClosed);
        ++result.expanded_nodes;

        if (entry_index == goal_index) {
            result.status = PlanStatus::Success;
            result.diagonal_moves = cost_diagonal_[entry_index];
            result.path_cost = cost_[entry_index] + cost_diagonal_[entry_index];
            result.path.resize(static_cast<std::size_t>(result.path_cost) + 1);
            int index = goal_index;
            for (auto it = result.path.rbegin(); it != result.path.rend(); ++it) {
                *it = GridCoord{index % width, index / width};
                if (index != start_index) {
                    const int direction = (state_[static_cast<std::size_t>(index)] & kParentMask) >>
                                          kParentShift;
                    index -= index_offsets[direction];
                }
            }
            return;
        }

        const int y = entry_index / width;
        const int x = entry_index - y * width;
        const GridCost g{cost_[entry_index], cost_diagonal_[entry_index]};
        for (int direction = 0; direction < 8; ++direction) {
            const int dx = kOctileOffsets[direction].x;
            const int dy = kOctileOffsets[direction].y;
            const int nx = x + dx;
            const int ny = y + dy;
            if (!free_at(nx, ny)) {
                continue;
            }
            const bool diagonal = direction >= kFirstDiagonal;
            if (diagonal && !(free_at(x + dx, y) && free_at(x, y + dy))) {
                continue;  // would cut across an obstacle corner
            }
            const int neighbor = entry_index + index_offsets[direction];
            std::uint8_t& neighbor_state = state_[static_cast<std::size_t>(neighbor)];
            if (neighbor_state & kClosed) {
                continue;
            }
            const GridCost tentative{g.straight + (diagonal ? 0 : 1),
                                     g.diagonal + (diagonal ? 1 : 0)};
            if (!(neighbor_state & kVisited)) {
                touched_.push_back(neighbor);
            } else if (compareGridCost(tentative, {cost_[neighbor], cost_diagonal_[neighbor]}) >= 0) {
                continue;  // only a strictly cheaper path replaces the parent
            }
            neighbor_state = visitedFrom(direction);
            cost_[neighbor] = static_cast<int>(tentative.straight);
            cost_diagonal_[neighbor] = static_cast<int>(tentative.diagonal);
            const GridCost h = octileDistance({nx, ny}, goal);
            open_octile_.push({{tentative.straight + h.straight, tentative.diagonal + h.diagonal},
                               h, neighbor});
        }
    }
    result.status = PlanStatus::NoPath;
}

PlanResult AStarPlanner::plan(const GridMap& map, GridCoord start, GridCoord goal,
                              Connectivity connectivity) {
    const auto t_begin = std::chrono::steady_clock::now();

    PlanResult result;
    result.status = validatePlanRequest(map, start, goal);
    if (result.status == PlanStatus::Success) {
        prepare(map.cellCount(), connectivity);
        if (connectivity == Connectivity::Eight) {
            searchOctile(map, start, goal, result);
        } else {
            search(map, start, goal, result);
        }
    }

    const auto t_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_begin).count();
    return result;
}

PlanResult planAStar(const GridMap& map, GridCoord start, GridCoord goal,
                     Connectivity connectivity) {
    AStarPlanner planner;
    return planner.plan(map, start, goal, connectivity);
}

bool isValidGridPath(const GridMap& map, const std::vector<GridCoord>& path,
                     Connectivity connectivity) {
    if (path.empty() || !map.isFree(path.front())) {
        return false;
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!isAllowedMove(map, path[i - 1], path[i], connectivity)) {
            return false;
        }
    }
    return true;
}

GridCost gridPathCost(const std::vector<GridCoord>& path) {
    GridCost cost;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const bool diagonal = path[i].x != path[i - 1].x && path[i].y != path[i - 1].y;
        (diagonal ? cost.diagonal : cost.straight) += 1;
    }
    return cost;
}

double pathLengthMeters(const GridMap& map, const std::vector<GridCoord>& path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const WorldPoint a = map.gridToWorld(path[i - 1]);
        const WorldPoint b = map.gridToWorld(path[i]);
        length += std::hypot(b.x - a.x, b.y - a.y);
    }
    return length;
}

std::optional<int> bfsShortestPathCost(const GridMap& map, GridCoord start, GridCoord goal) {
    if (!map.isFree(start) || !map.isFree(goal)) {
        return std::nullopt;
    }
    std::vector<int> distance(map.cellCount(), -1);
    std::queue<GridCoord> frontier;
    distance[map.index(start)] = 0;
    frontier.push(start);

    while (!frontier.empty()) {
        const GridCoord current = frontier.front();
        frontier.pop();
        const int current_distance = distance[map.index(current)];
        if (current == goal) {
            return current_distance;
        }
        for (const GridCoord& offset : kNeighborOffsets) {
            const GridCoord neighbor{current.x + offset.x, current.y + offset.y};
            if (map.isFree(neighbor) && distance[map.index(neighbor)] < 0) {
                distance[map.index(neighbor)] = current_distance + kStepCost;
                frontier.push(neighbor);
            }
        }
    }
    return std::nullopt;
}

std::optional<GridCost> referenceShortestPathCost(const GridMap& map, GridCoord start,
                                                  GridCoord goal, Connectivity connectivity) {
    if (connectivity == Connectivity::Four) {
        const std::optional<int> steps = bfsShortestPathCost(map, start, goal);
        return steps ? std::optional<GridCost>(GridCost{*steps, 0}) : std::nullopt;
    }
    // Plain Dijkstra (uniform-cost search) with exact costs: deliberately simple
    // and independent of AStarPlanner, for validating it.
    if (!map.isFree(start) || !map.isFree(goal)) {
        return std::nullopt;
    }
    std::vector<GridCost> best(map.cellCount());
    std::vector<bool> reached(map.cellCount(), false);
    std::vector<bool> done(map.cellCount(), false);
    const auto greater = [](const std::pair<GridCost, std::size_t>& a,
                            const std::pair<GridCost, std::size_t>& b) {
        return compareGridCost(a.first, b.first) > 0;
    };
    std::priority_queue<std::pair<GridCost, std::size_t>,
                        std::vector<std::pair<GridCost, std::size_t>>, decltype(greater)>
        frontier(greater);
    reached[map.index(start)] = true;
    frontier.push({GridCost{}, map.index(start)});
    while (!frontier.empty()) {
        const auto [cost, index] = frontier.top();
        frontier.pop();
        if (done[index]) {
            continue;
        }
        done[index] = true;
        const GridCoord current = map.coordFromIndex(index);
        if (current == goal) {
            return cost;
        }
        for (const GridCoord& offset : kOctileOffsets) {
            const GridCoord next{current.x + offset.x, current.y + offset.y};
            if (!isAllowedMove(map, current, next, Connectivity::Eight)) {
                continue;
            }
            const bool diagonal = offset.x != 0 && offset.y != 0;
            const GridCost next_cost{cost.straight + (diagonal ? 0 : 1),
                                     cost.diagonal + (diagonal ? 1 : 0)};
            const std::size_t next_index = map.index(next);
            if (!reached[next_index] || compareGridCost(next_cost, best[next_index]) < 0) {
                reached[next_index] = true;
                best[next_index] = next_cost;
                frontier.push({next_cost, next_index});
            }
        }
    }
    return std::nullopt;
}

}  // namespace amr
