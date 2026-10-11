#include "astar.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <queue>

namespace amr {

namespace {

constexpr int kStepCost = 1;

/// Neighbor order: up, right, down, left. (Order does not affect the result
/// because the open set has a total ordering; it is fixed for readability.)
constexpr std::array<GridCoord, 4> kNeighborOffsets{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

// Per-cell state byte used by AStarPlanner.
constexpr std::uint8_t kVisited = 1U << 0;   // cost_ holds this cell's best g so far
constexpr std::uint8_t kClosed = 1U << 1;    // cell has been expanded
constexpr int kParentShift = 2;              // bits 2-3: neighbor direction (0-3)
constexpr std::uint8_t kParentMask = 3U << kParentShift;  // that led to this cell

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

void AStarPlanner::prepare(std::size_t cell_count) {
    // Clear only what the previous search touched; everything else is still zero.
    for (const int index : touched_) {
        state_[static_cast<std::size_t>(index)] = 0;
    }
    touched_.clear();
    open_now_.clear();
    open_next_.clear();
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

PlanResult AStarPlanner::plan(const GridMap& map, GridCoord start, GridCoord goal) {
    const auto t_begin = std::chrono::steady_clock::now();

    PlanResult result;
    result.status = validatePlanRequest(map, start, goal);
    if (result.status == PlanStatus::Success) {
        prepare(map.cellCount());
        search(map, start, goal, result);
    }

    const auto t_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_begin).count();
    return result;
}

PlanResult planAStar(const GridMap& map, GridCoord start, GridCoord goal) {
    AStarPlanner planner;
    return planner.plan(map, start, goal);
}

bool isValidGridPath(const GridMap& map, const std::vector<GridCoord>& path) {
    if (path.empty()) {
        return false;
    }
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!map.isFree(path[i])) {
            return false;
        }
        if (i > 0 && manhattanDistance(path[i - 1], path[i]) != 1) {
            return false;
        }
    }
    return true;
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

}  // namespace amr
