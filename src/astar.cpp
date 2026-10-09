#include "astar.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <tuple>

namespace amr {

namespace {

constexpr int kStepCost = 1;
constexpr int kUnreached = std::numeric_limits<int>::max();
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

/// Neighbor order: up, right, down, left. (Order does not affect the result
/// because the open set has a total ordering; it is fixed for readability.)
constexpr std::array<GridCoord, 4> kNeighborOffsets{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}}};

struct OpenEntry {
    int f;
    int h;
    int y;
    int x;
};

/// Min-heap ordering: lowest f, then lowest h, then lowest y, then lowest x.
struct OpenEntryGreater {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const {
        return std::tie(a.f, a.h, a.y, a.x) > std::tie(b.f, b.h, b.y, b.x);
    }
};

std::vector<GridCoord> reconstructPath(const GridMap& map,
                                       const std::vector<std::size_t>& parent,
                                       std::size_t goal_index) {
    std::vector<GridCoord> path;
    for (std::size_t i = goal_index; i != kNoParent; i = parent[i]) {
        path.push_back(map.coordFromIndex(i));
    }
    std::reverse(path.begin(), path.end());
    return path;
}

/// Core search. Preconditions: start and goal are in bounds and free.
void searchAStar(const GridMap& map, GridCoord start, GridCoord goal, PlanResult& result) {
    const std::size_t cell_count = map.cellCount();
    std::vector<int> cost_so_far(cell_count, kUnreached);   // g(n)
    std::vector<std::size_t> parent(cell_count, kNoParent);
    std::vector<bool> closed(cell_count, false);
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open;

    const int start_h = manhattanDistance(start, goal);
    cost_so_far[map.index(start)] = 0;
    open.push({start_h, start_h, start.y, start.x});

    while (!open.empty()) {
        const OpenEntry entry = open.top();
        open.pop();
        const GridCoord current{entry.x, entry.y};
        const std::size_t current_index = map.index(current);

        // std::priority_queue has no decrease-key, so the same cell may be queued
        // several times. Only the first (cheapest) pop is processed.
        if (closed[current_index]) {
            continue;
        }
        closed[current_index] = true;
        ++result.expanded_nodes;

        if (current == goal) {
            result.status = PlanStatus::Success;
            result.path_cost = cost_so_far[current_index];
            result.path = reconstructPath(map, parent, current_index);
            return;
        }

        for (const GridCoord& offset : kNeighborOffsets) {
            const GridCoord neighbor{current.x + offset.x, current.y + offset.y};
            if (!map.isFree(neighbor)) {  // rejects out-of-bounds and occupied cells
                continue;
            }
            const std::size_t neighbor_index = map.index(neighbor);
            if (closed[neighbor_index]) {
                continue;
            }
            const int tentative_g = cost_so_far[current_index] + kStepCost;
            if (tentative_g < cost_so_far[neighbor_index]) {
                cost_so_far[neighbor_index] = tentative_g;
                parent[neighbor_index] = current_index;
                const int h = manhattanDistance(neighbor, goal);
                open.push({tentative_g + h, h, neighbor.y, neighbor.x});
            }
        }
    }
    result.status = PlanStatus::NoPath;
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

PlanResult planAStar(const GridMap& map, GridCoord start, GridCoord goal) {
    const auto t_begin = std::chrono::steady_clock::now();

    PlanResult result;
    result.status = validatePlanRequest(map, start, goal);
    if (result.status == PlanStatus::Success) {
        searchAStar(map, start, goal, result);
    }

    const auto t_end = std::chrono::steady_clock::now();
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_begin).count();
    return result;
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
