// Tests for the A* planner (TC01-TC11 planner-side expectations).
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "grid_map.hpp"
#include "test_framework.hpp"

using amr::GridCoord;
using amr::GridMap;
using amr::PlanResult;
using amr::PlanStatus;

namespace {

constexpr double kResolution = 0.5;
constexpr double kTolerance = 1e-9;

/// Build a map from text rows: '.' = free, '#' = occupied. Row 0 is the top.
GridMap makeMap(const std::vector<std::string>& rows) {
    const auto height = static_cast<int>(rows.size());
    const auto width = static_cast<int>(rows.front().size());
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(width, height, kResolution, std::move(cells));
}

bool pathContains(const std::vector<GridCoord>& path, GridCoord cell) {
    for (const GridCoord& c : path) {
        if (c == cell) {
            return true;
        }
    }
    return false;
}

/// Common checks for every successful plan.
void checkSuccessfulPlan(const GridMap& map, const PlanResult& result, GridCoord start,
                         GridCoord goal) {
    REQUIRE(result.success());
    REQUIRE(!result.path.empty());
    CHECK_EQ(result.path.front(), start);
    CHECK_EQ(result.path.back(), goal);
    CHECK(amr::isValidGridPath(map, result.path));
    CHECK_EQ(result.path.size(), static_cast<std::size_t>(result.path_cost) + 1);
    CHECK(result.expanded_nodes >= result.path.size());
    CHECK(result.planning_time_ms >= 0.0);
}

// TC01 - Empty map: path found, shortest, collision-free.
void tc01EmptyMap() {
    const GridMap map(10, 10, kResolution);
    const GridCoord start{0, 0};
    const GridCoord goal{9, 9};
    const PlanResult result = amr::planAStar(map, start, goal);
    checkSuccessfulPlan(map, result, start, goal);
    CHECK_EQ(result.path_cost, amr::manhattanDistance(start, goal));
    CHECK_NEAR(amr::pathLengthMeters(map, result.path), 18 * kResolution, kTolerance);
    // With the (f, h, y, x) tie-break, A* on an empty map heads straight for the
    // goal and expands only the cells on its own path.
    CHECK_EQ(result.expanded_nodes, result.path.size());
}

// TC02 - Single obstacle: path avoids it and stays in bounds.
void tc02SingleObstacle() {
    const GridMap map = makeMap({
        ".....",
        ".....",
        "..#..",
        ".....",
        ".....",
    });
    const GridCoord start{0, 2};
    const GridCoord goal{4, 2};
    const PlanResult result = amr::planAStar(map, start, goal);
    checkSuccessfulPlan(map, result, start, goal);
    CHECK(!pathContains(result.path, {2, 2}));
    for (const GridCoord& c : result.path) {
        CHECK(map.inBounds(c));
    }
    CHECK_EQ(result.path_cost, 6);  // 4 straight + 2 to step around
}

// TC03 - Narrow corridor (planner side): the only route uses both gaps.
void tc03NarrowCorridor() {
    const GridMap map = makeMap({
        ".......",
        "######.",
        ".......",
        ".######",
        ".......",
    });
    const GridCoord start{0, 0};
    const GridCoord goal{0, 4};
    const PlanResult result = amr::planAStar(map, start, goal);
    checkSuccessfulPlan(map, result, start, goal);
    CHECK(pathContains(result.path, {6, 1}));
    CHECK(pathContains(result.path, {0, 3}));
    CHECK_EQ(result.path_cost, 16);
}

// TC04 - Unreachable goal (planner side): failure, no fabricated path.
void tc04UnreachableGoal() {
    const GridMap walled_goal = makeMap({
        ".....",
        ".....",
        ".....",
        "...##",
        "...#.",
    });
    const PlanResult result = amr::planAStar(walled_goal, {0, 0}, {4, 4});
    CHECK(result.status == PlanStatus::NoPath);
    CHECK(result.path.empty());
    CHECK_EQ(result.path_cost, 0);
    CHECK(result.expanded_nodes > 0);
    CHECK(!amr::bfsShortestPathCost(walled_goal, {0, 0}, {4, 4}).has_value());

    // Completely blocked: the start is sealed in on all four sides.
    const GridMap sealed_start = makeMap({
        ".#...",
        "#....",
        ".....",
    });
    const PlanResult sealed = amr::planAStar(sealed_start, {0, 0}, {4, 2});
    CHECK(sealed.status == PlanStatus::NoPath);
    CHECK(sealed.path.empty());
    CHECK_EQ(sealed.expanded_nodes, static_cast<std::size_t>(1));
}

// TC05 - Start equals goal: one waypoint, zero cost.
void tc05StartEqualsGoal() {
    const GridMap map(5, 5, kResolution);
    const PlanResult result = amr::planAStar(map, {2, 3}, {2, 3});
    REQUIRE(result.success());
    REQUIRE(result.path.size() == 1);
    CHECK_EQ(result.path.front(), (GridCoord{2, 3}));
    CHECK_EQ(result.path_cost, 0);
    CHECK_NEAR(amr::pathLengthMeters(map, result.path), 0.0, kTolerance);
}

// TC06 - Invalid (out-of-bounds) start.
void tc06InvalidStart() {
    const GridMap map(5, 4, kResolution);
    for (const GridCoord start : {GridCoord{-1, 0}, GridCoord{5, 0}, GridCoord{0, 4},
                                  GridCoord{0, -3}}) {
        const PlanResult result = amr::planAStar(map, start, {1, 1});
        CHECK(result.status == PlanStatus::StartOutOfBounds);
        CHECK(result.path.empty());
        CHECK_EQ(result.expanded_nodes, static_cast<std::size_t>(0));
    }
}

// TC07 - Invalid (out-of-bounds) goal.
void tc07InvalidGoal() {
    const GridMap map(5, 4, kResolution);
    for (const GridCoord goal : {GridCoord{5, 3}, GridCoord{-1, 2}, GridCoord{2, 4}}) {
        const PlanResult result = amr::planAStar(map, {0, 0}, goal);
        CHECK(result.status == PlanStatus::GoalOutOfBounds);
        CHECK(result.path.empty());
    }
}

// TC08 - Blocked start or goal.
void tc08BlockedStartOrGoal() {
    const GridMap map = makeMap({
        "#....",
        ".....",
        "....#",
    });
    const PlanResult blocked_start = amr::planAStar(map, {0, 0}, {2, 1});
    CHECK(blocked_start.status == PlanStatus::StartOccupied);
    CHECK(blocked_start.path.empty());
    const PlanResult blocked_goal = amr::planAStar(map, {2, 1}, {4, 2});
    CHECK(blocked_goal.status == PlanStatus::GoalOccupied);
    CHECK(blocked_goal.path.empty());
}

// TC09 - Multiple equivalent routes: any optimal route is valid; the chosen
// route is fixed by the documented tie-break (lowest f, h, y, x).
void tc09MultipleEquivalentRoutes() {
    const GridMap map = makeMap({
        "...",
        ".#.",
        "...",
    });
    const GridCoord start{0, 0};
    const GridCoord goal{2, 2};
    const PlanResult result = amr::planAStar(map, start, goal);
    checkSuccessfulPlan(map, result, start, goal);
    CHECK_EQ(result.path_cost, 4);
    const std::vector<GridCoord> expected{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
    REQUIRE(result.path.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK_EQ(result.path[i], expected[i]);
    }
}

GridMap makeRandomMap(std::mt19937& rng, int width, int height, double density) {
    std::bernoulli_distribution occupied(density);
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(width) *
                                    static_cast<std::size_t>(height));
    for (auto& cell : cells) {
        cell = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
    }
    return GridMap(width, height, kResolution, std::move(cells));
}

// TC10 - Reproducibility: identical inputs give identical outputs.
void tc10Reproducibility() {
    std::mt19937 rng(1234);
    const GridMap map = makeRandomMap(rng, 30, 30, 0.25);
    GridMap cleared = map;
    cleared.setOccupied({0, 0}, false);
    cleared.setOccupied({29, 29}, false);

    const PlanResult first = amr::planAStar(cleared, {0, 0}, {29, 29});
    for (int run = 0; run < 5; ++run) {
        const PlanResult again = amr::planAStar(cleared, {0, 0}, {29, 29});
        CHECK(again.status == first.status);
        CHECK_EQ(again.path_cost, first.path_cost);
        CHECK_EQ(again.expanded_nodes, first.expanded_nodes);
        REQUIRE(again.path.size() == first.path.size());
        for (std::size_t i = 0; i < first.path.size(); ++i) {
            CHECK_EQ(again.path[i], first.path[i]);
        }
    }
}

// TC11 - A* versus BFS on many random maps: equal optimal costs, and A* fails
// exactly when BFS finds no path.
void tc11AStarMatchesBfs() {
    constexpr int kTrials = 500;
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> size_dist(1, 40);
    std::uniform_real_distribution<double> density_dist(0.0, 0.45);
    int reachable = 0;
    int unreachable = 0;

    for (int trial = 0; trial < kTrials; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        GridMap map = makeRandomMap(rng, width, height, density_dist(rng));
        std::uniform_int_distribution<int> x_dist(0, width - 1);
        std::uniform_int_distribution<int> y_dist(0, height - 1);
        const GridCoord start{x_dist(rng), y_dist(rng)};
        const GridCoord goal{x_dist(rng), y_dist(rng)};
        map.setOccupied(start, false);
        map.setOccupied(goal, false);

        const PlanResult result = amr::planAStar(map, start, goal);
        const auto reference = amr::bfsShortestPathCost(map, start, goal);
        if (reference) {
            ++reachable;
            checkSuccessfulPlan(map, result, start, goal);
            CHECK_EQ(result.path_cost, *reference);
        } else {
            ++unreachable;
            CHECK(result.status == PlanStatus::NoPath);
            CHECK(result.path.empty());
        }
    }
    // Make sure the random sample exercised both outcomes.
    CHECK(reachable > 0);
    CHECK(unreachable > 0);
}

// Maps of different dimensions, including single-row and single-column maps.
void testNonSquareMaps() {
    const GridMap row(25, 1, kResolution);
    const PlanResult along_row = amr::planAStar(row, {0, 0}, {24, 0});
    checkSuccessfulPlan(row, along_row, {0, 0}, {24, 0});
    CHECK_EQ(along_row.path_cost, 24);

    const GridMap column(1, 17, kResolution);
    const PlanResult along_column = amr::planAStar(column, {0, 16}, {0, 0});
    checkSuccessfulPlan(column, along_column, {0, 16}, {0, 0});
    CHECK_EQ(along_column.path_cost, 16);

    const GridMap wide(60, 7, kResolution);
    const PlanResult across = amr::planAStar(wide, {59, 0}, {0, 6});
    checkSuccessfulPlan(wide, across, {59, 0}, {0, 6});
    CHECK_EQ(across.path_cost, 65);
}

void testPathValidationHelpers() {
    const GridMap map = makeMap({
        "...",
        ".#.",
        "...",
    });
    CHECK(!amr::isValidGridPath(map, {}));
    CHECK(amr::isValidGridPath(map, {{0, 0}}));
    CHECK(amr::isValidGridPath(map, {{0, 0}, {1, 0}, {2, 0}}));
    CHECK(!amr::isValidGridPath(map, {{0, 0}, {1, 1}}));          // occupied cell
    CHECK(!amr::isValidGridPath(map, {{0, 0}, {2, 0}}));          // skips a cell
    CHECK(!amr::isValidGridPath(map, {{0, 2}, {1, 1}, {2, 0}}));  // diagonal moves
    CHECK(!amr::isValidGridPath(map, {{2, 0}, {3, 0}}));          // out of bounds
    CHECK_NEAR(amr::pathLengthMeters(map, {{0, 0}, {1, 0}, {2, 0}, {2, 1}}),
               3 * kResolution, kTolerance);
}

}  // namespace

int main() {
    return testfw::runAll({
        {"TC01 Empty map: shortest collision-free path", tc01EmptyMap},
        {"TC02 Single obstacle is avoided", tc02SingleObstacle},
        {"TC03 Narrow corridor is found", tc03NarrowCorridor},
        {"TC04 Unreachable goal returns NO_PATH", tc04UnreachableGoal},
        {"TC05 Start equals goal", tc05StartEqualsGoal},
        {"TC06 Invalid start position", tc06InvalidStart},
        {"TC07 Invalid goal position", tc07InvalidGoal},
        {"TC08 Blocked start or goal", tc08BlockedStartOrGoal},
        {"TC09 Multiple equivalent routes", tc09MultipleEquivalentRoutes},
        {"TC10 Reproducibility", tc10Reproducibility},
        {"TC11 A* cost equals BFS cost (500 random maps)", tc11AStarMatchesBfs},
        {"Maps of different dimensions", testNonSquareMaps},
        {"Path validation and length helpers", testPathValidationHelpers},
    });
}
