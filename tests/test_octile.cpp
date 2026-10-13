// Tests for 8-connected A*: exact octile costs, corner-cutting rule, optimality
// against an independent exact Dijkstra search, and diagonal motion.
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::Connectivity;
using amr::GridCoord;
using amr::GridCost;
using amr::GridMap;
using amr::PlanResult;
using amr::PlanStatus;

namespace {

constexpr double kResolution = 0.5;
constexpr Connectivity kEight = Connectivity::Eight;

GridMap makeMap(const std::vector<std::string>& rows, double resolution = kResolution) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                   resolution, std::move(cells));
}

GridMap makeRandomMap(std::mt19937& rng, int width, int height, double density,
                      double resolution = kResolution) {
    std::bernoulli_distribution occupied(density);
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(width) *
                                    static_cast<std::size_t>(height));
    for (auto& cell : cells) {
        cell = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
    }
    return GridMap(width, height, resolution, std::move(cells));
}

void checkSamePlan(const PlanResult& a, const PlanResult& b) {
    CHECK(a.status == b.status);
    CHECK_EQ(a.path_cost, b.path_cost);
    CHECK_EQ(a.diagonal_moves, b.diagonal_moves);
    CHECK_EQ(a.expanded_nodes, b.expanded_nodes);
    REQUIRE(a.path.size() == b.path.size());
    for (std::size_t i = 0; i < a.path.size(); ++i) {
        CHECK_EQ(a.path[i], b.path[i]);
    }
}

void testExactCostComparison() {
    const auto cmp = [](GridCost a, GridCost b) { return amr::compareGridCost(a, b); };
    CHECK_EQ(cmp({3, 0}, {0, 2}), 1);             // 3 > 2.8284
    CHECK_EQ(cmp({7, 0}, {0, 5}), -1);            // 7 < 7.0711
    CHECK_EQ(cmp({99, 0}, {0, 70}), 1);           // 99 > 98.9949 (99^2 - 2*70^2 = 1)
    CHECK_EQ(cmp({0, 70}, {99, 0}), -1);
    CHECK_EQ(cmp({1, 1}, {2, 0}), 1);             // 2.4142 > 2
    CHECK_EQ(cmp({10, 3}, {14, 0}), 1);           // 14.2426 > 14
    CHECK_EQ(cmp({4, 2}, {4, 2}), 0);
    CHECK_EQ(cmp({5, 1}, {6, 1}), -1);
    // Large values near the limits of the largest supported map.
    CHECK_EQ(cmp({100000000, 0}, {0, 70710678}), 1);    // 1e8 > 99999999.4
    CHECK_EQ(cmp({0, 70710679}, {100000000, 0}), 1);    // 100000000.8 > 1e8

    // Agreement with long double arithmetic wherever the values are not too close.
    std::mt19937 rng(5);
    std::uniform_int_distribution<int> value(0, 20000);
    const long double root2 = std::sqrt(2.0L);
    int checked = 0;
    for (int i = 0; i < 20000; ++i) {
        const GridCost a{value(rng), value(rng)};
        const GridCost b{value(rng), value(rng)};
        const long double diff = (a.straight + a.diagonal * root2) -
                                 (b.straight + b.diagonal * root2);
        if (std::fabs(static_cast<double>(diff)) > 1e-6) {
            CHECK_EQ(cmp(a, b), diff > 0 ? 1 : -1);
            ++checked;
        }
    }
    CHECK(checked > 19000);
}

void testOctileDistance() {
    CHECK(amr::octileDistance({0, 0}, {5, 2}) == (GridCost{3, 2}));
    CHECK(amr::octileDistance({5, 2}, {0, 0}) == (GridCost{3, 2}));
    CHECK(amr::octileDistance({3, 3}, {3, 3}) == (GridCost{0, 0}));
    CHECK(amr::octileDistance({0, 0}, {4, 4}) == (GridCost{0, 4}));
    CHECK(amr::octileDistance({2, 9}, {2, 1}) == (GridCost{8, 0}));
}

void testEmptyMapDiagonalPath() {
    const GridMap map(10, 5, kResolution);
    const PlanResult plan = amr::planAStar(map, {0, 0}, {9, 4}, kEight);
    REQUIRE(plan.success());
    CHECK(plan.cost() == amr::octileDistance({0, 0}, {9, 4}));  // 5 straight + 4 diagonal
    CHECK_EQ(plan.diagonal_moves, 4);
    CHECK_EQ(plan.path_cost, 9);
    CHECK(plan.cost() == amr::gridPathCost(plan.path));
    CHECK(amr::isValidGridPath(map, plan.path, kEight));
    CHECK_NEAR(amr::pathLengthMeters(map, plan.path), (5 + 4 * std::sqrt(2.0)) * kResolution,
               1e-9);
    // The heuristic is exact on an empty map, so A* expands only the path cells.
    CHECK_EQ(plan.expanded_nodes, plan.path.size());
}

void testNoCornerCutting() {
    // Both orthogonal cells blocked: the diagonal squeeze is not allowed at all.
    const GridMap sealed = makeMap({".#", "#."});
    CHECK(amr::planAStar(sealed, {0, 0}, {1, 1}, kEight).status == PlanStatus::NoPath);
    CHECK(!amr::isAllowedMove(sealed, {0, 0}, {1, 1}, kEight));

    // One orthogonal cell blocked: still no diagonal; go around (2 straight moves).
    const GridMap half = makeMap({".#", ".."});
    const PlanResult plan = amr::planAStar(half, {0, 0}, {1, 1}, kEight);
    REQUIRE(plan.success());
    CHECK(plan.cost() == (GridCost{2, 0}));
    CHECK_EQ(plan.path[1], (GridCoord{0, 1}));
    CHECK(!amr::isAllowedMove(half, {0, 0}, {1, 1}, kEight));

    // Both free: the diagonal is used.
    const GridMap open = makeMap({"..", ".."});
    CHECK(amr::planAStar(open, {0, 0}, {1, 1}, kEight).cost() == (GridCost{0, 1}));
}

void testPathValidation() {
    const GridMap map = makeMap({"...", ".#.", "..."});
    CHECK(amr::isValidGridPath(map, {{0, 0}, {1, 0}, {2, 0}}, kEight));
    CHECK(amr::isValidGridPath(map, {{0, 1}, {0, 2}, {1, 2}}, kEight));
    CHECK(!amr::isValidGridPath(map, {{0, 0}, {1, 1}}, kEight));        // into the obstacle
    CHECK(!amr::isValidGridPath(map, {{1, 0}, {0, 1}}, kEight));        // cuts corner (1,1)
    CHECK(!amr::isValidGridPath(map, {{0, 0}, {2, 2}}, kEight));        // jump
    CHECK(!amr::isValidGridPath(map, {{0, 2}, {1, 1}}, kEight));        // into the obstacle
    const GridMap free_map(3, 3, kResolution);
    CHECK(amr::isValidGridPath(free_map, {{0, 0}, {1, 1}, {2, 2}}, kEight));
    CHECK(!amr::isValidGridPath(free_map, {{0, 0}, {1, 1}}));            // diagonal with Four
    CHECK(amr::gridPathCost({{0, 0}, {1, 1}, {2, 1}, {3, 2}}) == (GridCost{1, 2}));
}

void testDeterministicTieBreak() {
    // (0,0) -> (2,1): two optimal routes, "diagonal then right" and "right then
    // diagonal" (cost 1 + sqrt(2) each). Both first moves have the same f; the
    // diagonal one has the lower h (1 < sqrt(2)), so it is expanded first.
    const GridMap map(3, 2, kResolution);
    const PlanResult plan = amr::planAStar(map, {0, 0}, {2, 1}, kEight);
    REQUIRE(plan.success());
    const std::vector<GridCoord> expected{{0, 0}, {1, 1}, {2, 1}};
    REQUIRE(plan.path.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK_EQ(plan.path[i], expected[i]);
    }
    std::mt19937 rng(9);
    const GridMap random_map = makeRandomMap(rng, 40, 40, 0.25);
    GridMap cleared = random_map;
    cleared.setOccupied({0, 0}, false);
    cleared.setOccupied({39, 39}, false);
    const PlanResult first = amr::planAStar(cleared, {0, 0}, {39, 39}, kEight);
    for (int run = 0; run < 3; ++run) {
        checkSamePlan(amr::planAStar(cleared, {0, 0}, {39, 39}, kEight), first);
    }
}

// A* versus the independent exact Dijkstra search, and 8- versus 4-connectivity.
void testAgainstDijkstraAndFourConnected() {
    std::mt19937 rng(2027);
    std::uniform_int_distribution<int> size_dist(1, 45);
    std::uniform_real_distribution<double> density_dist(0.0, 0.45);
    int reachable = 0;
    int unreachable = 0;
    int shorter = 0;
    for (int trial = 0; trial < 500; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        GridMap map = makeRandomMap(rng, width, height, density_dist(rng));
        std::uniform_int_distribution<int> x_dist(0, width - 1);
        std::uniform_int_distribution<int> y_dist(0, height - 1);
        const GridCoord start{x_dist(rng), y_dist(rng)};
        const GridCoord goal{x_dist(rng), y_dist(rng)};
        map.setOccupied(start, false);
        map.setOccupied(goal, false);

        const PlanResult eight = amr::planAStar(map, start, goal, kEight);
        const PlanResult four = amr::planAStar(map, start, goal);
        const auto reference = amr::referenceShortestPathCost(map, start, goal, kEight);
        // No corner cutting means a diagonal can always be replaced by two straight
        // moves, so 8-connectivity reaches exactly the same cells as 4-connectivity.
        CHECK(eight.status == four.status);
        if (reference) {
            ++reachable;
            REQUIRE(eight.success());
            CHECK(eight.cost() == *reference);                       // exactly optimal
            CHECK(eight.cost() == amr::gridPathCost(eight.path));
            CHECK(amr::isValidGridPath(map, eight.path, kEight));
            CHECK_EQ(eight.path.front(), start);
            CHECK_EQ(eight.path.back(), goal);
            CHECK(amr::compareGridCost(eight.cost(), four.cost()) <= 0);  // never longer
            shorter += amr::compareGridCost(eight.cost(), four.cost()) < 0;
        } else {
            ++unreachable;
            CHECK(eight.status == PlanStatus::NoPath);
            CHECK(eight.path.empty());
        }
    }
    CHECK(reachable > 0);
    CHECK(unreachable > 0);
    CHECK(shorter > reachable / 2);  // diagonals usually help
}

void testPlannerReuseAcrossConnectivity() {
    std::mt19937 rng(3);
    std::uniform_int_distribution<int> size_dist(1, 50);
    amr::AStarPlanner planner;
    for (int trial = 0; trial < 200; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        GridMap map = makeRandomMap(rng, width, height, 0.3);
        std::uniform_int_distribution<int> x_dist(0, width - 1);
        std::uniform_int_distribution<int> y_dist(0, height - 1);
        const GridCoord start{x_dist(rng), y_dist(rng)};
        const GridCoord goal{x_dist(rng), y_dist(rng)};
        map.setOccupied(start, false);
        map.setOccupied(goal, false);
        const Connectivity connectivity = trial % 2 ? kEight : Connectivity::Four;
        checkSamePlan(planner.plan(map, start, goal, connectivity),
                      amr::planAStar(map, start, goal, connectivity));
    }
}

void testDiagonalMotion() {
    const GridMap map(6, 6, kResolution);
    const PlanResult plan = amr::planAStar(map, {0, 0}, {5, 5}, kEight);
    REQUIRE(plan.success());
    const amr::MotionConfig config;
    const amr::SimulationResult sim = amr::simulatePath(map, plan.path, config, kEight);
    CHECK(sim.status == amr::NavigationStatus::GoalReached);
    CHECK_NEAR(sim.distance_traveled, 5 * std::sqrt(2.0) * kResolution, 1e-9);
    // A diagonal leg is 0.7071 m: 14 full 0.05 m steps plus one clamped step.
    CHECK_EQ(sim.steps, static_cast<std::size_t>(5 * 15));
    // The same diagonal path is structurally invalid for a 4-connected simulation.
    CHECK(amr::simulatePath(map, plan.path, config).status ==
          amr::NavigationStatus::InvalidPath);
}

void testCornerCuttingIsACollision() {
    // A path that squeezes diagonally between two obstacle cells (not produced by
    // the planner, but the simulator must still catch it).
    const GridMap map = makeMap({".#.", "#..", "..."});
    const std::vector<GridCoord> cutting{{0, 0}, {1, 1}, {2, 2}};
    const amr::SimulationResult sim = amr::simulatePath(map, cutting, amr::MotionConfig{}, kEight);
    CHECK(sim.status == amr::NavigationStatus::Collision);
    REQUIRE(!sim.trajectory.empty());
    // The shared corner (0.5, 0.5) lies sqrt(2) * 0.25 = 0.3536 m along the diagonal:
    // the robot stops before reaching it, within one 0.05 m step.
    const auto& last = sim.trajectory.back().state;
    const double travelled = std::hypot(last.x - 0.25, last.y - 0.25);
    const double corner = std::sqrt(2.0) * 0.25;
    CHECK(travelled < corner);
    CHECK(travelled > corner - 0.05 - 1e-9);
    CHECK_EQ(sim.steps, static_cast<std::size_t>(7));  // 0.35 m, the 8th step would cross it
    // A diagonal through a free 2x2 block is fine.
    const std::vector<GridCoord> free_diagonal{{1, 1}, {2, 2}};
    CHECK(amr::simulatePath(map, free_diagonal, amr::MotionConfig{}, kEight).status ==
          amr::NavigationStatus::GoalReached);
}

// Diagonal moves with a footprint: planning on the inflated map with the corner
// rule must keep the whole disc clear (docs/algorithms.md, section 9).
void testFootprintWithDiagonals() {
    std::mt19937 rng(2028);
    std::uniform_int_distribution<int> size_dist(4, 40);
    std::uniform_real_distribution<double> density_dist(0.0, 0.3);
    std::uniform_real_distribution<double> radius_cells(0.0, 1.5);
    const double resolutions[] = {0.25, 0.5, 1.0};
    int successes = 0;
    int diagonal_moves = 0;
    for (int trial = 0; trial < 300; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        const double res = resolutions[trial % 3];
        const GridMap map = makeRandomMap(rng, width, height, density_dist(rng), res);
        // Every fifth trial is a point robot (radius 0).
        const double radius = trial % 5 == 0 ? 0.0 : radius_cells(rng) * res;
        const GridMap inflated = amr::inflateObstacles(map, radius);
        std::vector<GridCoord> fits;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (inflated.isFree({x, y})) {
                    fits.push_back({x, y});
                }
            }
        }
        if (fits.empty()) {
            continue;
        }
        std::uniform_int_distribution<std::size_t> pick(0, fits.size() - 1);
        const PlanResult plan =
            amr::planWithFootprint(map, inflated, fits[pick(rng)], fits[pick(rng)], kEight);
        if (!plan.success()) {
            continue;
        }
        ++successes;
        diagonal_moves += plan.diagonal_moves;
        for (std::size_t i = 1; i < plan.path.size(); ++i) {
            const amr::WorldPoint a = map.gridToWorld(plan.path[i - 1]);
            const amr::WorldPoint b = map.gridToWorld(plan.path[i]);
            if (radius > 0.0) {
                CHECK(!amr::sweptDiscOverlapsObstacle(map, a, b, radius));
            } else {
                CHECK(!amr::segmentTouchesObstacle(map, a, b));
            }
        }
        amr::MotionConfig config;
        config.robot_radius = radius;
        config.max_time = 5000.0;
        const amr::SimulationResult sim = amr::simulatePath(map, plan.path, config, kEight);
        CHECK(sim.status == amr::NavigationStatus::GoalReached);
        CHECK_EQ(sim.collision_count, static_cast<std::size_t>(0));
    }
    CHECK(successes > 150);
    CHECK(diagonal_moves > 500);  // the sample must really exercise diagonal moves
    std::cout << "    (" << successes << " plans, " << diagonal_moves
              << " diagonal moves checked)\n";
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Exact cost comparison (incl. near-ties and large values)", testExactCostComparison},
        {"Octile distance", testOctileDistance},
        {"Empty map: octile-optimal diagonal path", testEmptyMapDiagonalPath},
        {"No cutting across obstacle corners", testNoCornerCutting},
        {"8-connected path validation", testPathValidation},
        {"Deterministic tie-breaking and reproducibility", testDeterministicTieBreak},
        {"A* = exact Dijkstra; 8-conn never longer than 4-conn (500 maps)",
         testAgainstDijkstraAndFourConnected},
        {"Reused planner matches fresh planner across connectivities", testPlannerReuseAcrossConnectivity},
        {"Diagonal motion in the simulator", testDiagonalMotion},
        {"Corner cutting is caught by the simulator", testCornerCuttingIsACollision},
        {"Footprint + diagonals keep the disc clear (300 maps)", testFootprintWithDiagonals},
    });
}
