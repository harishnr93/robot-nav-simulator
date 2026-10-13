// Tests for the minimum ghost-cell explanation of NO_PATH: cell covers against the
// planner's inflation, the exact solver against brute force (count, every optimal
// set, weighted optimum), and the returned path.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "footprint.hpp"
#include "ghost_cells.hpp"
#include "grid_map.hpp"
#include "test_framework.hpp"

using amr::Connectivity;
using amr::GhostQuery;
using amr::GhostStatus;
using amr::GridCoord;
using amr::GridMap;

namespace {

GridMap makeMap(const std::vector<std::string>& rows, double resolution = 0.5) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                   resolution, std::move(cells));
}

GridMap randomMap(std::mt19937& rng, int w, int h, double density, double resolution,
                  GridCoord start, GridCoord goal) {
    std::bernoulli_distribution occupied(density);
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(w * h));
    for (auto& c : cells) {
        c = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
    }
    cells[static_cast<std::size_t>(start.y * w + start.x)] = GridMap::kFree;
    cells[static_cast<std::size_t>(goal.y * w + goal.x)] = GridMap::kFree;
    return GridMap(w, h, resolution, std::move(cells));
}

/// The map with the given cells made free.
GridMap without(const GridMap& map, const std::vector<GridCoord>& removed) {
    GridMap edited = map;
    for (const GridCoord& c : removed) {
        edited.setOccupied(c, false);
    }
    return edited;
}

bool reachable(const GridMap& map, GridCoord start, GridCoord goal, double radius,
               Connectivity connectivity) {
    return amr::planWithFootprint(map, amr::inflateObstacles(map, radius), start, goal,
                                  connectivity)
        .success();
}

/// Covers reproduce inflateObstacles(): on the map itself and after removing any
/// subset R, a cell is blocked iff it is border-blocked or its cover is not inside R.
void testCoversMatchInflation() {
    std::mt19937 rng(11);
    int checked = 0;
    for (int trial = 0; trial < 150; ++trial) {
        const double res = trial % 2 == 0 ? 0.5 : 0.25;
        const double radius = std::vector<double>{0.0, 0.2, 0.25, 0.3, 0.5, 0.6, 0.75}[trial % 7];
        const GridMap map = randomMap(rng, 12, 10, 0.25, res, {0, 0}, {11, 9});
        const amr::CellCovers covers = amr::computeCellCovers(map, radius);
        std::bernoulli_distribution remove(0.4);
        std::vector<GridCoord> removed;
        std::vector<bool> is_removed(map.cellCount(), false);
        for (std::size_t i = 0; i < map.cellCount(); ++i) {
            if (map.data()[i] != GridMap::kFree && remove(rng)) {
                removed.push_back(map.coordFromIndex(i));
                is_removed[i] = true;
            }
        }
        for (const bool after_removal : {false, true}) {
            const GridMap edited = after_removal ? without(map, removed) : map;
            const GridMap inflated = amr::inflateObstacles(edited, radius);
            for (std::size_t i = 0; i < map.cellCount(); ++i) {
                bool blocked = covers.border_blocked[i] && radius > 0.0;
                for (const int o : covers.covers[i]) {
                    blocked = blocked || !(after_removal && is_removed[static_cast<std::size_t>(o)]);
                }
                CHECK_EQ(inflated.data()[i] != GridMap::kFree, blocked);
                ++checked;
            }
        }
    }
    CHECK(checked > 30000);
}

/// The minimum number of ghost cells equals brute force (planWithFootprint on every
/// subset, up to 3 cells) on 1500 random maps: 4- and 8-connected, point and disc
/// robots, two resolutions.
void testMinimumCountAgainstBruteForce() {
    std::mt19937 rng(7);
    int found = 0;
    int larger = 0;
    for (int trial = 0; trial < 1500; ++trial) {
        const int w = 6 + static_cast<int>(rng() % 6);
        const int h = 6 + static_cast<int>(rng() % 6);
        const double density = 0.04 + 0.16 * static_cast<double>(rng() % 100) / 100.0;
        const double res = trial % 3 == 0 ? 0.25 : 0.5;
        const double radius = std::vector<double>{0.0, 0.3, 0.6}[trial % 3 == 0 ? 1 : trial % 3];
        const Connectivity connectivity = trial % 2 == 0 ? Connectivity::Four : Connectivity::Eight;
        const GridCoord start{1, 1};
        const GridCoord goal{w - 2, h - 2};
        const GridMap map = randomMap(rng, w, h, density, res, start, goal);
        GhostQuery query;
        query.robot_radius = radius;
        query.connectivity = connectivity;
        const auto result = amr::findGhostCells(map, start, goal, query);
        const int reference = amr::referenceGhostCount(map, start, goal, radius, connectivity, 3);
        int solver = -1;
        if (result.status == GhostStatus::PathExists) {
            solver = 0;
        } else if (result.status == GhostStatus::Found) {
            solver = static_cast<int>(std::lround(result.cost));
            ++found;
            if (solver > 3) {
                ++larger;
                solver = -1;  // beyond the brute force's reach
            }
        }
        CHECK_EQ(solver, reference);
        if (result.status == GhostStatus::Found) {
            // The explanation really opens a path, and it is the one returned.
            const GridMap opened = without(map, result.solutions.front());
            CHECK(reachable(opened, start, goal, radius, connectivity));
            CHECK(amr::isValidGridPath(amr::inflateObstacles(opened, radius), result.path,
                                       connectivity));
            CHECK(result.path.front() == start);
            CHECK(result.path.back() == goal);
            CHECK_NEAR(result.lower_bound, result.cost, 1e-12);
        }
    }
    std::cout << "    " << found << " maps needed removals (" << larger << " needed more than 3)\n";
    CHECK(found > 700);
}

/// Every optimal set is listed: the solver's sets equal the brute force's sets.
void testAllOptimalExplanations() {
    std::mt19937 rng(23);
    int compared = 0;
    for (int trial = 0; trial < 600; ++trial) {
        const double radius = trial % 2 == 0 ? 0.3 : 0.0;
        const Connectivity connectivity = trial % 3 == 0 ? Connectivity::Eight : Connectivity::Four;
        // Densities where both robots often need one or two removals.
        const GridMap map = randomMap(rng, 9, 8, radius > 0.0 ? 0.08 : 0.35, 0.5, {1, 1}, {7, 6});
        GhostQuery query;
        query.robot_radius = radius;
        query.connectivity = connectivity;
        query.max_solutions = 100000;
        const auto result = amr::findGhostCells(map, {1, 1}, {7, 6}, query);
        if (result.status != GhostStatus::Found || result.cost > 2.5) {
            continue;
        }
        std::vector<std::vector<GridCoord>> reference;
        amr::referenceGhostCount(map, {1, 1}, {7, 6}, radius, connectivity, 2, &reference);
        const auto key = [&](const std::vector<GridCoord>& set) {
            std::vector<std::size_t> k;
            for (const auto& c : set) {
                k.push_back(map.index(c));
            }
            std::sort(k.begin(), k.end());
            return k;
        };
        std::vector<std::vector<std::size_t>> mine;
        std::vector<std::vector<std::size_t>> theirs;
        for (const auto& s : result.solutions) {
            mine.push_back(key(s));
        }
        for (const auto& s : reference) {
            theirs.push_back(key(s));
        }
        std::sort(mine.begin(), mine.end());
        std::sort(theirs.begin(), theirs.end());
        CHECK(mine == theirs);
        ++compared;
    }
    std::cout << "    compared " << compared << " maps\n";
    CHECK(compared > 200);
}

/// Weighted optimum against exhaustive search over every subset of the occupied
/// cells (small maps, removals of any size), with random positive or unit weights.
void testWeightedOptimum() {
    std::mt19937 rng(31);
    std::uniform_real_distribution<double> weight(0.1, 5.0);
    int compared = 0;
    for (int trial = 0; trial < 400; ++trial) {
        const double radius = std::vector<double>{0.0, 0.3, 0.5}[trial % 3];
        const Connectivity connectivity = trial % 2 == 0 ? Connectivity::Four : Connectivity::Eight;
        GridMap map = randomMap(rng, 7, 6, 0.3, 0.5, {1, 1}, {5, 4});
        std::vector<GridCoord> occupied;
        for (std::size_t i = 0; i < map.cellCount(); ++i) {
            if (map.data()[i] != GridMap::kFree) {
                occupied.push_back(map.coordFromIndex(i));
            }
        }
        if (occupied.size() > 13) {
            continue;  // keep the exhaustive search small
        }
        GhostQuery query;
        query.robot_radius = radius;
        query.connectivity = connectivity;
        query.weights.assign(map.cellCount(), 0.0);
        // Half the trials with unit weights: the exact minimum count, of any size.
        for (const auto& c : occupied) {
            query.weights[map.index(c)] = trial % 4 < 2 ? 1.0 : weight(rng);
        }
        double best = std::numeric_limits<double>::infinity();
        for (std::uint32_t mask = 0; mask < (1u << occupied.size()); ++mask) {
            std::vector<GridCoord> removed;
            double w = 0.0;
            for (std::size_t b = 0; b < occupied.size(); ++b) {
                if ((mask >> b) & 1u) {
                    removed.push_back(occupied[b]);
                    w += query.weights[map.index(occupied[b])];
                }
            }
            if (w < best && reachable(without(map, removed), {1, 1}, {5, 4}, radius, connectivity)) {
                best = w;
            }
        }
        const auto result = amr::findGhostCells(map, {1, 1}, {5, 4}, query);
        if (!std::isfinite(best)) {
            CHECK(result.status == GhostStatus::Impossible);
            continue;
        }
        if (best == 0.0) {
            CHECK(result.status == GhostStatus::PathExists);
            continue;
        }
        REQUIRE(result.status == GhostStatus::Found);
        CHECK_NEAR(result.cost, best, 1e-9);
        double w = 0.0;
        for (const auto& c : result.solutions.front()) {
            w += query.weights[map.index(c)];
        }
        CHECK_NEAR(w, best, 1e-9);
        ++compared;
    }
    CHECK(compared > 150);
}

/// One cell that frees nothing on its own: in a single-lane corridor two adjacent
/// obstacles' footprints overlap across the lane, so removing either alone leaves
/// the lane blocked by the other (the reason this is not a shortest-path problem).
void testOverlappingFootprints() {
    // 0.5 m cells, robot radius 0.6 m: walls in rows 0 and 4 leave only row 2 for
    // the robot's center; obstacles (5, 1) and (6, 1) both reach cells (5, 2), (6, 2).
    const GridMap map = makeMap({"############", ".....##.....", "............",
                                 "............", "############"});
    GhostQuery query;
    query.robot_radius = 0.6;
    query.max_solutions = 10;
    const GridCoord start{1, 2};
    const GridCoord goal{10, 2};
    const auto result = amr::findGhostCells(map, start, goal, query);
    REQUIRE(result.status == GhostStatus::Found);
    REQUIRE(result.solutions.size() == 1u);
    CHECK((result.solutions.front() == std::vector<GridCoord>{{5, 1}, {6, 1}}));
    for (const auto& cell : result.solutions.front()) {
        CHECK(!reachable(without(map, {cell}), start, goal, 0.6, Connectivity::Four));
    }
    CHECK(reachable(without(map, result.solutions.front()), start, goal, 0.6, Connectivity::Four));
}

void testStatuses() {
    const GridMap open = makeMap({"......", "......", "......"});
    GhostQuery query;
    CHECK(amr::findGhostCells(open, {0, 0}, {5, 2}, query).status == GhostStatus::PathExists);
    // A wide robot cannot fit next to the map border anywhere: nothing to remove helps.
    query.robot_radius = 1.0;
    CHECK(amr::findGhostCells(open, {0, 0}, {5, 2}, query).status == GhostStatus::Impossible);
    CHECK(amr::findGhostCells(open, {-1, 0}, {5, 2}, query).status == GhostStatus::Impossible);
    // A full wall: one cell must go for a point robot.
    const GridMap wall = makeMap({"..#..", "..#..", "..#.."});
    query.robot_radius = 0.0;
    auto result = amr::findGhostCells(wall, {0, 1}, {4, 1}, query);
    REQUIRE(result.status == GhostStatus::Found);
    CHECK_NEAR(result.cost, 1.0, 1e-12);
    // Weights pick the likely ghost: the middle cell is cheapest.
    query.weights.assign(wall.cellCount(), 5.0);
    query.weights[wall.index({2, 1})] = 0.5;
    query.max_solutions = 10;
    result = amr::findGhostCells(wall, {0, 0}, {4, 0}, query);
    REQUIRE(result.status == GhostStatus::Found);
    REQUIRE(result.solutions.size() == 1u);
    CHECK(result.solutions.front().front() == (GridCoord{2, 1}));
    // A cost limit below the optimum stops the search with a lower bound.
    query.weights.clear();
    query.max_cost = 0.5;
    result = amr::findGhostCells(wall, {0, 1}, {4, 1}, query);
    CHECK(result.status == GhostStatus::LimitReached);
    CHECK(result.lower_bound <= 1.0);
    // The start itself can be the ghost.
    const GridMap on_start = makeMap({"#...."});
    query.max_cost = std::numeric_limits<double>::infinity();
    result = amr::findGhostCells(on_start, {0, 0}, {4, 0}, query);
    REQUIRE(result.status == GhostStatus::Found);
    CHECK(result.solutions.front().front() == (GridCoord{0, 0}));
}

void testGhostWeight() {
    CHECK_NEAR(amr::ghostWeight(0.9), std::log(9.0), 1e-12);
    CHECK_NEAR(amr::ghostWeight(0.5), amr::kMinGhostWeight, 1e-15);    // undecided: nearly free
    CHECK(amr::ghostWeight(1.0) > 20.0);                                // certain: very costly
    CHECK(amr::ghostWeight(1.0) <= amr::kMaxGhostWeight);
    CHECK(amr::ghostWeight(0.99) > amr::ghostWeight(0.8));
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Covers reproduce the planner's inflation", testCoversMatchInflation},
        {"Minimum count equals brute force (1500 maps)", testMinimumCountAgainstBruteForce},
        {"Every optimal explanation is listed", testAllOptimalExplanations},
        {"Weighted and unit optimum equal exhaustive search", testWeightedOptimum},
        {"Overlapping footprints: one cell frees nothing", testOverlappingFootprints},
        {"Statuses and edge cases", testStatuses},
        {"Removal cost from occupancy probability", testGhostWeight},
    });
}
