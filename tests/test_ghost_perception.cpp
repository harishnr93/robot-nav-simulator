// Tests for the perception loop around NO_PATH: the beam and ghost model, the
// occupancy update, the ghost evidence, and the recovery strategies on a doorway
// blocked by a view-dependent ghost (or by a real obstacle).
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "footprint.hpp"
#include "ghost_cells.hpp"
#include "ghost_perception.hpp"
#include "grid_map.hpp"
#include "test_framework.hpp"

using amr::GhostSource;
using amr::GhostWorld;
using amr::GridCoord;
using amr::GridMap;
using amr::OccupancyBelief;
using amr::RecoveryOutcome;
using amr::RecoveryStrategy;
using amr::SensorModel;
using amr::WorldPoint;

namespace {

constexpr double kPi = 3.14159265358979323846;

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

SensorModel noiseless() {
    SensorModel sensor;
    sensor.miss_probability = 0.0;
    sensor.spurious_probability = 0.0;
    return sensor;
}

void testBeamTrace() {
    const GhostWorld world{makeMap({"..........", "......#...", ".........."}), {}};
    std::mt19937 rng(1);
    const WorldPoint origin = world.truth.gridToWorld({1, 1});
    const auto trace = amr::traceBeam(world, origin, 0.0, 10.0, noiseless(), rng);
    CHECK_EQ(trace.hit, static_cast<int>(world.truth.index({6, 1})));
    REQUIRE(trace.passed.size() == 4u);  // cells 2..5 of row 1
    CHECK_EQ(trace.passed.front(), static_cast<int>(world.truth.index({2, 1})));
    CHECK_EQ(trace.passed.back(), static_cast<int>(world.truth.index({5, 1})));
    // Out of range: no return.
    CHECK_EQ(amr::traceBeam(world, origin, 0.0, 1.0, noiseless(), rng).hit, -1);
    // Leaving the map: no return.
    CHECK_EQ(amr::traceBeam(world, origin, kPi, 10.0, noiseless(), rng).hit, -1);
}

void testGhostSource() {
    GhostWorld world{makeMap({"..........", "..........", ".........."}), {}};
    world.ghosts.push_back({{5, 1}, 0.0, 0.5, 0.7});  // active for beams heading +x +-0.5 rad
    std::mt19937 rng(2);
    const SensorModel sensor = noiseless();
    const WorldPoint west = world.truth.gridToWorld({1, 1});
    const WorldPoint north = world.truth.gridToWorld({5, 0});
    int hits = 0;
    const int beams = 4000;
    for (int i = 0; i < beams; ++i) {
        hits += amr::traceBeam(world, west, 0.0, 10.0, sensor, rng).hit ==
                        static_cast<int>(world.truth.index({5, 1}))
                    ? 1
                    : 0;
    }
    CHECK_NEAR(static_cast<double>(hits) / beams, 0.7, 0.03);  // in sector: hit probability
    for (int i = 0; i < 200; ++i) {
        // From the north the beam heads +y: out of the sector, it always passes.
        const auto trace = amr::traceBeam(world, north, kPi / 2, 10.0, sensor, rng);
        CHECK_EQ(trace.hit, -1);
    }
}

void testSensorNoise() {
    const GhostWorld world{makeMap({"..........", "........#.", ".........."}), {}};
    SensorModel sensor;
    sensor.miss_probability = 0.1;
    sensor.spurious_probability = 0.2;
    std::mt19937 rng(3);
    const WorldPoint origin = world.truth.gridToWorld({1, 1});
    int real = 0, early = 0, none = 0;
    const int beams = 20000;
    for (int i = 0; i < beams; ++i) {
        const int hit = amr::traceBeam(world, origin, 0.0, 10.0, sensor, rng).hit;
        if (hit == static_cast<int>(world.truth.index({8, 1}))) {
            ++real;
        } else if (hit >= 0) {
            ++early;
        } else {
            ++none;
        }
    }
    // A spurious range is drawn uniformly over the 10 m range in 20 % of the beams and
    // registers on the next free cell entered; the last one before the wall is
    // entered 2.75 m out: 0.2 * 2.75 / 10 = 5.5 % early returns.
    CHECK_NEAR(static_cast<double>(early) / beams, 0.2 * 2.75 / 10.0, 0.008);
    CHECK_NEAR(static_cast<double>(none) / (beams - early), 0.1, 0.01);  // misses
    CHECK(real > 0);
}

void testOccupancyUpdate() {
    OccupancyBelief belief(4, 1, 0.5);
    amr::BeamTrace trace;
    trace.passed = {0, 1};
    trace.hit = 2;
    trace.angle = 0.25;
    amr::integrateBeam(belief, trace);
    CHECK_NEAR(belief.log_odds[0], amr::kLogOddsMiss, 1e-12);
    CHECK_NEAR(belief.log_odds[2], amr::kLogOddsHit, 1e-12);
    CHECK_EQ(belief.hits[2], 1);
    CHECK_EQ(belief.passes[1], 1);
    REQUIRE(belief.hit_bearings[2].size() == 1u);
    CHECK_NEAR(belief.hit_bearings[2][0], 0.25, 1e-6);
    CHECK(belief.map().isOccupied({2, 0}));
    CHECK(belief.map().isFree({0, 0}));
    CHECK(belief.map().isFree({3, 0}));  // unknown counts as free
    for (int i = 0; i < 100; ++i) {
        amr::integrateBeam(belief, trace);
    }
    CHECK_NEAR(belief.log_odds[2], amr::kLogOddsLimit, 1e-12);
    CHECK_NEAR(belief.probability(2), 1.0 / (1.0 + std::exp(-amr::kLogOddsLimit)), 1e-12);
}

void testGhostEvidence() {
    // Hits barely separate real from ghost; one pass weighs as much as ~19 hits (plus
    // the prior in favour of real obstacles).
    const double per_hit = std::log((1.0 - amr::kRealMiss) / amr::kGhostHitProbability);
    const double per_pass = std::log(amr::kSeeThroughReal);
    CHECK_NEAR(amr::ghostLogLikelihoodRatio(0, 0), amr::kGhostPriorLogRatio, 1e-12);
    CHECK_NEAR(amr::ghostLogLikelihoodRatio(10, 2),
               amr::kGhostPriorLogRatio + 10 * per_hit + 2 * per_pass, 1e-12);
    CHECK(amr::ghostLogLikelihoodRatio(30, 0) > 0.0);   // never seen through: real
    CHECK(amr::ghostLogLikelihoodRatio(2, 1) <= 0.0);   // seen through once, few hits: ghost
    CHECK(amr::ghostLogLikelihoodRatio(10, 2) <= 0.0);  // seen through twice: ghost
    CHECK(amr::ghostLogLikelihoodRatio(10, 1) > 0.0);   // once, against many hits: not yet
    CHECK_NEAR(amr::ghostEvidenceWeight(30, 5), amr::kMinGhostWeight, 1e-15);
    CHECK(amr::ghostEvidenceWeight(60, 0) > amr::ghostEvidenceWeight(10, 0));
}

/// A room split by a wall with one doorway (rows 4..6 at x = 8). In front of the
/// doorway, cell (6, 5) holds either a ghost that reflects every beam heading east
/// (+-52 deg), or a real obstacle. The robot mapped from three viewpoints west of
/// it, all inside the ghost's sector, so the two look identical.
struct Doorway {
    GhostWorld world;
    std::vector<WorldPoint> viewpoints;
    OccupancyBelief belief{16, 10, 0.5};
    GridCoord start{2, 5};
    GridCoord goal{13, 5};
};

Doorway makeDoorway(bool real_obstacle, unsigned seed) {
    std::vector<std::string> rows(10, "........#.......");
    for (int y = 4; y <= 6; ++y) {
        rows[static_cast<std::size_t>(y)][8] = '.';
    }
    if (real_obstacle) {
        rows[5][6] = '#';
    }
    Doorway d;
    d.world.truth = makeMap(rows);
    if (!real_obstacle) {
        d.world.ghosts.push_back({{6, 5}, 0.0, 0.9, 1.0});
    }
    for (const GridCoord c : {GridCoord{2, 5}, GridCoord{2, 4}, GridCoord{2, 6}}) {
        d.viewpoints.push_back(d.world.truth.gridToWorld(c));
    }
    std::mt19937 rng(seed);
    for (const WorldPoint& v : d.viewpoints) {
        amr::scanInto(d.belief, d.world, v, noiseless(), rng);
    }
    return d;
}

bool beliefHasPath(const Doorway& d) {
    const GridMap seen = d.belief.map();
    return amr::planWithFootprint(seen, amr::inflateObstacles(seen, 0.3), d.start, d.goal,
                                  amr::Connectivity::Four)
        .success();
}

void testDoorwayGhost() {
    Doorway d = makeDoorway(false, 5);
    const std::size_t ghost = d.world.truth.index({6, 5});
    REQUIRE(d.belief.log_odds[ghost] > 0.0);
    CHECK_EQ(d.belief.passes[ghost], 0);  // never seen through: looks real
    REQUIRE(!beliefHasPath(d));
    amr::RecoveryConfig config;
    std::mt19937 rng(9);
    const auto likely = amr::recover(RecoveryStrategy::MostLikely, d.belief, d.world, d.viewpoints,
                                     d.start, d.goal, noiseless(), config, rng);
    // The cheapest explanation first names the two doorway edges (wall cells seen only
    // a few times, at grazing angles), not the ghost hit 25 times: correct under the
    // model. Re-sensing confirms them, and the ghost is next; seen from the side, the
    // beams pass through it.
    CHECK(likely.outcome == RecoveryOutcome::Recovered);
    CHECK(likely.cells_resensed <= 8);                 // a handful of cells
    CHECK(likely.beams == likely.cells_resensed * config.beams_per_cell);
    CHECK(likely.travel_m > 0.0);                      // it drove to see from another side
    CHECK_EQ(likely.real_cells_cleared, 0);
    CHECK(d.belief.passes[ghost] == 0);                // (the input belief is unchanged)
    // Clearing the map and scanning again from the same places brings the ghost back.
    const auto clear = amr::recover(RecoveryStrategy::ClearAll, d.belief, d.world, d.viewpoints,
                                    d.start, d.goal, noiseless(), config, rng);
    CHECK(clear.outcome == RecoveryOutcome::GaveUp);
    CHECK(clear.beams >= 3 * 180);
    // Re-sensing everything looks at every believed obstacle, and with plain log-odds
    // the ghost's 25 hits outweigh the passes for many rounds.
    const auto all = amr::recover(RecoveryStrategy::ResenseAll, d.belief, d.world, d.viewpoints,
                                  d.start, d.goal, noiseless(), config, rng);
    CHECK(all.outcome != RecoveryOutcome::Unsafe);
    CHECK(all.beams > 5 * likely.beams);
}

void testDoorwayRealObstacle() {
    Doorway d = makeDoorway(true, 5);
    REQUIRE(!beliefHasPath(d));
    amr::RecoveryConfig config;
    config.max_rounds = 6;
    std::mt19937 rng(9);
    // Re-sensing never declares a real obstacle free: the robot gives up safely.
    for (const RecoveryStrategy s :
         {RecoveryStrategy::MostLikely, RecoveryStrategy::LogOdds, RecoveryStrategy::FewestCells}) {
        const auto r = amr::recover(s, d.belief, d.world, d.viewpoints, d.start, d.goal, noiseless(),
                                    config, rng);
        CHECK(r.outcome == RecoveryOutcome::GaveUp);
        CHECK_EQ(r.real_cells_cleared, 0);
    }
    // Deleting the explanation without looking opens a path through the real obstacle.
    const auto blind = amr::recover(RecoveryStrategy::BlindDelete, d.belief, d.world, d.viewpoints,
                                    d.start, d.goal, noiseless(), config, rng);
    CHECK(blind.outcome == RecoveryOutcome::Unsafe);
    CHECK(blind.real_cells_cleared >= 1);
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Beam trace: passes, hit, range, map edge", testBeamTrace},
        {"Ghost source: in-sector hit rate, out-of-sector pass", testGhostSource},
        {"Sensor noise: misses and spurious returns", testSensorNoise},
        {"Occupancy update: log-odds, counts, bearings, clamp", testOccupancyUpdate},
        {"Ghost evidence: passes outweigh hits", testGhostEvidence},
        {"Doorway ghost: re-sensing a few cells recovers; clearing does not", testDoorwayGhost},
        {"Doorway real obstacle: never cleared by re-sensing; blind delete is unsafe", testDoorwayRealObstacle},
    });
}
