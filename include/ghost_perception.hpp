// Closing the perception loop around NO_PATH: an occupancy map built from noisy
// LiDAR scans contains "ghost" cells (false obstacles); when the disc robot cannot
// find a path, explain the failure with the minimum ghost set (ghost_cells.hpp),
// re-sense only those cells, and replan.
//
// World model. The true map, plus ghost sources: free cells that return a false
// hit to beams arriving within an angular sector (a reflection, a glass pane, a
// shiny floor patch). They are persistent and view-dependent: scanning again from
// the same places reproduces them, so clearing the map and re-mapping does not
// remove them, while looking from another direction does.
//
// Sensor. Beams are traced cell by cell (true map plus ghosts). A real obstacle
// returns a hit except for occasional misses (no return); an active ghost source
// returns a hit with its hit probability; a beam may also return a spurious short
// range. Occupancy is a log-odds grid (inverse sensor model: +kLogOddsHit for the
// hit cell, kLogOddsMiss for the cells passed), clamped to +-kLogOddsLimit. A cell
// is believed occupied when its log-odds are positive; unknown cells count as free.
// The clamp is high on purpose: the log-odds are also the removal costs of the
// explanation, and a low clamp (the usual +-3..6, meant for maps that must follow a
// changing world) makes a cell confirmed by 60 hits look as uncertain as a ghost
// seen 8 times. The explanation then kept choosing the same confirmed real cell.
//
// Ghost evidence. The occupancy log-odds treat a beam that passes through a cell as
// weak evidence (kLogOddsMiss = -0.4 against +0.85 per hit). But a beam can only pass
// through a ghost, never through a solid obstacle, so for "real or ghost?" one pass
// outweighs many hits. Two hypotheses for a cell believed occupied:
//   real:  every beam reaching it returns a hit (1 - miss), a pass is a fluke (eps)
//   ghost: in-sector beams hit with probability q, the rest pass
// give the log-likelihood ratio  prior + hits * log((1 - miss) / q) + passes * log(eps)
// (ghostEvidenceWeight): about +0.2 per hit and -3.9 per pass. It is the removal
// cost of the most likely explanation; the plain log-odds are kept as a strategy
// for comparison.
//
// Recovery strategies, applied while the belief map has NO_PATH (round limit):
//   MostLikely  explain with the minimum-weight ghost set, weights =
//               ghostEvidenceWeight (hit and pass counts), re-sense, and decide each
//               re-sensed cell by the same evidence (ratio <= 0: a ghost, freed);
//               replan
//   LogOdds     the same with weights = ghostWeight(p) of the occupancy log-odds
//   FewestCells explain with the minimum number of cells, re-sense, replan
//   ClearAll    reset the whole map and re-scan from every viewpoint (the costmap
//               clearing recovery of ROS navigation)
//   BlindDelete delete the most likely explanation without looking (fast, unsafe)
//   ResenseAll  re-sense every occupied cell the robot can see
// Re-sensing is active: the robot may drive to any cell it believes reachable. To
// re-sense a cell it picks the viewpoint (in range and in line of sight on the
// belief map) whose bearing to the cell differs most from the bearings that already
// produced hits there or were already used, preferring short drives, and aims beams
// at the cell's center. The map remembers the bearings of the hits in each cell, so
// a view-dependent ghost is looked at from another side first.
#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

struct GhostSource {
    GridCoord cell;
    double direction = 0.0;        ///< center of the active sector: beam heading, rad
    double half_width = 0.9;       ///< sector half width, rad
    double hit_probability = 0.8;  ///< chance an in-sector beam returns a hit here
};

struct SensorModel {
    int beams = 180;               ///< beams per full scan
    double max_range = 5.0;        ///< m
    double miss_probability = 0.02;     ///< a real obstacle returns nothing
    double spurious_probability = 0.01; ///< a beam returns a random short range
};

constexpr double kLogOddsHit = 0.85;    ///< inverse sensor model, hit cell (p = 0.70)
constexpr double kLogOddsMiss = -0.4;   ///< cells a beam passed (p = 0.40)
constexpr double kLogOddsLimit = 30.0;  // high: the evidence count matters (see below)

/// Occupancy grid in log-odds.
struct OccupancyBelief {
    int width = 0;
    int height = 0;
    double resolution = 0.5;
    std::vector<double> log_odds;  ///< per cell, 0 = unknown
    std::vector<std::vector<float>> hit_bearings;  ///< per cell: headings of beams that hit it
    std::vector<int> hits;    ///< per cell: beams that returned a hit there
    std::vector<int> passes;  ///< per cell: beams that passed through it

    OccupancyBelief(int w, int h, double res);
    double probability(std::size_t index) const;  ///< P(occupied)
    GridMap map() const;                          ///< occupied iff log-odds > 0
};

/// The simulated world: true map and ghost sources.
struct GhostWorld {
    GridMap truth;
    std::vector<GhostSource> ghosts;
};

/// Result of one beam: the cells it passed and the cell it hit (-1: no return).
struct BeamTrace {
    std::vector<int> passed;
    int hit = -1;
    double angle = 0.0;  ///< beam heading, rad
};

/// Trace one beam from `origin` with heading `angle` for up to `range` meters.
BeamTrace traceBeam(const GhostWorld& world, WorldPoint origin, double angle, double range,
                    const SensorModel& sensor, std::mt19937& rng);

/// Apply a beam to the belief (log-odds update).
void integrateBeam(OccupancyBelief& belief, const BeamTrace& trace);

/// A full 360-degree scan from `origin` into the belief; returns the beams fired.
int scanInto(OccupancyBelief& belief, const GhostWorld& world, WorldPoint origin,
             const SensorModel& sensor, std::mt19937& rng);

enum class RecoveryStrategy { MostLikely, LogOdds, FewestCells, ClearAll, BlindDelete, ResenseAll };

/// Ghost-hypothesis parameters (see the file comment).
constexpr double kGhostPriorLogRatio = 3.0;  ///< log P(real) / P(ghost) before evidence (~1:20)
constexpr double kRealMiss = 0.02;           ///< a real obstacle returns nothing
constexpr double kGhostHitProbability = 0.8; ///< an in-sector beam hits a ghost
constexpr double kSeeThroughReal = 0.02;     ///< eps: a beam "passes" a real cell (edge effects)

/// Log likelihood ratio real : ghost from a cell's hit and pass counts (prior included).
double ghostLogLikelihoodRatio(int hits, int passes);

/// Removal cost of a believed-occupied cell: ghostLogLikelihoodRatio clamped to
/// [kMinGhostWeight, kMaxGhostWeight].
double ghostEvidenceWeight(int hits, int passes);
std::string toString(RecoveryStrategy strategy);

struct RecoveryConfig {
    double robot_radius = 0.3;
    Connectivity connectivity = Connectivity::Four;
    int max_rounds = 10;
    int beams_per_cell = 5;        ///< beams per re-sensing of one cell
    std::size_t max_labels = 2'000'000;  ///< solver limit per round
};

enum class RecoveryOutcome {
    Recovered,    ///< found a path that is also free in the true world
    Unsafe,       ///< found a path that crosses a real obstacle (inflated)
    GaveUp,       ///< no path after the round limit, or nothing left to try
};
std::string toString(RecoveryOutcome outcome);

struct RecoveryResult {
    RecoveryOutcome outcome = RecoveryOutcome::GaveUp;
    int rounds = 0;
    long beams = 0;                 ///< beams fired by the recovery
    int cells_resensed = 0;         ///< re-sensing actions (cell x viewpoint)
    double travel_m = 0.0;          ///< straight-line driving between viewpoints, m
    int real_cells_cleared = 0;     ///< real obstacles believed free at the end
    double solver_ms = 0.0;
    std::string reason;             ///< why it gave up: round_limit, no_explanation, cannot_see
    std::vector<GridCoord> path;
};

/// Run one recovery strategy from a belief with NO_PATH. `viewpoints` are the
/// positions the robot has scanned from (ClearAll re-scans from them); re-sensing
/// may also use any cell the robot believes reachable from the start.
RecoveryResult recover(RecoveryStrategy strategy, OccupancyBelief belief, const GhostWorld& world,
                       const std::vector<WorldPoint>& viewpoints, GridCoord start, GridCoord goal,
                       const SensorModel& sensor, const RecoveryConfig& config,
                       std::mt19937& rng);

}  // namespace amr
