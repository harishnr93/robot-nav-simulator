// Searching for failure scenarios: instead of sampling obstacle routes at random and
// waiting for the rare failure, optimize them towards one.
//
// A scenario is a set of obstacles on straight lanes (the benchmark's generator:
// lanes of 2-8 cells from a start cell, horizontal or vertical, clear of the robot's
// start and goal), each with a speed and a radius. A run of the robot through it is
// scored by its danger:
//
//     danger = 10                                       if the run failed (collision or timeout)
//            = exposure + 0.01 x waiting time           otherwise
//     exposure = sum over the trajectory of max(0, 0.25 m - gap) x dt   (m s)
//
// (gap: from the robot's edge to the nearest obstacle edge at each sample), so a run
// that passed close, for long, or waited long, scores higher than one that passed far
// away, and the search has a slope to climb before it finds a failure. (The closest
// gap alone is flat: the robot keeps the same small gap in most close encounters.)
//
// The search is an evolution strategy with restarts:
//   - a population of random scenarios is evaluated;
//   - each generation, every member produces one child by 1-2 mutations (shift a
//     lane by a cell or two, turn it, lengthen or shorten it, change a speed or a
//     radius, or replace an obstacle by a random one); a child replaces its parent
//     if it is at least as dangerous;
//   - a member that fails is recorded (if not seen before) and restarted from a new
//     random scenario, so that the search keeps looking elsewhere instead of finding
//     the same failure again and again; so is a member that did not improve for
//     `patience` generations.
// Random sampling with the same number of simulations is the baseline.
//
// Shrinking: a failure is reduced to a minimal one that still fails, by removing
// obstacles, shortening lanes, slowing obstacles down and shrinking them, one change
// at a time, keeping every change after which the run still fails.
//
// Runs are evaluated in batches through a callback, so that the caller can run them
// in parallel; the search itself is deterministic for a given seed.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "dynamic_obstacles.hpp"
#include "grid_map.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace amr {

/// An obstacle on a straight lane.
struct LaneObstacle {
    GridCoord start;
    bool horizontal = true;
    int length = 4;        ///< cells beyond the start (the lane stops early at walls, start, goal)
    double speed = 0.3;    ///< m/s
    double radius = 0.3;   ///< m
};

struct SearchScenario {
    std::vector<LaneObstacle> obstacles;
};

/// The outcome of one run, as the search sees it.
struct Evaluation {
    double danger = 0.0;
    NavigationStatus status = NavigationStatus::NotRun;
    GridCoord where;       ///< the robot's cell at the end of the run
};

/// Danger of a finished run through `obstacles` (see the header comment).
Evaluation evaluateRun(const GridMap& map, const SimulationResult& result,
                       const std::vector<DynamicObstacle>& obstacles, double robot_radius = 0.0);

struct ScenarioSearchConfig {
    GridCoord start;            ///< the robot's start and goal (lanes keep clear of them)
    GridCoord goal;
    int obstacles = 3;
    double min_speed = 0.2, max_speed = 0.5;
    double min_radius = 0.25, max_radius = 0.35;
    int min_length = 2, max_length = 8;
    std::size_t population = 16;
    std::size_t budget = 2000;  ///< simulations
    int patience = 8;           ///< generations without improvement before a restart
    std::uint32_t seed = 1;
};

/// Runs a batch of scenarios (in any order, possibly in parallel) and returns their
/// evaluations, one per scenario.
using BatchEvaluator = std::function<std::vector<Evaluation>(const std::vector<SearchScenario>&)>;

struct FoundFailure {
    SearchScenario scenario;
    Evaluation evaluation;
    std::size_t simulation = 0;  ///< how many simulations had run when it was found
};

struct ScenarioSearchResult {
    std::vector<FoundFailure> failures;  ///< distinct failures, in the order found
    std::size_t simulations = 0;
    double best_danger = 0.0;            ///< among the runs that did not fail
};

/// The obstacles of a scenario (lanes cut at walls, start and goal). Obstacles
/// whose lane is empty are dropped.
std::vector<DynamicObstacle> toObstacles(const GridMap& map, const SearchScenario& scenario,
                                         GridCoord start, GridCoord goal);

/// A random scenario, as the benchmark's generator draws it.
SearchScenario randomScenario(const GridMap& map, const ScenarioSearchConfig& config, std::mt19937& rng);

/// The evolution strategy of the header comment.
ScenarioSearchResult searchFailures(const GridMap& map, const ScenarioSearchConfig& config,
                                    const BatchEvaluator& evaluate);

/// The baseline: `config.budget` random scenarios, evaluated in batches of
/// `config.population`.
ScenarioSearchResult sampleFailures(const GridMap& map, const ScenarioSearchConfig& config,
                                    const BatchEvaluator& evaluate);

/// A minimal version of a failing scenario that still fails (one change at a time).
SearchScenario shrinkFailure(const GridMap& map, const ScenarioSearchConfig& config, const SearchScenario& failing,
                             const BatchEvaluator& evaluate, std::size_t* simulations = nullptr);

/// The obstacle file (loadDynamicObstacles format) of a scenario.
std::string toObstacleFile(const GridMap& map, const SearchScenario& scenario, GridCoord start, GridCoord goal);

}  // namespace amr
