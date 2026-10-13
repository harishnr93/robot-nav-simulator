#include "ghost_perception.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "footprint.hpp"
#include "ghost_cells.hpp"
#include "lidar.hpp"

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Driving-cost weight in the viewpoint choice: rad of bearing difference per meter.
constexpr double kTravelWeight = 0.01;

/// Re-sense one cell: aim beams at its center from the viewpoint in line of sight
/// (on the belief map) whose bearing differs most from the bearings already used
/// (or that produced its hits), preferring short drives from `robot`. Returns the
/// beams fired (0 if no viewpoint sees the cell); moves `robot` to the viewpoint.
int resense(OccupancyBelief& belief, const GhostWorld& world,
            const std::vector<WorldPoint>& viewpoints, GridCoord cell,
            std::vector<double>& used_bearings, const SensorModel& sensor, int beams,
            WorldPoint& robot, double& travel, std::mt19937& rng) {
    const GridMap seen = belief.map();
    const WorldPoint target = seen.gridToWorld(cell);
    int best = -1;
    double best_score = -1.0;
    for (std::size_t i = 0; i < viewpoints.size(); ++i) {
        const WorldPoint& v = viewpoints[i];
        const double distance = std::hypot(target.x - v.x, target.y - v.y);
        if (distance > sensor.max_range || distance < 1e-9) {
            continue;
        }
        const double bearing = std::atan2(target.y - v.y, target.x - v.x);
        // Line of sight: the first believed obstacle along the beam is the cell itself.
        const RayHit hit = castRay(seen, v, bearing, distance);
        if (hit.hit) {
            const WorldPoint at{v.x + (hit.range + 1e-6) * std::cos(bearing),
                                v.y + (hit.range + 1e-6) * std::sin(bearing)};
            if (seen.worldToGrid(at) != cell) {
                continue;
            }
        }
        double score = kPi;  // new bearing: as different as it gets
        for (const double b : used_bearings) {
            score = std::min(score, std::fabs(wrapAngle(bearing - b)));
        }
        // Prefer the most different bearing, then a short drive.
        score -= kTravelWeight * std::hypot(v.x - robot.x, v.y - robot.y);
        if (score > best_score) {
            best_score = score;
            best = static_cast<int>(i);
        }
    }
    if (best < 0) {
        return 0;
    }
    const WorldPoint& v = viewpoints[static_cast<std::size_t>(best)];
    travel += std::hypot(v.x - robot.x, v.y - robot.y);
    robot = v;
    const double distance = std::hypot(target.x - v.x, target.y - v.y);
    const double bearing = std::atan2(target.y - v.y, target.x - v.x);
    used_bearings.push_back(bearing);
    // Small jitter: the beams cross the cell, not just its exact center.
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const double spread = belief.resolution / std::max(distance, belief.resolution);
    for (int b = 0; b < beams; ++b) {
        const double angle = bearing + jitter(rng) * spread;
        integrateBeam(belief, traceBeam(world, v, angle, distance + 0.75 * belief.resolution,
                                        sensor, rng));
    }
    return beams;
}

/// Cells the robot believes reachable from `start` (4-connected, inflated belief).
std::vector<GridCoord> believedReachable(const GridMap& seen, double radius, GridCoord start) {
    const GridMap inflated = inflateObstacles(seen, radius);
    std::vector<GridCoord> out;
    if (!inflated.isFree(start)) {
        return out;
    }
    std::vector<bool> visited(inflated.cellCount(), false);
    std::vector<GridCoord> stack{start};
    visited[inflated.index(start)] = true;
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    while (!stack.empty()) {
        const GridCoord c = stack.back();
        stack.pop_back();
        out.push_back(c);
        for (int d = 0; d < 4; ++d) {
            const GridCoord q{c.x + dx[d], c.y + dy[d]};
            if (inflated.isFree(q) && !visited[inflated.index(q)]) {
                visited[inflated.index(q)] = true;
                stack.push_back(q);
            }
        }
    }
    return out;
}

bool pathIsFreeIn(const GridMap& truth, double radius, const std::vector<GridCoord>& path,
                  Connectivity connectivity) {
    return isValidGridPath(inflateObstacles(truth, radius), path, connectivity);
}

}  // namespace

OccupancyBelief::OccupancyBelief(int w, int h, double res)
    : width(w), height(h), resolution(res),
      log_odds(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0.0),
      hit_bearings(log_odds.size()), hits(log_odds.size(), 0), passes(log_odds.size(), 0) {}

double ghostLogLikelihoodRatio(int hits, int passes) {
    return kGhostPriorLogRatio + hits * std::log((1.0 - kRealMiss) / kGhostHitProbability) +
           passes * std::log(kSeeThroughReal);
}

double ghostEvidenceWeight(int hits, int passes) {
    return std::clamp(ghostLogLikelihoodRatio(hits, passes), kMinGhostWeight, kMaxGhostWeight);
}

double OccupancyBelief::probability(std::size_t index) const {
    return 1.0 / (1.0 + std::exp(-log_odds[index]));
}

GridMap OccupancyBelief::map() const {
    std::vector<std::uint8_t> cells(log_odds.size());
    for (std::size_t i = 0; i < log_odds.size(); ++i) {
        cells[i] = log_odds[i] > 0.0 ? GridMap::kOccupied : GridMap::kFree;
    }
    return GridMap(width, height, resolution, std::move(cells));
}

BeamTrace traceBeam(const GhostWorld& world, WorldPoint origin, double angle, double range,
                    const SensorModel& sensor, std::mt19937& rng) {
    BeamTrace trace;
    trace.angle = angle;
    const GridMap& map = world.truth;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double spurious_at =
        unit(rng) < sensor.spurious_probability ? unit(rng) * range : std::numeric_limits<double>::infinity();
    const double step = map.resolution() / 16.0;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    int previous = static_cast<int>(map.index(map.worldToGrid(origin)));
    for (double t = step; t <= range; t += step) {
        const GridCoord cell = map.worldToGrid({origin.x + t * c, origin.y + t * s});
        if (!map.inBounds(cell)) {
            return trace;  // left the map: no return
        }
        const int index = static_cast<int>(map.index(cell));
        if (index == previous) {
            continue;
        }
        previous = index;
        if (map.isOccupied(cell)) {
            if (unit(rng) >= sensor.miss_probability) {
                trace.hit = index;
            }
            return trace;  // a miss returns nothing (no information past here)
        }
        for (const GhostSource& ghost : world.ghosts) {
            if (ghost.cell == cell && std::fabs(wrapAngle(angle - ghost.direction)) <= ghost.half_width &&
                unit(rng) < ghost.hit_probability) {
                trace.hit = index;
                return trace;
            }
        }
        if (t >= spurious_at) {
            trace.hit = index;
            return trace;
        }
        trace.passed.push_back(index);
    }
    return trace;
}

void integrateBeam(OccupancyBelief& belief, const BeamTrace& trace) {
    for (const int index : trace.passed) {
        double& l = belief.log_odds[static_cast<std::size_t>(index)];
        l = std::max(l + kLogOddsMiss, -kLogOddsLimit);
        ++belief.passes[static_cast<std::size_t>(index)];
    }
    if (trace.hit >= 0) {
        double& l = belief.log_odds[static_cast<std::size_t>(trace.hit)];
        l = std::min(l + kLogOddsHit, kLogOddsLimit);
        ++belief.hits[static_cast<std::size_t>(trace.hit)];
        auto& bearings = belief.hit_bearings[static_cast<std::size_t>(trace.hit)];
        if (bearings.size() < 64) {
            bearings.push_back(static_cast<float>(trace.angle));
        }
    }
}

int scanInto(OccupancyBelief& belief, const GhostWorld& world, WorldPoint origin,
             const SensorModel& sensor, std::mt19937& rng) {
    for (int b = 0; b < sensor.beams; ++b) {
        const double angle = -kPi + 2.0 * kPi * b / sensor.beams;
        integrateBeam(belief, traceBeam(world, origin, angle, sensor.max_range, sensor, rng));
    }
    return sensor.beams;
}

std::string toString(RecoveryStrategy strategy) {
    switch (strategy) {
        case RecoveryStrategy::MostLikely:  return "most_likely";
        case RecoveryStrategy::LogOdds:     return "log_odds";
        case RecoveryStrategy::FewestCells: return "fewest_cells";
        case RecoveryStrategy::ClearAll:    return "clear_all";
        case RecoveryStrategy::BlindDelete: return "blind_delete";
        case RecoveryStrategy::ResenseAll:  return "resense_all";
    }
    return "unknown";
}

std::string toString(RecoveryOutcome outcome) {
    switch (outcome) {
        case RecoveryOutcome::Recovered: return "RECOVERED";
        case RecoveryOutcome::Unsafe:    return "UNSAFE";
        case RecoveryOutcome::GaveUp:    return "GAVE_UP";
    }
    return "UNKNOWN";
}

RecoveryResult recover(RecoveryStrategy strategy, OccupancyBelief belief, const GhostWorld& world,
                       const std::vector<WorldPoint>& viewpoints, GridCoord start, GridCoord goal,
                       const SensorModel& sensor, const RecoveryConfig& config,
                       std::mt19937& rng) {
    RecoveryResult result;
    std::vector<bool> believed_at_start(belief.log_odds.size());
    for (std::size_t i = 0; i < belief.log_odds.size(); ++i) {
        believed_at_start[i] = belief.log_odds[i] > 0.0;
    }
    const auto finish = [&](RecoveryOutcome outcome) {
        result.outcome = outcome;
        for (std::size_t i = 0; i < belief.log_odds.size(); ++i) {
            if (believed_at_start[i] && belief.log_odds[i] <= 0.0 &&
                world.truth.data()[i] != GridMap::kFree) {
                ++result.real_cells_cleared;
            }
        }
        return result;
    };
    std::map<std::size_t, std::vector<double>> bearings;  // per re-sensed cell
    WorldPoint robot = belief.map().gridToWorld(start);
    for (int round = 0;; ++round) {
        const GridMap seen = belief.map();
        const PlanResult plan = planWithFootprint(
            seen, inflateObstacles(seen, config.robot_radius), start, goal, config.connectivity);
        if (plan.success()) {
            result.path = plan.path;
            return finish(pathIsFreeIn(world.truth, config.robot_radius, plan.path,
                                       config.connectivity)
                              ? RecoveryOutcome::Recovered
                              : RecoveryOutcome::Unsafe);
        }
        if (round >= config.max_rounds) {
            result.reason = "round_limit";
            return finish(RecoveryOutcome::GaveUp);
        }
        ++result.rounds;
        if (strategy == RecoveryStrategy::ClearAll) {
            std::fill(belief.log_odds.begin(), belief.log_odds.end(), 0.0);
            for (auto& b : belief.hit_bearings) {
                b.clear();
            }
            std::fill(belief.hits.begin(), belief.hits.end(), 0);
            std::fill(belief.passes.begin(), belief.passes.end(), 0);
            for (const WorldPoint& v : viewpoints) {
                result.beams += scanInto(belief, world, v, sensor, rng);
            }
            continue;
        }
        std::vector<GridCoord> targets;
        if (strategy == RecoveryStrategy::ResenseAll) {
            for (std::size_t i = 0; i < belief.log_odds.size(); ++i) {
                if (belief.log_odds[i] > 0.0) {
                    targets.push_back(seen.coordFromIndex(i));
                }
            }
        } else {
            GhostQuery query;
            query.robot_radius = config.robot_radius;
            query.connectivity = config.connectivity;
            query.max_labels = config.max_labels;
            if (strategy != RecoveryStrategy::FewestCells) {
                query.weights.assign(belief.log_odds.size(), 0.0);
                for (std::size_t i = 0; i < belief.log_odds.size(); ++i) {
                    if (belief.log_odds[i] > 0.0) {
                        query.weights[i] = strategy == RecoveryStrategy::LogOdds
                                               ? ghostWeight(belief.probability(i))
                                               : ghostEvidenceWeight(belief.hits[i], belief.passes[i]);
                    }
                }
            }
            const GhostResult explanation = findGhostCells(seen, start, goal, query);
            result.solver_ms += explanation.time_ms;
            if (explanation.status != GhostStatus::Found) {
                result.reason = "no_explanation_" + toString(explanation.status);
                return finish(RecoveryOutcome::GaveUp);  // no removable explanation
            }
            targets = explanation.solutions.front();
        }
        if (strategy == RecoveryStrategy::BlindDelete) {
            for (const GridCoord& c : targets) {
                belief.log_odds[seen.index(c)] = -kLogOddsLimit;  // declared free unseen
            }
            continue;
        }
        // Candidate viewpoints: where the robot has been, and every cell it believes
        // it can reach from the start.
        std::vector<WorldPoint> candidates = viewpoints;
        for (const GridCoord& c : believedReachable(seen, config.robot_radius, start)) {
            candidates.push_back(seen.gridToWorld(c));
        }
        long fired = 0;
        for (const GridCoord& c : targets) {
            auto [it, fresh] = bearings.try_emplace(seen.index(c));
            if (fresh) {
                const auto& hits = belief.hit_bearings[seen.index(c)];
                it->second.assign(hits.begin(), hits.end());  // directions that saw a hit
            }
            const int beams = resense(belief, world, candidates, c, it->second, sensor,
                                      config.beams_per_cell, robot, result.travel_m, rng);
            fired += beams;
            // Most likely: decide the re-sensed cell by the same real-vs-ghost evidence
            // (a cell seen through is a ghost however often it reflected).
            const std::size_t i = seen.index(c);
            if (strategy == RecoveryStrategy::MostLikely &&
                ghostLogLikelihoodRatio(belief.hits[i], belief.passes[i]) <= 0.0) {
                belief.log_odds[i] = std::min(belief.log_odds[i], kLogOddsMiss);
            }
            result.cells_resensed += beams > 0 ? 1 : 0;
        }
        result.beams += fired;
        if (fired == 0) {
            result.reason = "cannot_see";
            return finish(RecoveryOutcome::GaveUp);  // nothing can be looked at again
        }
    }
}

}  // namespace amr
