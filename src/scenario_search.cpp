#include "scenario_search.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>

namespace amr {

namespace {

constexpr double kFailureDanger = 10.0;

bool failed(NavigationStatus status) {
    return status == NavigationStatus::Collision || status == NavigationStatus::Timeout;
}

/// The lane of one obstacle: its end cell (start itself if the lane is empty).
GridCoord laneEnd(const GridMap& map, const LaneObstacle& o, GridCoord start, GridCoord goal) {
    GridCoord b = o.start;
    for (int i = 0; i < o.length; ++i) {
        const GridCoord next{b.x + (o.horizontal ? 1 : 0), b.y + (o.horizontal ? 0 : 1)};
        if (!map.isFree(next) || next == start || next == goal) {
            break;
        }
        b = next;
    }
    return b;
}

/// A key that identifies a scenario by its obstacles (order does not matter).
std::string key(const GridMap& map, const SearchScenario& s, GridCoord start, GridCoord goal) {
    std::vector<std::tuple<int, int, int, int, long, long>> parts;
    for (const LaneObstacle& o : s.obstacles) {
        const GridCoord b = laneEnd(map, o, start, goal);
        parts.emplace_back(o.start.x, o.start.y, b.x, b.y, std::lround(o.speed * 1e6), std::lround(o.radius * 1e6));
    }
    std::sort(parts.begin(), parts.end());
    std::ostringstream out;
    for (const auto& [x0, y0, x1, y1, v, r] : parts) {
        out << x0 << ',' << y0 << ',' << x1 << ',' << y1 << ',' << v << ',' << r << ';';
    }
    return out.str();
}

bool valid(const GridMap& map, const ScenarioSearchConfig& config, const SearchScenario& s) {
    return static_cast<int>(toObstacles(map, s, config.start, config.goal).size()) ==
           static_cast<int>(s.obstacles.size());
}

LaneObstacle randomLane(const GridMap& map, const ScenarioSearchConfig& config, std::mt19937& rng) {
    std::uniform_int_distribution<int> xs(0, map.width() - 1), ys(0, map.height() - 1);
    std::uniform_int_distribution<int> length(config.min_length, config.max_length);
    std::uniform_real_distribution<double> speed(config.min_speed, config.max_speed);
    std::uniform_real_distribution<double> radius(config.min_radius, config.max_radius);
    LaneObstacle o;
    o.start = {xs(rng), ys(rng)};
    o.horizontal = rng() % 2 == 0;
    o.length = length(rng);
    o.speed = speed(rng);
    o.radius = radius(rng);
    return o;
}

/// One or two random changes; retried until the scenario is valid.
SearchScenario mutate(const GridMap& map, const ScenarioSearchConfig& config, const SearchScenario& parent,
                      std::mt19937& rng) {
    for (int attempt = 0; attempt < 50; ++attempt) {
        SearchScenario child = parent;
        const int changes = 1 + static_cast<int>(rng() % 2);
        for (int c = 0; c < changes; ++c) {
            LaneObstacle& o = child.obstacles[rng() % child.obstacles.size()];
            std::normal_distribution<double> jitter(0.0, 1.0);
            switch (rng() % 7) {
                case 0:  // shift along x
                    o.start.x += (rng() % 2 == 0 ? 1 : -1) * (1 + static_cast<int>(rng() % 2));
                    break;
                case 1:  // shift along y
                    o.start.y += (rng() % 2 == 0 ? 1 : -1) * (1 + static_cast<int>(rng() % 2));
                    break;
                case 2:
                    o.horizontal = !o.horizontal;
                    break;
                case 3:
                    o.length = std::clamp(o.length + (rng() % 2 == 0 ? 1 : -1), config.min_length, config.max_length);
                    break;
                case 4:
                    o.speed = std::clamp(o.speed * (1.0 + 0.15 * jitter(rng)), config.min_speed, config.max_speed);
                    break;
                case 5:
                    o.radius = std::clamp(o.radius + 0.02 * jitter(rng), config.min_radius, config.max_radius);
                    break;
                default:
                    o = randomLane(map, config, rng);
                    break;
            }
        }
        if (valid(map, config, child)) {
            return child;
        }
    }
    return parent;
}

/// Distinct failures, recorded in order.
class Archive {
public:
    Archive(const GridMap& map, const ScenarioSearchConfig& config) : map_(map), config_(config) {}

    /// Records a failing run; true if it was new.
    bool record(const SearchScenario& s, const Evaluation& e, std::size_t simulation, ScenarioSearchResult& out) {
        if (!keys_.insert(key(map_, s, config_.start, config_.goal)).second) {
            return false;
        }
        out.failures.push_back({s, e, simulation});
        return true;
    }

private:
    const GridMap& map_;
    const ScenarioSearchConfig& config_;
    std::set<std::string> keys_;
};

}  // namespace

Evaluation evaluateRun(const GridMap& map, const SimulationResult& result,
                       const std::vector<DynamicObstacle>& obstacles, double robot_radius) {
    constexpr double kExposureGap = 0.25;  // m
    Evaluation e;
    e.status = result.status;
    if (!result.trajectory.empty()) {
        const auto& last = result.trajectory.back().state;
        e.where = map.worldToGrid({last.x, last.y});
    }
    if (failed(result.status)) {
        e.danger = kFailureDanger;
        return e;
    }
    double exposure = 0.0;
    for (std::size_t k = 1; k < result.trajectory.size(); ++k) {
        const auto& s = result.trajectory[k].state;
        const double dt = s.time - result.trajectory[k - 1].state.time;
        double gap = std::numeric_limits<double>::infinity();
        for (const DynamicObstacle& o : obstacles) {
            const WorldPoint c = obstaclePosition(o, s.time);
            gap = std::min(gap, std::hypot(c.x - s.x, c.y - s.y) - o.radius - robot_radius);
        }
        exposure += std::max(0.0, kExposureGap - gap) * dt;
    }
    e.danger = exposure + 0.01 * static_cast<double>(result.wait_steps) * 0.1;
    return e;
}

std::vector<DynamicObstacle> toObstacles(const GridMap& map, const SearchScenario& scenario, GridCoord start,
                                         GridCoord goal) {
    std::vector<DynamicObstacle> out;
    for (const LaneObstacle& o : scenario.obstacles) {
        if (!map.isFree(o.start) || o.start == start || o.start == goal) {
            continue;
        }
        const GridCoord b = laneEnd(map, o, start, goal);
        if (b == o.start) {
            continue;
        }
        DynamicObstacle d;
        d.route = {map.gridToWorld(o.start), map.gridToWorld(b)};
        d.speed = o.speed;
        d.radius = o.radius;
        if (validateDynamicObstacle(map, d).empty()) {
            out.push_back(d);
        }
    }
    return out;
}

SearchScenario randomScenario(const GridMap& map, const ScenarioSearchConfig& config, std::mt19937& rng) {
    SearchScenario s;
    while (static_cast<int>(s.obstacles.size()) < config.obstacles) {
        SearchScenario candidate = s;
        candidate.obstacles.push_back(randomLane(map, config, rng));
        if (valid(map, config, candidate)) {
            s = candidate;
        }
    }
    return s;
}

ScenarioSearchResult searchFailures(const GridMap& map, const ScenarioSearchConfig& config,
                                    const BatchEvaluator& evaluate) {
    ScenarioSearchResult result;
    result.best_danger = -std::numeric_limits<double>::infinity();
    std::mt19937 rng(config.seed);
    Archive archive(map, config);
    std::set<std::pair<int, int>> places;  // where known failures happened
    const std::size_t n = std::max<std::size_t>(2, config.population);
    struct Member {
        SearchScenario scenario;
        double danger;
        int age;
    };
    std::vector<Member> pool;
    std::vector<SearchScenario> batch;
    for (std::size_t i = 0; i < n; ++i) {
        batch.push_back(randomScenario(map, config, rng));
    }
    while (result.simulations + batch.size() <= config.budget) {
        const std::vector<Evaluation> evaluations = evaluate(batch);
        for (std::size_t i = 0; i < batch.size(); ++i) {
            const Evaluation& e = evaluations[i];
            ++result.simulations;
            if (failed(e.status)) {
                archive.record(batch[i], e, result.simulations, result);
                // A failure is not kept: its neighbours mostly fail the same way. A
                // failure somewhere new is worth looking around, though: its first
                // neighbours join with a high danger.
                if (places.insert({e.where.x, e.where.y}).second) {
                    pool.push_back({batch[i], 1.0, 0});
                }
                continue;
            }
            result.best_danger = std::max(result.best_danger, e.danger);
            pool.push_back({batch[i], e.danger, 0});
        }
        // (mu + lambda): keep the n most dangerous, ties to the younger; members that
        // stayed for `patience` generations make way (they have been mined).
        for (Member& m : pool) {
            ++m.age;
        }
        pool.erase(std::remove_if(pool.begin(), pool.end(), [&](const Member& m) { return m.age > config.patience; }),
                   pool.end());
        std::stable_sort(pool.begin(), pool.end(), [](const Member& a, const Member& b) {
            return a.danger != b.danger ? a.danger > b.danger : a.age < b.age;
        });
        if (pool.size() > n) {
            pool.resize(n);
        }
        // Children of tournament winners; a quarter of the batch is fresh random
        // scenarios, so the search does not stop exploring.
        batch.clear();
        for (std::size_t i = 0; i < n; ++i) {
            if (pool.empty() || i % 4 == 3) {
                batch.push_back(randomScenario(map, config, rng));
                continue;
            }
            const Member& a = pool[rng() % pool.size()];
            const Member& b = pool[rng() % pool.size()];
            batch.push_back(mutate(map, config, (a.danger >= b.danger ? a : b).scenario, rng));
        }
    }
    return result;
}

ScenarioSearchResult sampleFailures(const GridMap& map, const ScenarioSearchConfig& config,
                                    const BatchEvaluator& evaluate) {
    ScenarioSearchResult result;
    result.best_danger = -std::numeric_limits<double>::infinity();
    std::mt19937 rng(config.seed);
    Archive archive(map, config);
    const std::size_t n = std::max<std::size_t>(1, config.population);
    while (result.simulations + n <= config.budget) {
        std::vector<SearchScenario> batch;
        for (std::size_t i = 0; i < n; ++i) {
            batch.push_back(randomScenario(map, config, rng));
        }
        const std::vector<Evaluation> evaluations = evaluate(batch);
        for (std::size_t i = 0; i < n; ++i) {
            ++result.simulations;
            if (failed(evaluations[i].status)) {
                archive.record(batch[i], evaluations[i], result.simulations, result);
            } else {
                result.best_danger = std::max(result.best_danger, evaluations[i].danger);
            }
        }
    }
    return result;
}

SearchScenario shrinkFailure(const GridMap& map, const ScenarioSearchConfig& config, const SearchScenario& failing,
                             const BatchEvaluator& evaluate, std::size_t* simulations) {
    SearchScenario current = failing;
    for (int round = 0; round < 100; ++round) {
        // Every single simplification of the current scenario, simplest first.
        std::vector<SearchScenario> candidates;
        for (std::size_t i = 0; i < current.obstacles.size() && current.obstacles.size() > 1; ++i) {
            SearchScenario c = current;
            c.obstacles.erase(c.obstacles.begin() + static_cast<long>(i));
            candidates.push_back(c);
        }
        for (std::size_t i = 0; i < current.obstacles.size(); ++i) {
            const LaneObstacle& o = current.obstacles[i];
            if (o.length > 1) {
                SearchScenario c = current;
                c.obstacles[i].length = o.length - 1;
                candidates.push_back(c);
            }
            if (o.speed > config.min_speed + 1e-9) {
                SearchScenario c = current;
                c.obstacles[i].speed = std::max(config.min_speed, std::round(o.speed * 0.9 * 100.0) / 100.0);
                candidates.push_back(c);
            }
            if (o.radius > config.min_radius + 1e-9) {
                SearchScenario c = current;
                c.obstacles[i].radius = std::max(config.min_radius, std::round((o.radius - 0.02) * 100.0) / 100.0);
                candidates.push_back(c);
            }
        }
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                        [&](const SearchScenario& c) { return !valid(map, config, c); }),
                         candidates.end());
        if (candidates.empty()) {
            break;
        }
        const std::vector<Evaluation> evaluations = evaluate(candidates);
        if (simulations != nullptr) {
            *simulations += candidates.size();
        }
        bool simpler = false;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (failed(evaluations[i].status)) {
                current = candidates[i];
                simpler = true;
                break;
            }
        }
        if (!simpler) {
            break;
        }
    }
    return current;
}

std::string toObstacleFile(const GridMap& map, const SearchScenario& scenario, GridCoord start, GridCoord goal) {
    std::ostringstream out;
    out << std::setprecision(17);
    for (const LaneObstacle& o : scenario.obstacles) {
        const GridCoord b = laneEnd(map, o, start, goal);
        if (b == o.start) {
            continue;
        }
        out << o.radius << ", " << o.speed << ", " << o.start.x << ", " << o.start.y << ", " << b.x << ", " << b.y
            << "\n";
    }
    return out.str();
}

}  // namespace amr
