#include "spacetime_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <tuple>
#include <unordered_map>

#include "astar.hpp"
#include "diff_drive.hpp"

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kForever = 1 << 29;

struct Interval {
    int start;
    int end;  // inclusive; kForever: open-ended
};

class SafeIntervals {
public:
    SafeIntervals(const GridMap& map, const std::vector<ObstaclePrediction>& predictions,
                  const SpaceTimeConfig& config, int horizon_steps)
        : map_(map), predictions_(predictions), config_(config), horizon_(horizon_steps) {
        const double res = map.resolution();
        std::unordered_map<int, std::vector<int>> unsafe;
        for (const ObstaclePrediction& o : predictions) {
            const int last = std::min(horizon_steps, static_cast<int>(o.centers.size()) - 1);
            for (int k = 0; k <= last; ++k) {
                const double reach = reachAt(o, k);
                const WorldPoint c = o.centers[static_cast<std::size_t>(k)];
                const int x0 = static_cast<int>(std::floor((c.x - reach) / res));
                const int x1 = static_cast<int>(std::floor((c.x + reach) / res));
                const int y0 = static_cast<int>(std::floor((c.y - reach) / res));
                const int y1 = static_cast<int>(std::floor((c.y + reach) / res));
                for (int y = y0; y <= y1; ++y) {
                    for (int x = x0; x <= x1; ++x) {
                        if (!map.inBounds({x, y})) {
                            continue;
                        }
                        const WorldPoint center = map.gridToWorld({x, y});
                        if (std::hypot(center.x - c.x, center.y - c.y) < reach) {
                            unsafe[y * map.width() + x].push_back(k);
                        }
                    }
                }
            }
        }
        for (auto& [cell, steps] : unsafe) {
            std::sort(steps.begin(), steps.end());
            steps.erase(std::unique(steps.begin(), steps.end()), steps.end());
            std::vector<Interval> safe;
            int from = 0;
            for (const int k : steps) {
                if (k > from) {
                    safe.push_back({from, k - 1});
                }
                from = k + 1;
            }
            safe.push_back({from, kForever});
            intervals_[cell] = std::move(safe);
        }
    }

    const std::vector<Interval>& of(GridCoord c) const {
        const auto it = intervals_.find(c.y * map_.width() + c.x);
        return it == intervals_.end() ? always_ : it->second;
    }

    /// True if a robot center at p is clear of every predicted obstacle at step k.
    bool safeAt(WorldPoint p, int k) const {
        if (k > horizon_) {
            return true;
        }
        for (const ObstaclePrediction& o : predictions_) {
            if (k >= static_cast<int>(o.centers.size())) {
                continue;
            }
            const WorldPoint c = o.centers[static_cast<std::size_t>(k)];
            if (std::hypot(p.x - c.x, p.y - c.y) < reachAt(o, k)) {
                return false;
            }
        }
        return true;
    }

private:
    double reachAt(const ObstaclePrediction& o, int k) const {
        const auto i = static_cast<std::size_t>(k);
        const double inflation = o.inflation.size() > i ? o.inflation[i] : 0.0;
        // Half the displacement to the neighbouring sample: it moves between samples.
        double displacement = 0.0;
        if (i + 1 < o.centers.size()) {
            displacement = std::hypot(o.centers[i + 1].x - o.centers[i].x, o.centers[i + 1].y - o.centers[i].y);
        } else if (i > 0) {
            displacement = std::hypot(o.centers[i].x - o.centers[i - 1].x, o.centers[i].y - o.centers[i - 1].y);
        }
        // (Capped at a cell: a prediction that jumps further is not a motion.)
        return o.radius + inflation + config_.clearance + 0.5 * std::min(displacement, map_.resolution());
    }

    const GridMap& map_;
    const std::vector<ObstaclePrediction>& predictions_;
    const SpaceTimeConfig& config_;
    int horizon_;
    std::unordered_map<int, std::vector<Interval>> intervals_;
    std::vector<Interval> always_{{0, kForever}};
};

struct Node {
    GridCoord cell;
    int interval;
    int direction;   // 0..7 (k x 45 deg), 8 = the start heading
    int arrive;      // step
    int depart_parent;  // step the parent left towards this node
    int parent;
    int moves;       // cells moved so far (tie-break: among equally early plans, drive less)
};

}  // namespace

std::vector<GridCoord> planCells(const SpaceTimePlan& plan) {
    std::vector<GridCoord> cells;
    for (const TimedWaypoint& w : plan.waypoints) {
        cells.push_back(w.cell);
    }
    return cells;
}

SpaceTimePlan planSpaceTime(const GridMap& map, GridCoord start, double start_heading, GridCoord goal,
                            const std::vector<ObstaclePrediction>& predictions, const SpaceTimeConfig& config) {
    const auto t0 = std::chrono::steady_clock::now();
    SpaceTimePlan plan;
    if (validatePlanRequest(map, start, goal) != PlanStatus::Success) {
        return plan;
    }
    const int horizon = static_cast<int>(std::ceil(config.horizon / config.dt - 1e-9));
    const SafeIntervals safe(map, predictions, config, horizon);
    const double res = map.resolution();
    const auto steps_for = [&](double seconds) { return static_cast<int>(std::ceil(seconds / config.dt - 1e-9)); };
    const auto angle_of = [&](int direction) { return direction == 8 ? start_heading : direction * kPi / 4.0; };
    const auto turn_steps = [&](int from, int to) {
        if (!config.turn_aware) {
            return 0;
        }
        const double angle = std::fabs(wrapAngle(angle_of(to) - angle_of(from)));
        if (angle < 1e-6) {
            return 0;
        }
        return steps_for(angle / config.max_angular_speed + config.turn_overhead);
    };
    const auto heuristic = [&](GridCoord c) {
        const double dx = std::abs(c.x - goal.x), dy = std::abs(c.y - goal.y);
        const double cells = config.connectivity == Connectivity::Eight
                                 ? (std::max(dx, dy) - std::min(dx, dy)) + std::min(dx, dy) * std::sqrt(2.0)
                                 : dx + dy;
        return static_cast<int>(std::floor(cells * res / config.speed / config.dt));
    };
    std::vector<Node> nodes;
    // f, then moves (among equally early plans, the one that drives less), node index.
    using Entry = std::tuple<int, int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::unordered_map<long long, int> best;
    const auto key = [&](GridCoord c, int interval, int direction) {
        return ((static_cast<long long>(c.y) * map.width() + c.x) * 4096 + interval) * 16 + direction;
    };
    // The start: the interval containing the arrival there (s0, normally now). If the
    // robot already stands on an unsafe start, it must leave at once: a zero-length
    // interval. A start reached later must be safe on arrival.
    const int s0 = steps_for(config.start_time);
    const auto& start_intervals = safe.of(start);
    int start_interval = -1;
    for (std::size_t i = 0; i < start_intervals.size(); ++i) {
        if (start_intervals[i].start <= s0 && start_intervals[i].end >= s0) {
            start_interval = static_cast<int>(i);
        }
    }
    std::vector<Interval> start_override;
    if (start_interval < 0) {
        if (s0 > 0) {
            plan.planning_time_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            return plan;
        }
        start_interval = 0;
        start_override.push_back({0, 0});
    }
    // Without turn costs the heading does not matter: one state per cell and interval.
    const int start_direction = config.turn_aware ? 8 : 0;
    nodes.push_back({start, start_interval, start_direction, s0, 0, -1, 0});
    best[key(start, start_interval, start_direction)] = s0;
    open.push({s0 + heuristic(start), 0, 0});
    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    int found = -1;
    while (!open.empty() && plan.expanded < config.max_expansions) {
        const auto [f, moves, index] = open.top();
        open.pop();
        const Node node = nodes[static_cast<std::size_t>(index)];
        static_cast<void>(f);
        static_cast<void>(moves);
        if (best[key(node.cell, node.interval, node.direction)] < node.arrive) {
            continue;  // a better copy was expanded
        }
        ++plan.expanded;
        if (node.cell == goal) {
            found = index;
            break;
        }
        const std::vector<Interval>& here = (index == 0 && !start_override.empty()) ? start_override : safe.of(node.cell);
        const int end_here = here[static_cast<std::size_t>(node.interval)].end;
        for (int d = 0; d < 8; ++d) {
            if (config.connectivity == Connectivity::Four && d % 2 == 1) {
                continue;
            }
            const GridCoord next{node.cell.x + dx[d], node.cell.y + dy[d]};
            if (!isAllowedMove(map, node.cell, next, config.connectivity)) {
                continue;
            }
            const double length = (d % 2 == 1 ? std::sqrt(2.0) : 1.0) * res;
            const int move = std::max(1, steps_for(length / config.speed));
            const int earliest = node.arrive + turn_steps(node.direction, d);
            if (earliest > end_here) {
                continue;
            }
            const WorldPoint a = map.gridToWorld(node.cell), b = map.gridToWorld(next);
            const auto& there = safe.of(next);
            for (std::size_t j = 0; j < there.size(); ++j) {
                const int first = std::max(earliest, there[j].start - move);
                const int last = std::min(end_here, there[j].end == kForever ? kForever : there[j].end - move);
                if (first > last) {
                    continue;
                }
                // The earliest departure whose crossing stays safe at every step.
                int depart = -1;
                for (int dep = first; dep <= last && dep <= first + 400; ++dep) {
                    bool ok = true;
                    for (int k = dep + 1; k < dep + move && ok; ++k) {
                        const double s = static_cast<double>(k - dep) / move;
                        ok = safe.safeAt({a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s}, k);
                    }
                    if (ok) {
                        depart = dep;
                        break;
                    }
                }
                if (depart < 0) {
                    continue;
                }
                const int arrive = depart + move;
                const int direction = config.turn_aware ? d : 0;
                const long long k = key(next, static_cast<int>(j), direction);
                const auto it = best.find(k);
                if (it != best.end() && it->second <= arrive) {
                    continue;
                }
                best[k] = arrive;
                nodes.push_back({next, static_cast<int>(j), direction, arrive, depart, index, node.moves + 1});
                open.push({arrive + heuristic(next), node.moves + 1, static_cast<int>(nodes.size() - 1)});
            }
        }
    }
    if (found >= 0) {
        std::vector<int> chain;
        for (int i = found; i >= 0; i = nodes[static_cast<std::size_t>(i)].parent) {
            chain.push_back(i);
        }
        std::reverse(chain.begin(), chain.end());
        for (std::size_t i = 0; i < chain.size(); ++i) {
            const Node& n = nodes[static_cast<std::size_t>(chain[i])];
            const double depart = i + 1 < chain.size()
                                      ? nodes[static_cast<std::size_t>(chain[i + 1])].depart_parent * config.dt
                                      : n.arrive * config.dt;
            plan.waypoints.push_back({n.cell, n.arrive * config.dt, depart});
        }
        plan.success = true;
        plan.arrival = nodes[static_cast<std::size_t>(found)].arrive * config.dt;
    }
    plan.planning_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return plan;
}

}  // namespace amr
