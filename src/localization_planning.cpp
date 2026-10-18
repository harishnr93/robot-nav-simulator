#include "localization_planning.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

#include "astar.hpp"
#include "ekf.hpp"

namespace amr {

namespace {

constexpr double kMinIncidenceCosine = 0.2;  // as in the classic EKF

}  // namespace

std::vector<double> computeLocalizability(const GridMap& map, const LocalizabilityConfig& config) {
    LidarConfig lidar;
    lidar.beams = config.beams;
    lidar.max_range = config.max_range;
    const std::vector<double> offsets = beamOffsets(lidar);
    const double inv_r = 1.0 / (config.range_sigma * config.range_sigma);
    const double prior = 1.0 / (config.prior_sigma * config.prior_sigma);
    std::vector<double> result(map.cellCount(), std::numeric_limits<double>::infinity());
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (!map.isFree({x, y})) {
                continue;
            }
            const WorldPoint c = map.gridToWorld({x, y});
            // Information of one scan (3 x 3) plus a weak prior on every axis.
            double info[9] = {prior, 0, 0, 0, prior, 0, 0, 0, prior};
            for (const double a : offsets) {
                const RayHit hit = castRay(map, c, a, config.max_range);
                if (!hit.hit || hit.range <= 0.0) {
                    continue;
                }
                const double incidence = hit.face == RayFace::Vertical     ? std::cos(a)
                                         : hit.face == RayFace::Horizontal ? std::sin(a)
                                                                           : 0.0;
                if (std::fabs(incidence) < kMinIncidenceCosine) {
                    continue;
                }
                const std::array<double, 3> h = rangeJacobian(hit.face, a, hit.range);
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        info[3 * i + j] += h[static_cast<std::size_t>(i)] * h[static_cast<std::size_t>(j)] * inv_r;
                    }
                }
            }
            // Position block of the inverse = inverse of the Schur complement
            // S = I_pp - I_pt I_tt^-1 I_tp (2 x 2).
            const double s00 = info[0] - info[2] * info[6] / info[8];
            const double s01 = info[1] - info[2] * info[7] / info[8];
            const double s11 = info[4] - info[5] * info[7] / info[8];
            // Largest eigenvalue of S^-1 = 1 / smallest eigenvalue of S.
            const double mean = 0.5 * (s00 + s11);
            const double root = std::sqrt(0.25 * (s00 - s11) * (s00 - s11) + s01 * s01);
            const double smallest = std::max(mean - root, 1e-12);
            result[static_cast<std::size_t>(y * map.width() + x)] = std::sqrt(1.0 / smallest);
        }
    }
    return result;
}

double localizationPenalty(double localizability, const LocalizationPlanConfig& config) {
    if (!std::isfinite(localizability)) {
        return config.max_penalty;
    }
    return std::clamp(std::log2(localizability / config.good), 0.0, config.max_penalty);
}

PlanResult planForLocalization(const GridMap& map, GridCoord start, GridCoord goal,
                               const std::vector<double>& localizability,
                               const LocalizationPlanConfig& config) {
    const auto t0 = std::chrono::steady_clock::now();
    PlanResult result;
    result.status = validatePlanRequest(map, start, goal);
    if (result.status != PlanStatus::Success) {
        return result;
    }
    result.status = PlanStatus::NoPath;
    const int w = map.width();
    const auto index = [w](GridCoord c) { return static_cast<std::size_t>(c.y * w + c.x); };
    std::vector<double> g(map.cellCount(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(map.cellCount(), -1);
    std::vector<bool> closed(map.cellCount(), false);
    const auto heuristic = [&](GridCoord c) {
        const double dx = std::abs(c.x - goal.x), dy = std::abs(c.y - goal.y);
        return (std::max(dx, dy) - std::min(dx, dy)) + std::min(dx, dy) * std::sqrt(2.0);
    };
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    g[index(start)] = 0.0;
    open.push({heuristic(start), static_cast<int>(index(start))});
    while (!open.empty()) {
        const int current = open.top().second;
        open.pop();
        if (closed[static_cast<std::size_t>(current)]) {
            continue;
        }
        closed[static_cast<std::size_t>(current)] = true;
        ++result.expanded_nodes;
        const GridCoord c{current % w, current / w};
        if (c == goal) {
            break;
        }
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const GridCoord n{c.x + dx, c.y + dy};
                if ((dx == 0 && dy == 0) || !isAllowedMove(map, c, n, Connectivity::Eight)) {
                    continue;
                }
                const double length = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
                const double step =
                    length * (1.0 + config.weight * localizationPenalty(localizability[index(n)], config));
                const double candidate = g[static_cast<std::size_t>(current)] + step;
                if (candidate < g[index(n)]) {
                    g[index(n)] = candidate;
                    parent[index(n)] = current;
                    open.push({candidate + heuristic(n), static_cast<int>(index(n))});
                }
            }
        }
    }
    if (closed[index(goal)]) {
        for (int i = static_cast<int>(index(goal)); i >= 0; i = parent[static_cast<std::size_t>(i)]) {
            result.path.push_back({i % w, i / w});
        }
        std::reverse(result.path.begin(), result.path.end());
        result.status = PlanStatus::Success;
        result.path_cost = static_cast<int>(result.path.size()) - 1;
        for (std::size_t k = 1; k < result.path.size(); ++k) {
            const bool diagonal = result.path[k].x != result.path[k - 1].x && result.path[k].y != result.path[k - 1].y;
            result.diagonal_moves += diagonal ? 1 : 0;
        }
    }
    result.planning_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return result;
}

}  // namespace amr
