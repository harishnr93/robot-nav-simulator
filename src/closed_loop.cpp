#include "closed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <random>

#include "diff_drive.hpp"
#include "footprint.hpp"
#include "surface_model.hpp"

namespace amr {

double obstacleDistance(const GridMap& map, WorldPoint p, double search_radius) {
    const double r = map.resolution();
    double best = search_radius;
    const int x0 = static_cast<int>(std::floor((p.x - search_radius) / r)) - 1;
    const int x1 = static_cast<int>(std::floor((p.x + search_radius) / r)) + 1;
    const int y0 = static_cast<int>(std::floor((p.y - search_radius) / r)) - 1;
    const int y1 = static_cast<int>(std::floor((p.y + search_radius) / r)) + 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            if (map.isFree({x, y})) {
                continue;  // outside the map counts as occupied
            }
            const double dx = std::max({x * r - p.x, 0.0, p.x - (x + 1) * r});
            const double dy = std::max({y * r - p.y, 0.0, p.y - (y + 1) * r});
            best = std::min(best, std::hypot(dx, dy));
        }
    }
    return best;
}

ClosedLoopResult simulateWithEstimate(const GridMap& map, const std::vector<GridCoord>& path,
                                      const ClosedLoopConfig& config, Connectivity connectivity) {
    (void)connectivity;
    ClosedLoopResult result;
    const MotionConfig& m = config.motion;
    if (path.empty() || !validateMotionConfig(m).empty()) {
        result.status = path.empty() ? NavigationStatus::InvalidPath : NavigationStatus::InvalidConfig;
        return result;
    }
    std::vector<WorldPoint> waypoints;
    for (const GridCoord& c : path) {
        waypoints.push_back(map.gridToWorld(c));
    }
    // Truth, odometry noise and the filter, as in runLocalization.
    RobotState truth{waypoints.front().x, waypoints.front().y, 0.0, wrapAngle(m.initial_heading)};
    std::mt19937 rng(config.ekf.odometry.seed);
    std::normal_distribution<double> unit(0.0, 1.0);
    const double ps = config.ekf.initial_position_sigma;
    const double hs = config.ekf.initial_heading_sigma;
    const Pose start{truth.x + ps * unit(rng), truth.y + ps * unit(rng), wrapAngle(truth.theta + hs * unit(rng))};
    const Matrix3 initial{ps * ps, 0.0, 0.0, 0.0, ps * ps, 0.0, 0.0, 0.0, hs * hs};
    const bool classic = config.ekf.model.classic();
    PoseEkf ekf(start, initial);
    const std::unique_ptr<SurfaceModel> surface = classic ? nullptr : makeSurface(map, config.ekf.model.surface);
    std::optional<AugmentedEkf> augmented;
    if (!classic) {
        augmented.emplace(start, initial, *surface, config.ekf.model, map.resolution());
    }
    const auto estimate = [&]() { return classic ? ekf.pose() : augmented->pose(); };
    const auto covariance = [&]() { return classic ? ekf.covariance() : augmented->covariance(); };
    Pose dead_reckoning = start;
    RangeErrorModel errors(config.lidar);
    const std::vector<double> offsets = beamOffsets(config.lidar);
    const auto scan_and_update = [&](double time) {
        if (!config.lidar.enabled()) {
            return UpdateStats{};
        }
        errors.nextScan(time);
        std::vector<double> ranges;
        std::vector<bool> hits;
        for (const double offset : offsets) {
            const RayHit ray = castRay(map, {truth.x, truth.y}, truth.theta + offset, config.lidar.max_range);
            ranges.push_back(ray.hit ? errors.measure(ray.range) : ray.range);
            hits.push_back(ray.hit);
        }
        return classic ? ekf.update(map, offsets, ranges, hits, config.lidar.max_range, config.ekf.range_sigma,
                                    config.ekf.gate)
                       : augmented->update(offsets, ranges, hits, config.lidar.max_range, config.ekf.range_sigma,
                                           config.ekf.gate);
    };
    const auto record = [&](double time, double v, double omega, UpdateStats stats) {
        result.trajectory.push_back({truth, 0, v, omega});
        LocalizationSample s;
        s.time = time;
        s.truth = {truth.x, truth.y, truth.theta};
        s.odometry = dead_reckoning;
        s.estimate = estimate();
        s.covariance = covariance();
        s.update = stats;
        result.localization.push_back(s);
        result.max_error = std::max(result.max_error, std::hypot(truth.x - s.estimate.x, truth.y - s.estimate.y));
        result.max_claimed_std =
            std::max(result.max_claimed_std, std::sqrt(std::max(s.covariance[0], s.covariance[4])));
        result.min_clearance =
            std::min(result.min_clearance, obstacleDistance(map, {truth.x, truth.y}, 2.0) - m.robot_radius);
    };
    result.min_clearance = std::numeric_limits<double>::infinity();
    record(0.0, 0.0, 0.0, scan_and_update(0.0));
    const auto max_steps = static_cast<std::size_t>(std::floor(m.max_time / m.dt + 1e-9));
    const double step_limit = m.speed * m.dt;
    std::size_t target = 1;
    std::size_t steps = 0;
    double held_since = -1.0;   // safety rule: when the current hold started
    double commit_left = 0.0;   // safety rule: meters still driven without holding
    while (true) {
        const Pose e = estimate();
        // Skip waypoints the estimate already stands on; finish at the goal.
        while (target < waypoints.size() - 1 &&
               std::hypot(waypoints[target].x - e.x, waypoints[target].y - e.y) <= 0.5 * step_limit) {
            ++target;
        }
        const WorldPoint goal = waypoints[std::min(target, waypoints.size() - 1)];
        const double distance = std::hypot(goal.x - e.x, goal.y - e.y);
        if (target >= waypoints.size() - 1 && distance <= m.goal_tolerance) {
            result.status = NavigationStatus::GoalReached;
            break;
        }
        if (steps >= max_steps) {
            result.status = NavigationStatus::Timeout;
            break;
        }
        const double error = wrapAngle(std::atan2(goal.y - e.y, goal.x - e.x) - e.theta);
        double v = 0.0, omega = 0.0;
        if (std::fabs(error) > config.drive_alignment) {
            omega = std::clamp(m.heading_gain * error, -m.max_angular_speed, m.max_angular_speed);
        } else {
            v = std::min(m.speed, distance / m.dt);
            omega = std::clamp(0.5 * m.heading_gain * error, -m.max_angular_speed, m.max_angular_speed);
        }
        if (config.safety_sigmas > 0.0 && v > 0.0 && commit_left <= 0.0) {
            const Matrix3 p = covariance();
            const double sigma = std::sqrt(std::max(p[0], p[4]));
            const WorldPoint ahead{e.x + v * m.dt * std::cos(e.theta), e.y + v * m.dt * std::sin(e.theta)};
            if (obstacleDistance(map, ahead, 2.0) - m.robot_radius < config.safety_sigmas * sigma) {
                if (held_since < 0.0) {
                    held_since = static_cast<double>(steps) * m.dt;
                }
                if (static_cast<double>(steps) * m.dt - held_since < config.safety_patience - 1e-9) {
                    v = 0.0;  // hold: let the scans bring the uncertainty down first
                    ++result.safety_holds;
                } else {
                    commit_left = config.safety_commit;  // waited enough: drive on
                    held_since = -1.0;
                }
            } else {
                held_since = -1.0;
            }
        }
        commit_left -= v * m.dt;
        // The true robot moves as commanded.
        const RobotState before = truth;
        const Pose next = integrateUnicycle({truth.x, truth.y, truth.theta}, v, omega, m.dt);
        ++steps;
        const double time = static_cast<double>(steps) * m.dt;
        truth = {next.x, next.y, time, next.theta};
        if (sweptDiscOverlapsObstacle(map, {before.x, before.y}, {truth.x, truth.y}, m.robot_radius) ||
            !map.isFree(map.worldToGrid({truth.x, truth.y}))) {
            result.status = NavigationStatus::Collision;
            record(time, v, omega, {});
            break;
        }
        // Odometry of the true motion, then the filter.
        const OdometryIncrement increment = trueIncrement(before, truth);
        const std::array<double, 2> var = odometryVariances({v * m.dt, omega * m.dt}, config.ekf.odometry);
        const OdometryIncrement measured{increment.distance + std::sqrt(var[0]) * unit(rng),
                                         increment.rotation + std::sqrt(var[1]) * unit(rng)};
        dead_reckoning = applyIncrement(dead_reckoning, measured);
        if (classic) {
            ekf.predict(measured, var);
        } else {
            augmented->predict(measured, var, m.dt);
        }
        record(time, v, omega, scan_and_update(time));
    }
    result.final_goal_error = std::hypot(truth.x - waypoints.back().x, truth.y - waypoints.back().y);
    return result;
}

}  // namespace amr
