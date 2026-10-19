#include "closed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <random>

#include "diff_drive.hpp"
#include "dynamic_obstacles.hpp"
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
    waypoints.reserve(path.size());
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
    // Cross-track error: the true center's distance to the planned polyline.
    const auto cross_track = [&]() {
        double best = std::hypot(truth.x - waypoints.front().x, truth.y - waypoints.front().y);
        for (std::size_t i = 1; i < waypoints.size(); ++i) {
            best = std::min(best, pointSegmentDistance({truth.x, truth.y}, waypoints[i - 1], waypoints[i]));
        }
        return best;
    };
    const auto record_cross_track = [&]() {
        const double e = cross_track();
        result.cross_track.push_back(e);
        result.max_cross_track = std::max(result.max_cross_track, e);
    };
    // The actuators: actual speed and turn rate, and the correlated disturbance.
    const ActuatorConfig& act = config.actuator;
    std::mt19937 actuator_rng(act.seed);
    double actual_v = 0.0, actual_omega = 0.0, disturbance = 0.0;
    double integral = 0.0;  // line follower: integral of the cross-track error
    result.min_clearance = std::numeric_limits<double>::infinity();
    record(0.0, 0.0, 0.0, scan_and_update(0.0));
    record_cross_track();
    const auto max_steps = static_cast<std::size_t>(std::floor(m.max_time / m.dt + 1e-9));
    const double step_limit = m.speed * m.dt;
    // The first waypoint from i on where the path turns (or the goal).
    const auto cornerAfter = [&](std::size_t i) {
        for (std::size_t j = i; j + 1 < waypoints.size(); ++j) {
            const double cross = (waypoints[j].x - waypoints[j - 1].x) * (waypoints[j + 1].y - waypoints[j].y) -
                                 (waypoints[j].y - waypoints[j - 1].y) * (waypoints[j + 1].x - waypoints[j].x);
            if (std::fabs(cross) > 1e-12) {
                return j;
            }
        }
        return waypoints.size() - 1;
    };
    std::size_t target = 1;
    std::size_t steps = 0;
    double held_since = -1.0;   // safety rule: when the current hold started
    double commit_left = 0.0;   // safety rule: meters still driven without holding
    while (true) {
        const Pose e = config.steer_by_truth ? Pose{truth.x, truth.y, truth.theta} : estimate();
        // Skip waypoints the estimate already stands on; finish at the goal. The line
        // follower also passes a waypoint once its projection on the segment has gone
        // beyond it (it may pass it a little to the side).
        const auto passed = [&](std::size_t i) {
            if (std::hypot(waypoints[i].x - e.x, waypoints[i].y - e.y) <= 0.5 * step_limit) {
                return true;
            }
            if (config.tracker != PathTracker::LineFollow || i == 0) {
                return false;
            }
            const WorldPoint a = waypoints[i - 1], b = waypoints[i];
            return (e.x - b.x) * (b.x - a.x) + (e.y - b.y) * (b.y - a.y) >= 0.0;
        };
        while (target < waypoints.size() - 1 && passed(target)) {
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
        // The line follower past the end of its segment (it overshot the goal) aims
        // at the waypoint like the turn-then-drive rule.
        const bool beyond = target > 0 && (e.x - goal.x) * (goal.x - waypoints[target - 1].x) +
                                                  (e.y - goal.y) * (goal.y - waypoints[target - 1].y) >
                                              0.0;
        if (config.tracker == PathTracker::LineFollow && target > 0 && !beyond) {
            // Follow the segment from the previous waypoint to the target.
            const WorldPoint a = waypoints[target - 1];
            const double length = std::hypot(goal.x - a.x, goal.y - a.y);
            const double direction = length > 1e-9 ? std::atan2(goal.y - a.y, goal.x - a.x) : e.theta;
            const double heading_error = wrapAngle(direction - e.theta);
            // Signed cross-track error, positive to the left of the segment.
            const double cross = length > 1e-9 ? ((e.x - a.x) * -std::sin(direction) + (e.y - a.y) * std::cos(direction))
                                                : 0.0;
            if (std::fabs(heading_error) > config.drive_alignment && std::fabs(error) > config.drive_alignment) {
                omega = std::clamp(m.heading_gain * heading_error, -m.max_angular_speed, m.max_angular_speed);
                integral = 0.0;
            } else {
                const LineFollowGains& g = config.line_follow;
                // Brake ahead of a corner or the goal: a lagging drive keeps rolling for
                // about its time constant after the command drops.
                const double to_turn = std::hypot(waypoints[cornerAfter(target)].x - e.x,
                                                  waypoints[cornerAfter(target)].y - e.y);
                v = std::min({m.speed, distance / m.dt, to_turn / (m.dt + g.stop_lead)});
                integral = std::clamp(integral + cross * m.dt, -g.max_integral, g.max_integral);
                omega = std::clamp(g.heading * heading_error - g.cross * cross - g.integral * integral,
                                   -m.max_angular_speed, m.max_angular_speed);
            }
        } else if (std::fabs(error) > config.drive_alignment) {
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
        // The true robot moves as the actuators make it (as commanded if ideal).
        const RobotState before = truth;
        double move_v = v, move_omega = omega;
        if (!act.ideal()) {
            if (act.rate_noise > 0.0) {  // Gauss-Markov, exact discretisation
                const double phi = std::exp(-m.dt / act.noise_time);
                disturbance = phi * disturbance +
                              act.rate_noise * std::sqrt(1.0 - phi * phi) * std::normal_distribution<double>()(actuator_rng);
            }
            const double target_v = act.speed_scale * v;
            const double target_omega = omega + act.turn_bias + disturbance;
            const double keep = act.lag > 0.0 ? std::exp(-m.dt / act.lag) : 0.0;
            const double next_v = target_v + (actual_v - target_v) * keep;
            const double next_omega = target_omega + (actual_omega - target_omega) * keep;
            move_v = 0.5 * (actual_v + next_v);  // the step's mean (exact for a linear change)
            move_omega = 0.5 * (actual_omega + next_omega);
            actual_v = next_v;
            actual_omega = next_omega;
        }
        const Pose next = integrateUnicycle({truth.x, truth.y, truth.theta}, move_v, move_omega, m.dt);
        ++steps;
        const double time = static_cast<double>(steps) * m.dt;
        truth = {next.x, next.y, time, next.theta};
        if (sweptDiscOverlapsObstacle(map, {before.x, before.y}, {truth.x, truth.y}, m.robot_radius) ||
            !map.isFree(map.worldToGrid({truth.x, truth.y}))) {
            result.status = NavigationStatus::Collision;
            record(time, move_v, move_omega, {});
            record_cross_track();
            break;
        }
        // Odometry of the true motion, then the filter.
        const OdometryIncrement increment = trueIncrement(before, truth);
        const std::array<double, 2> var =
            odometryVariances({move_v * m.dt, move_omega * m.dt}, config.ekf.odometry);
        const OdometryIncrement measured{increment.distance + std::sqrt(var[0]) * unit(rng),
                                         increment.rotation + std::sqrt(var[1]) * unit(rng)};
        dead_reckoning = applyIncrement(dead_reckoning, measured);
        if (classic) {
            ekf.predict(measured, var);
        } else {
            augmented->predict(measured, var, m.dt);
        }
        record(time, move_v, move_omega, scan_and_update(time));
        record_cross_track();
    }
    result.final_goal_error = std::hypot(truth.x - waypoints.back().x, truth.y - waypoints.back().y);
    return result;
}

}  // namespace amr
