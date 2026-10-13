#include "simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "footprint.hpp"

namespace amr {

namespace {

double distanceBetween(WorldPoint a, WorldPoint b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

/// Structural path check. Occupancy is deliberately NOT checked here: entering an
/// occupied cell (or touching one while cutting a corner) is detected by the
/// motion-time collision checker instead.
bool isStructurallyValidPath(const GridMap& map, const std::vector<GridCoord>& path,
                             Connectivity connectivity) {
    if (path.empty()) {
        return false;
    }
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!map.inBounds(path[i])) {
            return false;
        }
        if (i == 0) {
            continue;
        }
        const int dx = std::abs(path[i].x - path[i - 1].x);
        const int dy = std::abs(path[i].y - path[i - 1].y);
        const bool straight = dx + dy == 1;
        const bool diagonal = dx == 1 && dy == 1;
        if (!straight && !(diagonal && connectivity == Connectivity::Eight)) {
            return false;
        }
    }
    return true;
}

/// Skip waypoints the robot is already standing on (zero-distance waypoints).
std::size_t skipReachedWaypoints(const std::vector<WorldPoint>& waypoints,
                                 std::size_t target, WorldPoint position) {
    while (target < waypoints.size() &&
           distanceBetween(position, waypoints[target]) <= kArrivalEpsilon) {
        ++target;
    }
    return target;
}

}  // namespace

double SimulationResult::travelTime() const {
    if (trajectory.empty()) {
        return 0.0;
    }
    return trajectory.back().state.time - trajectory.front().state.time;
}

bool isSegmentCollisionFree(const GridMap& map, WorldPoint a, WorldPoint b) {
    const double length = distanceBetween(a, b);
    const double spacing = map.resolution() / kCollisionSamplesPerCell;
    const auto intervals = static_cast<long>(std::ceil(length / spacing));
    for (long i = 0; i <= intervals; ++i) {
        const double t =
            intervals == 0 ? 0.0 : static_cast<double>(i) / static_cast<double>(intervals);
        const WorldPoint p{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (!map.isFree(map.worldToGrid(p))) {
            return false;
        }
    }
    return true;
}

SimulationResult simulatePath(const GridMap& map, const std::vector<GridCoord>& path,
                              const MotionConfig& config, Connectivity connectivity) {
    SimulationResult result;
    if (!validateMotionConfig(config).empty()) {
        result.status = NavigationStatus::InvalidConfig;
        return result;
    }
    if (!isStructurallyValidPath(map, path, connectivity)) {
        result.status = NavigationStatus::InvalidPath;
        return result;
    }

    std::vector<WorldPoint> waypoints;
    waypoints.reserve(path.size());
    for (const GridCoord& cell : path) {
        waypoints.push_back(map.gridToWorld(cell));
    }
    const std::size_t last_index = waypoints.size() - 1;
    const WorldPoint goal = waypoints.back();
    const double step_limit = config.speed * config.dt;
    // Floor with a small epsilon so e.g. 600 / 0.1 gives 6000 steps, not 5999.
    const auto max_steps =
        static_cast<std::size_t>(std::floor(config.max_time / config.dt + 1e-9));

    WorldPoint position = waypoints.front();
    const bool diff_drive = config.model == RobotModel::DiffDrive;
    double theta = diff_drive ? wrapAngle(config.initial_heading) : 0.0;
    std::size_t target = skipReachedWaypoints(waypoints, 1, position);
    auto record = [&](double time, double v = 0.0, double omega = 0.0) {
        const std::size_t recorded_target = target < waypoints.size() ? target : last_index;
        result.trajectory.push_back(
            {RobotState{position.x, position.y, time, theta}, recorded_target, v, omega});
    };

    // Exact number of samples for an uninterrupted run: each leg takes
    // ceil(leg length / step) steps (the last one clamped), plus the t = 0 sample.
    std::size_t expected_steps = 0;
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        const double leg = distanceBetween(waypoints[i - 1], waypoints[i]);
        expected_steps += static_cast<std::size_t>(std::ceil(leg / step_limit - kArrivalEpsilon));
    }
    result.trajectory.reserve(std::min(expected_steps, max_steps) + 1);

    record(0.0);
    // Point robot: the Phase 3 point check; with 8-connectivity the step must also
    // not touch an occupied cell (a diagonal move through the shared corner of an
    // obstacle touches it). Disc robot: exact swept-disc check.
    const auto step_is_free = [&](WorldPoint from, WorldPoint to) {
        if (config.robot_radius > 0.0) {
            return !sweptDiscOverlapsObstacle(map, from, to, config.robot_radius);
        }
        return isSegmentCollisionFree(map, from, to) &&
               (connectivity == Connectivity::Four || !segmentTouchesObstacle(map, from, to));
    };

    if (!step_is_free(position, position)) {
        result.collision_count = 1;
        result.status = NavigationStatus::Collision;
    }

    bool turning = false;
    while (result.status == NavigationStatus::NotRun && target < waypoints.size()) {
        if (result.steps >= max_steps) {
            result.status = NavigationStatus::Timeout;
            break;
        }
        if (diff_drive) {
            // Turn-then-drive controller. While the heading is off the direction to
            // the next waypoint, rotate in place (v = 0) under the P heading
            // controller. Once aligned, drive straight to the waypoint (omega = 0),
            // so the robot's center follows the planned, collision-checked segments.
            const WorldPoint& waypoint = waypoints[target];
            const double desired = std::atan2(waypoint.y - position.y, waypoint.x - position.x);
            const double error = wrapAngle(desired - theta);
            if (std::fabs(error) > kDriveAlignment) {
                const double omega = headingControl(error, config.heading_gain,
                                                    config.max_angular_speed,
                                                    config.heading_tolerance, config.dt);
                theta = integrateUnicycle({position.x, position.y, theta}, 0.0, omega,
                                          config.dt).theta;
                // A disc rotating in place sweeps no new area: no collision check.
                result.turns += turning ? 0 : 1;
                turning = true;
                result.turn_time += config.dt;
                result.total_rotation += std::fabs(omega) * config.dt;
                ++result.steps;
                record(static_cast<double>(result.steps) * config.dt, 0.0, omega);
                continue;
            }
            turning = false;
            theta = desired;  // residual error <= kDriveAlignment is absorbed
        }
        const StepResult step = stepToward(position, waypoints[target], step_limit);
        if (!step_is_free(position, step.position)) {
            // The unsafe step is not executed; the robot stops at its last safe pose.
            result.collision_count = 1;
            result.status = NavigationStatus::Collision;
            break;
        }
        position = step.position;
        result.distance_traveled += step.distance;
        ++result.steps;
        if (step.reached) {
            target = skipReachedWaypoints(waypoints, target + 1, position);
        }
        // Time is computed from the step count (not accumulated) to avoid drift.
        record(static_cast<double>(result.steps) * config.dt,
               diff_drive ? step.distance / config.dt : 0.0, 0.0);
    }

    result.final_goal_error = distanceBetween(position, goal);
    if (result.status == NavigationStatus::NotRun) {
        result.status = result.final_goal_error <= config.goal_tolerance
                            ? NavigationStatus::GoalReached
                            : NavigationStatus::Timeout;
    }
    return result;
}

}  // namespace amr
