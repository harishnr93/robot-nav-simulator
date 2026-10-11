#include "simulator.hpp"

#include <algorithm>
#include <cmath>

#include "astar.hpp"

namespace amr {

namespace {

double distanceBetween(WorldPoint a, WorldPoint b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

/// Structural path check. Occupancy is deliberately NOT checked here: entering an
/// occupied cell is detected by the motion-time collision checker instead.
bool isStructurallyValidPath(const GridMap& map, const std::vector<GridCoord>& path) {
    if (path.empty()) {
        return false;
    }
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!map.inBounds(path[i])) {
            return false;
        }
        if (i > 0 && manhattanDistance(path[i - 1], path[i]) != 1) {
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
                              const MotionConfig& config) {
    SimulationResult result;
    if (!validateMotionConfig(config).empty()) {
        result.status = NavigationStatus::InvalidConfig;
        return result;
    }
    if (!isStructurallyValidPath(map, path)) {
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
    std::size_t target = skipReachedWaypoints(waypoints, 1, position);
    auto record = [&](double time) {
        const std::size_t recorded_target = target < waypoints.size() ? target : last_index;
        result.trajectory.push_back({RobotState{position.x, position.y, time}, recorded_target});
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
    if (!isSegmentCollisionFree(map, position, position)) {
        result.collision_count = 1;
        result.status = NavigationStatus::Collision;
    }

    while (result.status == NavigationStatus::NotRun && target < waypoints.size()) {
        if (result.steps >= max_steps) {
            result.status = NavigationStatus::Timeout;
            break;
        }
        const StepResult step = stepToward(position, waypoints[target], step_limit);
        if (!isSegmentCollisionFree(map, position, step.position)) {
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
        record(static_cast<double>(result.steps) * config.dt);
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
