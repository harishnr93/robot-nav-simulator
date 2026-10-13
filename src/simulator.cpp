#include "simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "footprint.hpp"

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;

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

/// What the robot knows about dynamic obstacles at one instant, and the planning
/// map with the cells they block.
class DynamicView {
public:
    DynamicView(const DynamicScenario& scenario, double robot_radius, double dt)
        : scenario_(scenario), robot_radius_(robot_radius), dt_(dt),
          working_(*scenario.planning_map), last_seen_(scenario.obstacles.size()) {}

    /// Look around from `robot` at time t: find the visible obstacles, estimate their
    /// velocities and block the planning cells near their predicted sweeps (the
    /// previous step's blocks are lifted first). Called once per step.
    void observe(const GridMap& map, WorldPoint robot, double t) {
        for (const GridCoord& cell : blocked_) {
            working_.setOccupied(cell, false);
        }
        blocked_.clear();
        seen_.clear();
        const double horizon = scenario_.replanning.prediction_horizon;
        for (std::size_t i = 0; i < scenario_.obstacles.size(); ++i) {
            const DynamicObstacle& obstacle = scenario_.obstacles[i];
            const WorldPoint center = obstaclePosition(obstacle, t);
            if (!isObstacleVisible(map, robot, center, obstacle.radius,
                                   scenario_.replanning.sensing_range)) {
                last_seen_[i].reset();
                continue;
            }
            // Velocity from the previous step's sighting (none on first sight).
            WorldPoint velocity{0.0, 0.0};
            if (last_seen_[i]) {
                velocity = {(center.x - last_seen_[i]->x) / dt_,
                            (center.y - last_seen_[i]->y) / dt_};
            }
            last_seen_[i] = center;
            seen_.push_back({center, velocity, obstacle.radius});
            const WorldPoint ahead{center.x + velocity.x * horizon,
                                   center.y + velocity.y * horizon};
            block(center, ahead, obstacle.radius + robot_radius_ + scenario_.replanning.margin);
        }
    }

    /// Smallest predicted gap between the robot and the seen obstacles' edges when
    /// the robot is at robot[j] at time j dt from now (robot[0] = now) and every
    /// obstacle keeps its estimated velocity. Each step is checked exactly.
    double predictedGap(const std::vector<WorldPoint>& robot) const {
        return predict(robot).gap;
    }

    /// Prediction for a robot motion as in predictedGap(): the smallest gap and the
    /// number of steps before the first predicted contact (robot.size() if none).
    struct Prediction {
        double gap;
        std::size_t steps_to_contact;
    };
    Prediction predict(const std::vector<WorldPoint>& robot) const {
        Prediction result{std::numeric_limits<double>::infinity(), robot.size()};
        for (const Seen& s : seen_) {
            for (std::size_t j = 1; j < robot.size(); ++j) {
                const double t0 = static_cast<double>(j - 1) * dt_;
                const double t1 = static_cast<double>(j) * dt_;
                const WorldPoint q0{s.center.x + s.velocity.x * t0, s.center.y + s.velocity.y * t0};
                const WorldPoint q1{s.center.x + s.velocity.x * t1, s.center.y + s.velocity.y * t1};
                const double gap = closestApproach(robot[j - 1], robot[j], q0, q1) - s.radius -
                                   robot_radius_;
                result.gap = std::min(result.gap, gap);
                if (gap < 0.0) {
                    result.steps_to_contact = std::min(result.steps_to_contact, j - 1);
                }
            }
        }
        return result;
    }

    GridMap& planningMap() { return working_; }
    double dt() const { return dt_; }
    std::size_t blockedCount() const { return blocked_.size(); }

private:
    struct Seen {
        WorldPoint center;
        WorldPoint velocity;  ///< estimated, m/s
        double radius;
    };

    /// Block the free cells whose centers are closer than `reach` to segment a-b
    /// (the same cell-center rule as obstacle inflation).
    void block(WorldPoint a, WorldPoint b, double reach) {
        const double res = working_.resolution();
        const int x0 = std::max(static_cast<int>(std::floor((std::min(a.x, b.x) - reach) / res)), 0);
        const int x1 = std::min(static_cast<int>(std::floor((std::max(a.x, b.x) + reach) / res)),
                                working_.width() - 1);
        const int y0 = std::max(static_cast<int>(std::floor((std::min(a.y, b.y) - reach) / res)), 0);
        const int y1 = std::min(static_cast<int>(std::floor((std::max(a.y, b.y) + reach) / res)),
                                working_.height() - 1);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const GridCoord cell{x, y};
                if (working_.isFree(cell) &&
                    pointSegmentDistance(working_.gridToWorld(cell), a, b) < reach) {
                    working_.setOccupied(cell, true);
                    blocked_.push_back(cell);
                }
            }
        }
    }

    const DynamicScenario& scenario_;
    double robot_radius_;
    double dt_;
    GridMap working_;                // planning map plus the cells blocked by seen obstacles
    std::vector<GridCoord> blocked_; // cells set in working_ by the last observe()
    std::vector<Seen> seen_;
    std::vector<std::optional<WorldPoint>> last_seen_;  // per obstacle, previous step
};

/// Positions of a robot that starts at `position` and keeps following the
/// waypoints from `target` on, one entry per step (entry 0 = now), `steps` steps.
std::vector<WorldPoint> predictMotion(const std::vector<WorldPoint>& waypoints, std::size_t target,
                                      WorldPoint position, double step_limit, std::size_t steps) {
    std::vector<WorldPoint> path{position};
    for (std::size_t j = 0; j < steps; ++j) {
        if (target < waypoints.size()) {
            const StepResult step = stepToward(position, waypoints[target], step_limit);
            position = step.position;
            if (step.reached) {
                ++target;
            }
        }
        path.push_back(position);
    }
    return path;
}

/// Look-ahead gate: true unless following the path for the prediction horizon
/// would come within `margin` of a seen obstacle (moving at its estimated
/// velocity) while standing still would keep a larger gap.
bool moveIsSafe(const DynamicView& view, const std::vector<WorldPoint>& waypoints,
                std::size_t target, WorldPoint position, double step_limit,
                const ReplanningConfig& config) {
    const auto steps = static_cast<std::size_t>(
        std::max(1.0, std::ceil(config.prediction_horizon / view.dt() - 1e-9)));
    const double moving = view.predictedGap(predictMotion(waypoints, target, position,
                                                          step_limit, steps));
    if (moving >= config.margin) {
        return true;
    }
    const double standing = view.predictedGap(std::vector<WorldPoint>(steps + 1, position));
    return moving > standing;
}

/// Evasive step for a robot that would otherwise stand still (turn in place):
/// if standing still is predicted to end in contact, returns the position one step
/// forward or backward along `theta`, whichever (if either) does better when the
/// robot keeps going that way: a later contact, or a larger gap. Otherwise
/// `position` itself.
template <typename StepIsFree>
WorldPoint evasiveStep(const DynamicView& view, const GridMap& map, WorldPoint position,
                       double theta, double step_limit, const ReplanningConfig& config,
                       const StepIsFree& step_is_free) {
    const auto steps = static_cast<std::size_t>(
        std::max(1.0, std::ceil(config.prediction_horizon / view.dt() - 1e-9)));
    DynamicView::Prediction best = view.predict(std::vector<WorldPoint>(steps + 1, position));
    if (best.gap >= 0.0) {
        return position;
    }
    // Prefer the option that delays the contact longest (when it cannot be avoided
    // within the horizon, the gap alone does not tell the options apart), then the
    // larger gap.
    const auto better = [](const DynamicView::Prediction& a, const DynamicView::Prediction& b) {
        return a.steps_to_contact != b.steps_to_contact ? a.steps_to_contact > b.steps_to_contact
                                                        : a.gap > b.gap;
    };
    WorldPoint chosen = position;
    for (const double direction : {1.0, -1.0}) {
        const WorldPoint candidate{position.x + direction * step_limit * std::cos(theta),
                                   position.y + direction * step_limit * std::sin(theta)};
        if (!map.isFree(map.worldToGrid(candidate)) || !step_is_free(position, candidate)) {
            continue;
        }
        // Keep stepping in that direction while the cells stay free.
        std::vector<WorldPoint> motion{position};
        for (std::size_t j = 1; j <= steps; ++j) {
            const WorldPoint& last = motion.back();
            const WorldPoint further{last.x + direction * step_limit * std::cos(theta),
                                     last.y + direction * step_limit * std::sin(theta)};
            motion.push_back(map.isFree(map.worldToGrid(further)) && step_is_free(last, further)
                                 ? further
                                 : last);
        }
        const DynamicView::Prediction option = view.predict(motion);
        if (better(option, best)) {
            best = option;
            chosen = candidate;
        }
    }
    return chosen;
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

std::string validateReplanningConfig(const ReplanningConfig& config) {
    if (!std::isfinite(config.sensing_range) || config.sensing_range <= 0.0 ||
        config.sensing_range > kMaxSensingRange) {
        return "sensing range must be in (0, 1000] m";
    }
    if (!std::isfinite(config.margin) || config.margin < 0.0 || config.margin > kMaxRobotRadius) {
        return "replanning margin must be in [0, 100] m";
    }
    if (!std::isfinite(config.prediction_horizon) || config.prediction_horizon < 0.0 ||
        config.prediction_horizon > kMaxPredictionHorizon) {
        return "prediction horizon must be in [0, 100] s";
    }
    if (!std::isfinite(config.patience) || config.patience < 0.0 ||
        config.patience > kMaxPredictionHorizon) {
        return "replanning patience must be in [0, 100] s";
    }
    return {};
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
                              const MotionConfig& config, Connectivity connectivity,
                              const DynamicScenario* dynamic) {
    SimulationResult result;
    if (!validateMotionConfig(config).empty() ||
        (dynamic && (dynamic->planning_map == nullptr ||
                     !validateReplanningConfig(dynamic->replanning).empty()))) {
        result.status = NavigationStatus::InvalidConfig;
        return result;
    }
    if (!isStructurallyValidPath(map, path, connectivity)) {
        result.status = NavigationStatus::InvalidPath;
        return result;
    }

    // With replanning the remaining waypoints change; `cells` stays parallel to them.
    std::vector<GridCoord> cells = path;
    std::vector<WorldPoint> waypoints;
    waypoints.reserve(path.size());
    for (const GridCoord& cell : path) {
        waypoints.push_back(map.gridToWorld(cell));
    }
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
        const std::size_t last = waypoints.size() - 1;
        const std::size_t recorded_target = target < waypoints.size() ? target : last;
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

    // Dynamic obstacles: exact contact test over one step (robot linear a -> b).
    std::optional<DynamicView> view;
    AStarPlanner planner;  // one planner for every replan: its buffers are reused
    if (dynamic) {
        view.emplace(*dynamic, config.robot_radius, config.dt);
        result.min_dynamic_clearance = std::numeric_limits<double>::infinity();
    }
    const auto hits_obstacle = [&](WorldPoint a, WorldPoint b, double t0, double t1) {
        for (std::size_t i = 0; i < dynamic->obstacles.size(); ++i) {
            const DynamicObstacle& obstacle = dynamic->obstacles[i];
            const double gap = minDistanceDuring(obstacle, a, b, t0, t1) - obstacle.radius -
                               config.robot_radius;
            result.min_dynamic_clearance = std::min(result.min_dynamic_clearance, gap);
            if (gap < -kFootprintTolerance) {
                result.collided_obstacle = static_cast<int>(i);
                return true;
            }
        }
        return false;
    };

    if (!step_is_free(position, position) ||
        (dynamic && hits_obstacle(position, position, 0.0, 0.0))) {
        result.collision_count = 1;
        result.status = NavigationStatus::Collision;
    }

    bool turning = false;
    double blocked_since = -1.0;  // dynamic: when the current path became blocked
    constexpr std::size_t kNoTarget = static_cast<std::size_t>(-1);
    std::size_t reverse_target = kNoTarget;  // dynamic: retreat waypoint, driven backwards
    while (result.status == NavigationStatus::NotRun && target < waypoints.size()) {
        if (result.steps >= max_steps) {
            result.status = NavigationStatus::Timeout;
            break;
        }
        const double now = static_cast<double>(result.steps) * config.dt;
        const double next = static_cast<double>(result.steps + 1) * config.dt;
        bool wait = false;
        if (dynamic) {
            view->observe(map, position, now);
            GridMap& planning = view->planningMap();
            // The goal itself never counts as blocked (it cannot be avoided, only
            // timed): the look-ahead gate decides when to drive into it.
            planning.setOccupied(cells.back(), false);
            const bool blocked = std::any_of(cells.begin() + static_cast<long>(target), cells.end(),
                                             [&](GridCoord c) { return planning.isOccupied(c); });
            if (!blocked) {
                blocked_since = -1.0;
            } else {
                if (blocked_since < 0.0) {
                    blocked_since = now;
                }
                const bool at_cell = distanceBetween(position, waypoints[target - 1]) <= kArrivalEpsilon;
                const bool target_blocked = planning.isOccupied(cells[target]);
                // Act at once when the robot itself is threatened (its own cell, or
                // the cell it is entering, is blocked) or the blockage outlasted the
                // patience. Until then it follows the free part of the path and waits
                // at the last free waypoint: a moving obstacle usually clears the
                // way, while detouring at once makes the robot chase the crossing the
                // obstacle is about to block, back and forth beside it.
                const bool urgent = (at_cell && planning.isOccupied(cells[target - 1])) ||
                                    (!at_cell && target_blocked) ||
                                    now - blocked_since >= dynamic->replanning.patience - 1e-9;
                if (!urgent) {
                    wait = at_cell && target_blocked;
                } else {
                    // Replan from the current cell if the robot stands on a waypoint,
                    // otherwise from the waypoint it is driving to. If that waypoint
                    // is blocked, retreat: replan from the waypoint it came from
                    // (statically free, and just left), driving back to it first.
                    const bool retreat = !at_cell && target_blocked;
                    const std::size_t from = at_cell || retreat ? target - 1 : target;
                    planning.setOccupied(cells[from], false);  // the robot's own cell
                    const PlanResult plan = planner.plan(planning, cells[from], cells.back(),
                                                         connectivity);
                    ReplanEvent event;
                    event.time = now;
                    event.from = cells[from];
                    event.status = plan.status;
                    event.cost = plan.cost();
                    event.expanded_nodes = plan.expanded_nodes;
                    event.planning_time_ms = plan.planning_time_ms;
                    event.blocked_cells = view->blockedCount();
                    if (plan.success()) {
                        event.path = plan.path;
                        cells.resize(from + 1);
                        waypoints.resize(from + 1);
                        reverse_target = kNoTarget;
                        if (retreat) {
                            // Back to the previous waypoint first (the next target).
                            cells.push_back(cells[from]);
                            waypoints.push_back(waypoints[from]);
                            reverse_target = target;
                        }
                        for (std::size_t i = 1; i < plan.path.size(); ++i) {
                            cells.push_back(plan.path[i]);
                            waypoints.push_back(map.gridToWorld(plan.path[i]));
                        }
                        blocked_since = -1.0;
                    } else if (retreat) {
                        // No way around yet, but the waypoint ahead is blocked: back off
                        // to the previous one and keep the old path for when it clears.
                        cells.insert(cells.begin() + static_cast<long>(target), cells[target - 1]);
                        waypoints.insert(waypoints.begin() + static_cast<long>(target),
                                         waypoints[target - 1]);
                        reverse_target = target;
                    } else {
                        // No way around yet: keep following the old path while its next
                        // waypoint is free (the safety stop still guards every step), and
                        // wait on a waypoint whose successor is blocked.
                        wait = at_cell && target_blocked;
                    }
                    result.replans.push_back(std::move(event));
                }
            }
        }
        if (wait) {
            if (hits_obstacle(position, position, now, next)) {
                result.collision_count = 1;
                result.status = NavigationStatus::Collision;
                break;
            }
            turning = false;
            ++result.wait_steps;
            ++result.steps;
            record(next, 0.0, 0.0);
            continue;
        }
        if (diff_drive) {
            // Turn-then-drive controller. While the heading is off the direction to
            // the next waypoint, rotate in place (v = 0) under the P heading
            // controller. Once aligned, drive straight to the waypoint (omega = 0),
            // so the robot's center follows the planned, collision-checked segments.
            // A retreat (dynamic obstacles) is driven backwards, without turning.
            const WorldPoint& waypoint = waypoints[target];
            const double ahead = std::atan2(waypoint.y - position.y, waypoint.x - position.x);
            const bool reversing = target == reverse_target ||
                                   (dynamic && std::fabs(wrapAngle(ahead - theta)) > kPi / 2 + 1e-9);
            if (reversing) {
                reverse_target = target;
            }
            const double desired =
                wrapAngle(std::atan2(waypoint.y - position.y, waypoint.x - position.x) +
                          (reversing ? kPi : 0.0));
            const double error = wrapAngle(desired - theta);
            if (std::fabs(error) > kDriveAlignment && dynamic) {
                // Evasive step: turning in place is standing still. If that would
                // end in contact with a seen obstacle, step straight forward or
                // backward instead, whichever keeps the larger predicted gap.
                const WorldPoint evade = evasiveStep(*view, map, position, theta, step_limit,
                                                     dynamic->replanning, step_is_free);
                if (evade.x != position.x || evade.y != position.y) {
                    if (hits_obstacle(position, evade, now, next)) {
                        result.collision_count = 1;
                        result.status = NavigationStatus::Collision;
                        break;
                    }
                    const double direction =
                        (evade.x - position.x) * std::cos(theta) + (evade.y - position.y) * std::sin(theta) < 0.0 ? -1.0 : 1.0;
                    position = evade;
                    result.distance_traveled += step_limit;
                    ++result.steps;
                    turning = false;
                    record(next, direction * step_limit / config.dt, 0.0);
                    continue;
                }
            }
            if (std::fabs(error) > kDriveAlignment) {
                const double omega = headingControl(error, config.heading_gain,
                                                    config.max_angular_speed,
                                                    config.heading_tolerance, config.dt);
                if (dynamic && hits_obstacle(position, position, now, next)) {
                    result.collision_count = 1;
                    result.status = NavigationStatus::Collision;
                    break;
                }
                theta = integrateUnicycle({position.x, position.y, theta}, 0.0, omega,
                                          config.dt).theta;
                // A disc rotating in place sweeps no new area: no collision check.
                result.turns += turning ? 0 : 1;
                turning = true;
                result.turn_time += config.dt;
                result.total_rotation += std::fabs(omega) * config.dt;
                ++result.steps;
                record(next, 0.0, omega);
                continue;
            }
            turning = false;
            theta = desired;  // residual error <= kDriveAlignment is absorbed
        }
        const StepResult step = stepToward(position, waypoints[target], step_limit);
        if (dynamic && !moveIsSafe(*view, waypoints, target, position, step_limit,
                                   dynamic->replanning)) {
            // Look-ahead gate: driving on would meet a seen obstacle within the
            // prediction horizon, and standing still is safer. Let it pass.
            if (hits_obstacle(position, position, now, next)) {
                result.collision_count = 1;
                result.status = NavigationStatus::Collision;
                break;
            }
            ++result.wait_steps;
            ++result.steps;
            record(next, 0.0, 0.0);
            continue;
        }
        if (!step_is_free(position, step.position) ||
            (dynamic && hits_obstacle(position, step.position, now, next))) {
            // The unsafe step is not executed; the robot stops at its last safe pose.
            result.collision_count = 1;
            result.status = NavigationStatus::Collision;
            break;
        }
        const double direction = target == reverse_target ? -1.0 : 1.0;  // backwards?
        position = step.position;
        result.distance_traveled += step.distance;
        ++result.steps;
        if (step.reached) {
            if (target == reverse_target) {
                reverse_target = kNoTarget;  // the retreat is complete
            }
            target = skipReachedWaypoints(waypoints, target + 1, position);
        }
        // Time is computed from the step count (not accumulated) to avoid drift.
        record(next, diff_drive ? direction * step.distance / config.dt : 0.0, 0.0);
    }

    result.final_goal_error = distanceBetween(position, goal);
    if (result.status == NavigationStatus::NotRun) {
        result.status = result.final_goal_error <= config.goal_tolerance
                            ? NavigationStatus::GoalReached
                            : NavigationStatus::Timeout;
    }
    if (dynamic && !std::isfinite(result.min_dynamic_clearance)) {
        result.min_dynamic_clearance = 0.0;  // no obstacles
    }
    return result;
}

}  // namespace amr
