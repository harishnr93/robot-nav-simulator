#include "simulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <queue>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "footprint.hpp"
#include "geometry2d.hpp"
#include "lidar.hpp"
#include "obstacle_tracking.hpp"
#include "spacetime_planner.hpp"

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
          working_(*scenario.planning_map), last_seen_(scenario.obstacles.size()),
          lidar_(lidarFor(scenario)), errors_(lidar_), tracker_(scenario.perception.tracker) {}

    /// Look around from `robot` (heading `theta`) at time t: find the obstacles,
    /// predict their motion and block the planning cells near their predicted sweeps
    /// (the previous step's blocks are lifted first). Called once per step.
    void observe(const GridMap& map, WorldPoint robot, double theta, double t,
                 const std::vector<MovingDisc>& others = {}) {
        for (const GridCoord& cell : blocked_) {
            working_.setOccupied(cell, false);
        }
        blocked_.clear();
        seen_.clear();

        if (scenario_.perception.mode == ObstaclePerception::Lidar) {
            observeWithLidar(map, robot, theta, t, others);
            return;
        }
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
            seen_.push_back({center, velocity, obstacle.radius, {}, {}});
            const WorldPoint ahead{center.x + velocity.x * horizon,
                                   center.y + velocity.y * horizon};
            block(center, ahead, obstacle.radius + robot_radius_ + scenario_.replanning.margin);
        }
        oracle_obstacles_seen_ = seen_.size();
        // Other robots, seen the same way (velocity from two sightings).
        std::map<int, WorldPoint> seen_robots;
        for (const MovingDisc& other : others) {
            if (!isObstacleVisible(map, robot, other.center, other.radius, scenario_.replanning.sensing_range)) {
                continue;
            }
            WorldPoint velocity{0.0, 0.0};
            const auto last = last_robots_.find(other.id);
            if (last != last_robots_.end()) {
                velocity = {(other.center.x - last->second.x) / dt_, (other.center.y - last->second.y) / dt_};
            }
            seen_robots[other.id] = other.center;
            seen_.push_back({other.center, velocity, other.radius, {}, {}});
            const WorldPoint ahead{other.center.x + velocity.x * horizon, other.center.y + velocity.y * horizon};
            block(other.center, ahead, other.radius + robot_radius_ + scenario_.replanning.margin);
        }
        last_robots_ = std::move(seen_robots);
    }

    /// Smallest predicted gap between the robot and the seen obstacles' edges when
    /// the robot is at robot[j] at time j dt from now (robot[0] = now) and every
    /// obstacle keeps its estimated velocity. Each step is checked exactly.
    double predictedGap(const std::vector<WorldPoint>& robot) const {
        return predict(robot).gap;
    }

    /// Occlusion awareness: true if, standing at `p`, the robot could be reached
    /// within the reaction time by an obstacle emerging from the free space hidden
    /// (by walls) from `p`.
    bool exposedToPhantoms(const GridMap& map, WorldPoint p) const {
        const PerceptionConfig& c = scenario_.perception;
        const double reach = c.phantom_speed * c.phantom_reaction + c.phantom_radius + robot_radius_;
        return hiddenDistance(map, p, reach) < reach;
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
                WorldPoint q0{s.center.x + s.velocity.x * t0, s.center.y + s.velocity.y * t0};
                WorldPoint q1{s.center.x + s.velocity.x * t1, s.center.y + s.velocity.y * t1};
                double extra = 0.0;
                if (!s.path.empty()) {  // tracked: the predicted path, inflated
                    const std::size_t last = s.path.size() - 1;
                    q0 = s.path[std::min(j - 1, last)];
                    q1 = s.path[std::min(j, last)];
                    extra = s.inflation[std::min(j, last)];
                }
                const double gap = closestApproach(robot[j - 1], robot[j], q0, q1) - s.radius -
                                   extra - robot_radius_;
                result.gap = std::min(result.gap, gap);
                if (gap < 0.0) {
                    result.steps_to_contact = std::min(result.steps_to_contact, j - 1);
                }
            }
        }
        return result;
    }

    /// Predicted motion of every perceived obstacle (and other robot) for `steps`
    /// steps from time t, for space-time planning. With perfect prediction: the
    /// scripted obstacles' true future motion instead.
    std::vector<ObstaclePrediction> longPredictions(double t, std::size_t steps) const {
        std::vector<ObstaclePrediction> out;
        if (scenario_.replanning.perfect_prediction) {
            for (const DynamicObstacle& obstacle : scenario_.obstacles) {
                ObstaclePrediction p;
                p.radius = obstacle.radius;
                for (std::size_t k = 0; k <= steps; ++k) {
                    p.centers.push_back(obstaclePosition(obstacle, t + static_cast<double>(k) * dt_));
                }
                out.push_back(std::move(p));
            }
        }
        if (scenario_.perception.mode == ObstaclePerception::Lidar) {
            for (const Track& track : tracker_.tracks()) {
                if (scenario_.replanning.perfect_prediction && isScriptedObstacle(track.position(), t)) {
                    continue;  // its true future is above
                }
                for (const Track& hypothesis : turningHypotheses(track)) {
                    ObstaclePrediction p;
                    p.radius = track.radius;
                    for (const PredictedPosition& q : tracker_.predictPath(hypothesis, dt_, steps)) {
                        p.centers.push_back(q.center);
                        p.inflation.push_back(std::min(2.0 * q.sigma, scenario_.perception.max_inflation));
                    }
                    out.push_back(std::move(p));
                }
            }
            // Other robots are tracked like any obstacle; their tracks are above.
            return out;
        }
        // Oracle sightings: straight on at the estimated velocity (other robots too;
        // with perfect prediction the scripted obstacles were added above).
        const std::size_t first = scenario_.replanning.perfect_prediction ? oracle_obstacles_seen_ : 0;
        for (std::size_t i = first; i < seen_.size(); ++i) {
            const Seen& s = seen_[i];
            ObstaclePrediction p;
            p.radius = s.radius;
            for (std::size_t k = 0; k <= steps; ++k) {
                const double tau = static_cast<double>(k) * dt_;
                p.centers.push_back({s.center.x + s.velocity.x * tau, s.center.y + s.velocity.y * tau});
            }
            out.push_back(std::move(p));
        }
        return out;
    }

    /// The smallest gap between a robot motion (one entry per step, entry 0 = now)
    /// and long predictions made now, as predictedGap() does for the seen obstacles.
    double gapAgainst(const std::vector<ObstaclePrediction>& predictions, const std::vector<WorldPoint>& robot) const {
        double gap = std::numeric_limits<double>::infinity();
        for (const ObstaclePrediction& p : predictions) {
            const std::size_t last = p.centers.size() - 1;
            for (std::size_t j = 1; j < robot.size() && !p.centers.empty(); ++j) {
                const double extra = p.inflation.empty() ? 0.0 : p.inflation[std::min(j, last)];
                gap = std::min(gap, closestApproach(robot[j - 1], robot[j], p.centers[std::min(j - 1, last)],
                                                    p.centers[std::min(j, last)]) -
                                        p.radius - extra - robot_radius_);
            }
        }
        return gap;
    }

    /// Long predictions of a lane track whose turning point ahead is not known yet:
    /// constant velocity would carry it on through walls, and a plan that follows it
    /// into its lane meets it head-on when it turns. It may turn anywhere before the
    /// wall ahead; planned against are the two extremes, turning at the wall (it
    /// cannot pass it) and turning now, each ping-ponging back to the end behind it
    /// (if unknown too, the wall behind). Any other track: itself.
    std::vector<Track> turningHypotheses(const Track& track) const {
        const TrackerConfig& c = tracker_.config();
        if (!track.confirmed || !track.lane || c.prediction != MotionPrediction::LearnedReversals) {
            return {track};
        }
        const WorldPoint a = track.lane_axis;
        const WorldPoint p = track.position();
        const double s = (p.x - track.lane_center.x) * a.x + (p.y - track.lane_center.y) * a.y;
        const double v = track.x[2] * a.x + track.x[3] * a.y;
        const bool ahead_known = v > 0.0 ? track.has_high_end : track.has_low_end;
        if (std::fabs(v) < c.reversal_speed || ahead_known) {
            return {track};
        }
        // Free run from p along +-axis before the disc would touch a wall.
        const auto free_run = [&](double direction) {
            constexpr double kStep = 0.05;
            double run = 0.0;
            while (run < 50.0) {
                const WorldPoint q{p.x + direction * (run + kStep) * a.x, p.y + direction * (run + kStep) * a.y};
                if (!working_.inBounds(working_.worldToGrid(q)) || !scenario_.planning_map->isFree(working_.worldToGrid(q))) {
                    break;
                }
                run += kStep;
            }
            return std::max(0.0, run + 0.5 * working_.resolution() - track.radius);
        };
        Track wall = track, now = track;
        const double ahead = v > 0.0 ? s + free_run(1.0) : s - free_run(-1.0);
        const bool behind_known = v > 0.0 ? track.has_low_end : track.has_high_end;
        const double behind = behind_known ? (v > 0.0 ? track.low_end : track.high_end)
                                           : (v > 0.0 ? s - free_run(-1.0) : s + free_run(1.0));
        for (Track* h : {&wall, &now}) {
            const double turn = h == &wall ? ahead : s;
            h->has_low_end = h->has_high_end = true;
            h->low_end = v > 0.0 ? std::min(behind, turn) : turn;
            h->high_end = v > 0.0 ? turn : std::max(behind, turn);
        }
        return {wall, now};
    }

    /// True if a track at `p` is one of the scripted obstacles at time t (perfect
    /// prediction: their futures are known; what else is tracked, other robots, is not).
    bool isScriptedObstacle(WorldPoint p, double t) const {
        return std::any_of(scenario_.obstacles.begin(), scenario_.obstacles.end(), [&](const DynamicObstacle& o) {
            return distanceBetween(p, obstaclePosition(o, t)) < o.radius + 0.25;
        });
    }

    GridMap& planningMap() { return working_; }
    double dt() const { return dt_; }
    std::size_t blockedCount() const { return blocked_.size(); }

    std::size_t tracksCreated() const { return created_; }

private:
    struct Seen {
        WorldPoint center;
        WorldPoint velocity;  ///< estimated, m/s (oracle)
        double radius;
        std::vector<WorldPoint> path;   ///< tracked: predicted positions every dt (entry 0 = now)
        std::vector<double> inflation;  ///< tracked: radius added for uncertainty, per entry
    };

    /// Distance through free cells from `p` to the nearest free cell within sensing
    /// range that is hidden from `p` by walls, searched up to `limit` (infinity beyond).
    double hiddenDistance(const GridMap& map, WorldPoint p, double limit) const {
        const double sensing = scenario_.replanning.sensing_range;
        const double res = map.resolution();
        const GridCoord start = map.worldToGrid(p);
        if (!map.isFree(start)) {
            return std::numeric_limits<double>::infinity();
        }
        const int w = map.width();
        std::vector<std::pair<double, GridCoord>> heap{{0.0, start}};
        std::vector<double> best;
        std::vector<int> touched;
        const auto slot = [&](GridCoord c) -> double& {
            const int key = c.y * w + c.x;
            if (best.empty()) {
                best.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(map.height()),
                           std::numeric_limits<double>::infinity());
            }
            return best[static_cast<std::size_t>(key)];
        };
        slot(start) = 0.0;
        const auto greater = [](const auto& a, const auto& b) { return a.first > b.first; };
        while (!heap.empty()) {
            std::pop_heap(heap.begin(), heap.end(), greater);
            const auto [d, cell] = heap.back();
            heap.pop_back();
            if (d > slot(cell) || d > limit) {
                continue;
            }
            const WorldPoint center = map.gridToWorld(cell);
            const double straight = std::hypot(center.x - p.x, center.y - p.y);
            if (straight > 0.0 && straight <= sensing &&
                castRay(map, p, std::atan2(center.y - p.y, center.x - p.x), straight).hit) {
                return d;  // hidden behind a wall
            }
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const GridCoord next{cell.x + dx, cell.y + dy};
                    if ((dx == 0 && dy == 0) || !map.isFree(next)) {
                        continue;
                    }
                    const double nd = d + res * (dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0);
                    if (nd < slot(next)) {
                        slot(next) = nd;
                        heap.push_back({nd, next});
                        std::push_heap(heap.begin(), heap.end(), greater);
                    }
                }
            }
        }
        return std::numeric_limits<double>::infinity();
    }

    static LidarConfig lidarFor(const DynamicScenario& scenario) {
        LidarConfig lidar = scenario.perception.lidar;
        if (lidar.beams == 0) {
            lidar.beams = 360;
            lidar.max_range = scenario.replanning.sensing_range;
        }
        return lidar;
    }

    /// Lidar perception: scan the map and the obstacles, subtract the map, track,
    /// and block along every track's predicted path.
    void observeWithLidar(const GridMap& map, WorldPoint robot, double theta, double t,
                          const std::vector<MovingDisc>& others) {
        LidarScan scan;
        scan.time = t;
        scan.origin = robot;
        scan.heading = theta;
        errors_.nextScan(t);
        for (const double offset : beamOffsets(lidar_)) {
            const double angle = theta + offset;
            const RayHit wall = castRay(map, robot, angle, lidar_.max_range);
            double range = wall.hit ? wall.range : std::numeric_limits<double>::infinity();
            for (const DynamicObstacle& obstacle : scenario_.obstacles) {
                range = std::min(range, rayToCircle(obstaclePosition(obstacle, t), obstacle.radius, robot, angle));
            }
            for (const MovingDisc& other : others) {
                range = std::min(range, rayToCircle(other.center, other.radius, robot, angle));
            }
            const bool hit = range <= lidar_.max_range;
            scan.angles.push_back(angle);
            scan.ranges.push_back(hit ? errors_.measure(range) : lidar_.max_range);
            scan.hits.push_back(hit);
        }
        const bool full_circle = lidar_.fov >= 2.0 * kPi - 1e-12;
        const auto detections =
            detectMovingObstacles(map, scan, lidar_.max_range, full_circle, scenario_.perception.detection);
        tracker_.update(t, detections, map, scan, lidar_.max_range);
        for (const Track& track : tracker_.tracks()) {
            created_ = std::max(created_, static_cast<std::size_t>(track.id + 1));
        }
        const double horizon = scenario_.replanning.prediction_horizon;
        const auto steps = static_cast<std::size_t>(std::max(1.0, std::ceil(horizon / dt_ - 1e-9)));
        for (const Track& track : tracker_.tracks()) {
            const auto predicted = tracker_.predictPath(track, dt_, steps);
            Seen seen{track.position(), track.velocity(), track.radius, {}, {}};
            for (const PredictedPosition& p : predicted) {
                seen.path.push_back(p.center);
                seen.inflation.push_back(std::min(2.0 * p.sigma, scenario_.perception.max_inflation));
            }
            const double base = track.radius + robot_radius_ + scenario_.replanning.margin;
            // The sweep over the horizon (the whole predicted path when the horizon is
            // positive, the current position otherwise).
            const std::size_t last = horizon > 0.0 ? seen.path.size() - 1 : 0;
            const bool inflate = scenario_.perception.inflate_planning;
            if (last == 0) {
                block(seen.path[0], seen.path[0], base + (inflate ? seen.inflation[0] : 0.0));
            }
            for (std::size_t j = 0; j < last; ++j) {
                block(seen.path[j], seen.path[j + 1], base + (inflate ? seen.inflation[j + 1] : 0.0));
            }
            seen_.push_back(std::move(seen));
        }
    }

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
    std::vector<std::optional<WorldPoint>> last_seen_;  // per obstacle, previous step (oracle)
    std::map<int, WorldPoint> last_robots_;             // other robots seen last step (oracle)
    std::size_t oracle_obstacles_seen_ = 0;             // oracle: seen_ entries that are scripted obstacles
    LidarConfig lidar_;                                 // Lidar perception
    RangeErrorModel errors_;
    ObstacleTracker tracker_;
    std::size_t created_ = 0;
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

/// Like predictMotion, but holding at each waypoint until its scheduled departure
/// (absolute times `departs`, parallel to `waypoints`; the clock starts at `now`).
std::vector<WorldPoint> predictScheduledMotion(const std::vector<WorldPoint>& waypoints,
                                               const std::vector<double>& departs, std::size_t target,
                                               WorldPoint position, double step_limit, std::size_t steps,
                                               double now, double dt) {
    std::vector<WorldPoint> path{position};
    for (std::size_t j = 0; j < steps; ++j) {
        const double t = now + static_cast<double>(j) * dt;
        const bool holding = distanceBetween(position, waypoints[target - 1]) <= kArrivalEpsilon &&
                             t + 1e-9 < departs[target - 1];
        if (target < waypoints.size() && !holding) {
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

/// Row-major index of an in-bounds cell.
std::size_t cellIndex(const GridMap& map, GridCoord c) {
    return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(map.width()) + static_cast<std::size_t>(c.x);
}

/// Shortest distance from every cell to `goal` on the static map (m; infinity if
/// unreachable), by Dijkstra over the allowed moves.
std::vector<double> costToGo(const GridMap& map, GridCoord goal, Connectivity connectivity) {
    const int width = map.width();
    std::vector<double> cost(static_cast<std::size_t>(width) * static_cast<std::size_t>(map.height()),
                             std::numeric_limits<double>::infinity());
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    cost[cellIndex(map, goal)] = 0.0;
    open.push({0.0, goal.y * width + goal.x});
    while (!open.empty()) {
        const auto [d, index] = open.top();
        open.pop();
        if (d > cost[static_cast<std::size_t>(index)]) {
            continue;
        }
        const GridCoord c{index % width, index / width};
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const GridCoord n{c.x + dx, c.y + dy};
                if ((dx == 0 && dy == 0) || !map.isFree(n) || !isAllowedMove(map, n, c, connectivity)) {
                    continue;
                }
                const double next = d + (dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0) * map.resolution();
                const std::size_t i = cellIndex(map, n);
                if (next < cost[i]) {
                    cost[i] = next;
                    open.push({next, n.y * width + n.x});
                }
            }
        }
    }
    return cost;
}

/// Smallest predicted gap accepted once the gate has waited out the patience, m.
constexpr double kWaivedMargin = 0.02;

/// Look-ahead gate: true unless following the path for the prediction horizon
/// would come within `margin` of a seen obstacle (moving at its estimated
/// velocity) while standing still would keep a larger gap.
bool moveIsSafe(const DynamicView& view, const std::vector<WorldPoint>& waypoints,
                std::size_t target, WorldPoint position, double step_limit,
                const ReplanningConfig& config, bool margin_waived) {
    const auto steps = static_cast<std::size_t>(
        std::max(1.0, std::ceil(config.prediction_horizon / view.dt() - 1e-9)));
    const double moving = view.predictedGap(predictMotion(waypoints, target, position,
                                                          step_limit, steps));
    // After waiting out the patience, the margin is a wish, not a rule: an obstacle
    // that patrols right next to the path never leaves it, and contact-free is enough.
    if (moving >= (margin_waived ? kWaivedMargin : config.margin)) {
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
    if (!std::isfinite(config.spacetime_horizon) || config.spacetime_horizon <= 0.0 ||
        config.spacetime_horizon > kMaxPredictionHorizon) {
        return "space-time horizon must be in (0, 100] s";
    }
    if (!std::isfinite(config.livelock_window) || config.livelock_window < 0.0 ||
        config.livelock_window > kMaxLivelockWindow) {
        return "livelock window must be in [0, 3600] s";
    }
    if (!std::isfinite(config.spacetime_period) || config.spacetime_period < 0.0 ||
        config.spacetime_period > kMaxPredictionHorizon) {
        return "space-time replanning period must be in [0, 100] s";
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

/// Everything a Navigator carries from one step to the next (the loop variables of
/// the original single-robot simulation).
class NavigatorState {
public:
    NavigatorState(const GridMap& map, const std::vector<GridCoord>& path, const MotionConfig& config,
                   Connectivity connectivity, const DynamicScenario* dynamic)
        : map_(map), config_(config), connectivity_(connectivity), dynamic_(dynamic) {
        if (!validateMotionConfig(config).empty() ||
            (dynamic && (dynamic->planning_map == nullptr ||
                         !validateReplanningConfig(dynamic->replanning).empty()))) {
            result_.status = NavigationStatus::InvalidConfig;
            finished_ = true;
            return;
        }
        if (!isStructurallyValidPath(map, path, connectivity)) {
            result_.status = NavigationStatus::InvalidPath;
            finished_ = true;
            return;
        }
        // With replanning the remaining waypoints change; `cells_` stays parallel to them.
        cells_ = path;
        waypoints_.reserve(path.size());
        for (const GridCoord& cell : path) {
            waypoints_.push_back(map.gridToWorld(cell));
        }
        goal_ = waypoints_.back();
        step_limit_ = config.speed * config.dt;
        // Floor with a small epsilon so e.g. 600 / 0.1 gives 6000 steps, not 5999.
        max_steps_ = static_cast<std::size_t>(std::floor(config.max_time / config.dt + 1e-9));
        position_ = waypoints_.front();
        previous_ = position_;
        diff_drive_ = config.model == RobotModel::DiffDrive;
        theta_ = diff_drive_ ? wrapAngle(config.initial_heading) : 0.0;
        target_ = skipReachedWaypoints(waypoints_, 1, position_);
        // Exact number of samples for an uninterrupted run: each leg takes
        // ceil(leg length / step) steps (the last one clamped), plus the t = 0 sample.
        std::size_t expected_steps = 0;
        for (std::size_t i = 1; i < waypoints_.size(); ++i) {
            const double leg = distanceBetween(waypoints_[i - 1], waypoints_[i]);
            expected_steps += static_cast<std::size_t>(std::ceil(leg / step_limit_ - kArrivalEpsilon));
        }
        result_.trajectory.reserve(std::min(expected_steps, max_steps_) + 1);
        record(0.0);
        if (dynamic) {
            view_.emplace(*dynamic, config.robot_radius, config.dt);
            result_.min_dynamic_clearance = std::numeric_limits<double>::infinity();
            dynamic_patience_ = dynamic->replanning.patience;
            planner_mode_ = dynamic->replanning.planner;
            if (dynamic->replanning.livelock_window > 0.0) {
                cost_to_go_ = costToGo(*dynamic->planning_map, cells_.back(), connectivity);
            }
        }
        if (!stepIsFree(position_, position_) || (dynamic && hitsObstacle(position_, position_, 0.0, 0.0))) {
            result_.collision_count = 1;
            result_.status = NavigationStatus::Collision;
        }
        finishIfDone();
    }

    bool finished() const { return finished_; }
    const SimulationResult& result() const { return result_; }
    WorldPoint position() const { return position_; }
    WorldPoint previous() const { return previous_; }

    void collideWithRobot(int other) {
        if (finished_) {
            return;
        }
        result_.collision_count = 1;
        result_.collided_robot = other;
        result_.status = NavigationStatus::Collision;
        finishIfDone();
    }

    void step(const std::vector<MovingDisc>& others) {
        if (finished_) {
            return;
        }
        previous_ = position_;
        stepOnce(others);
        finishIfDone();
    }

private:
    void record(double time, double v = 0.0, double omega = 0.0) {
        const std::size_t last = waypoints_.size() - 1;
        const std::size_t recorded_target = target_ < waypoints_.size() ? target_ : last;
        result_.trajectory.push_back({RobotState{position_.x, position_.y, time, theta_}, recorded_target, v, omega});
    }

    /// Point robot: the Phase 3 point check; with 8-connectivity the step must also
    /// not touch an occupied cell (a diagonal move through the shared corner of an
    /// obstacle touches it). Disc robot: exact swept-disc check.
    bool stepIsFree(WorldPoint from, WorldPoint to) const {
        if (config_.robot_radius > 0.0) {
            return !sweptDiscOverlapsObstacle(map_, from, to, config_.robot_radius);
        }
        return isSegmentCollisionFree(map_, from, to) &&
               (connectivity_ == Connectivity::Four || !segmentTouchesObstacle(map_, from, to));
    }

    /// Dynamic obstacles: exact contact test over one step (robot linear a -> b).
    bool hitsObstacle(WorldPoint a, WorldPoint b, double t0, double t1) {
        for (std::size_t i = 0; i < dynamic_->obstacles.size(); ++i) {
            const DynamicObstacle& obstacle = dynamic_->obstacles[i];
            const double gap = minDistanceDuring(obstacle, a, b, t0, t1) - obstacle.radius - config_.robot_radius;
            result_.min_dynamic_clearance = std::min(result_.min_dynamic_clearance, gap);
            if (gap < -kFootprintTolerance) {
                result_.collided_obstacle = static_cast<int>(i);
                return true;
            }
        }
        return false;
    }

    void collide() {
        result_.collision_count = 1;
        result_.status = NavigationStatus::Collision;
    }

    /// A step spent standing (waiting, held by a gate).
    void standStill(double now, double next) {
        if (hitsObstacle(position_, position_, now, next)) {
            collide();
            return;
        }
        ++result_.wait_steps;
        ++result_.steps;
        record(next, 0.0, 0.0);
    }

    void finishIfDone() {
        if (finished_) {
            return;
        }
        if (result_.status == NavigationStatus::InvalidConfig || result_.status == NavigationStatus::InvalidPath) {
            finished_ = true;
            return;
        }
        if (result_.status == NavigationStatus::NotRun && target_ < waypoints_.size()) {
            return;  // still on the way
        }
        result_.final_goal_error = distanceBetween(position_, goal_);
        if (result_.status == NavigationStatus::NotRun) {
            result_.status = result_.final_goal_error <= config_.goal_tolerance ? NavigationStatus::GoalReached
                                                                               : NavigationStatus::Timeout;
        }
        if (dynamic_) {
            result_.tracks_created = view_->tracksCreated();
        }
        if (dynamic_ && !std::isfinite(result_.min_dynamic_clearance)) {
            result_.min_dynamic_clearance = 0.0;  // no obstacles
        }
        finished_ = true;
    }

    /// Space-time planning. On a cell: replan when there is no valid schedule, when it
    /// is older than the period, or when the robot fell behind it. Then (anywhere) the
    /// schedule is checked against the current predictions for contact over the
    /// prediction horizon, waits included; a schedule that fails is replanned (between
    /// cells, from either end of the segment). On a cell before the planned departure
    /// the robot holds, turning towards the next cell meanwhile. True if the step was
    /// spent holding; false leaves the step to the controller (and, without a valid
    /// schedule, to the reactive rules).
    bool spaceTimeStep(double now, double next) {
        const bool at_cell = distanceBetween(position_, waypoints_[target_ - 1]) <= kArrivalEpsilon;
        constexpr double kSlip = 0.5;  // s behind schedule before replanning
        if (at_cell && (!schedule_valid_ || now > departs_[target_ - 1] + kSlip)) {
            replanSpaceTime(now, false);
        } else if (at_cell && now >= last_plan_ + dynamic_->replanning.spacetime_period - 1e-9) {
            replanSpaceTime(now, true);
        }
        if (schedule_valid_) {
            const auto steps = static_cast<std::size_t>(
                std::max(1.0, std::ceil(dynamic_->replanning.prediction_horizon / config_.dt - 1e-9)));
            schedule_valid_ = view_->gapAgainst(view_->longPredictions(now, steps),
                                                predictScheduledMotion(waypoints_, departs_, target_, position_,
                                                                       step_limit_, steps, now, config_.dt)) >=
                              kWaivedMargin;
            if (!schedule_valid_ && !at_cell) {
                replanSpaceTime(now, false);
            }
        } else if (!at_cell) {
            replanSpaceTime(now, false);
        }
        if (!schedule_valid_ || !at_cell || now + 1e-9 >= departs_[target_ - 1]) {
            return false;
        }
        if (diff_drive_) {
            // Turn towards the next cell while waiting (turning in place is safe in the cell).
            const WorldPoint& waypoint = waypoints_[target_];
            const double error = wrapAngle(std::atan2(waypoint.y - position_.y, waypoint.x - position_.x) - theta_);
            if (std::fabs(error) > kDriveAlignment) {
                const double omega = headingControl(error, config_.heading_gain, config_.max_angular_speed,
                                                    config_.heading_tolerance, config_.dt);
                if (hitsObstacle(position_, position_, now, next)) {
                    collide();
                    return true;
                }
                theta_ = integrateUnicycle({position_.x, position_.y, theta_}, 0.0, omega, config_.dt).theta;
                result_.turns += turning_ ? 0 : 1;
                turning_ = true;
                result_.turn_time += config_.dt;
                result_.total_rotation += std::fabs(omega) * config_.dt;
                ++result_.steps;
                record(next, 0.0, omega);
                return true;
            }
        }
        turning_ = false;
        standStill(now, next);
        return true;
    }

    /// Space-time plan to the goal from the robot's cell or, between cells, from the
    /// end of its segment (ahead, or behind: driven back to) whose plan arrives first,
    /// provided the drive there is contact-free under the predictions. Replaces the
    /// schedule on success; invalidates it otherwise. A routine replan (`routine`:
    /// the schedule is valid, only due) keeps the schedule while it stays
    /// contact-free over the whole horizon, unless the new plan arrives clearly
    /// earlier: replanning every step otherwise flips between routes of about equal
    /// arrival as the predictions' phase shifts, and the robot shuttles between them.
    void replanSpaceTime(double now, bool routine) {
        const auto steps =
            static_cast<std::size_t>(std::ceil(dynamic_->replanning.spacetime_horizon / config_.dt - 1e-9));
        SpaceTimeConfig sc;
        sc.dt = config_.dt;
        sc.horizon = dynamic_->replanning.spacetime_horizon;
        sc.clearance = config_.robot_radius + dynamic_->replanning.margin;
        sc.speed = config_.speed;
        sc.connectivity = connectivity_;
        sc.turn_aware = diff_drive_;
        sc.max_angular_speed = config_.max_angular_speed;
        const std::vector<ObstaclePrediction> predictions = view_->longPredictions(now, steps);
        const bool at_cell = distanceBetween(position_, waypoints_[target_ - 1]) <= kArrivalEpsilon;
        std::vector<std::size_t> ends;
        if (at_cell) {
            ends.push_back(target_ - 1);
        } else {
            for (const std::size_t end : {target_, target_ - 1}) {
                // The straight drive there must clear the walls (the robot may be off
                // the segment after evasive steps) and the predicted obstacles.
                const double distance = distanceBetween(position_, waypoints_[end]);
                const auto drive = static_cast<std::size_t>(std::ceil(distance / step_limit_ - 1e-9));
                if (stepIsFree(position_, waypoints_[end]) &&
                    view_->gapAgainst(predictions, predictMotion({waypoints_[end]}, 0, position_, step_limit_, drive)) >=
                        kWaivedMargin) {
                    ends.push_back(end);
                }
            }
        }
        SpaceTimePlan plan;
        std::size_t chosen = target_ - 1;
        double expanded = 0.0, planning_ms = 0.0;
        for (const std::size_t end : ends) {
            SpaceTimeConfig c = sc;
            c.start_time = at_cell ? 0.0 : distanceBetween(position_, waypoints_[end]) / config_.speed;
            SpaceTimePlan candidate =
                planSpaceTime(*dynamic_->planning_map, cells_[end], theta_, cells_.back(), predictions, c);
            expanded += static_cast<double>(candidate.expanded);
            planning_ms += candidate.planning_time_ms;
            if (!candidate.success || candidate.arrival > c.horizon) {
                // No way through with the margin inside the horizon: a plan that
                // crosses only once the predictions run out is postponed again at every
                // replan. Like the gate after the patience, settle for contact-free.
                SpaceTimeConfig waived = c;
                waived.clearance = config_.robot_radius + kWaivedMargin;
                SpaceTimePlan relaxed =
                    planSpaceTime(*dynamic_->planning_map, cells_[end], theta_, cells_.back(), predictions, waived);
                expanded += static_cast<double>(relaxed.expanded);
                planning_ms += relaxed.planning_time_ms;
                if (relaxed.success && (!candidate.success || relaxed.arrival < candidate.arrival)) {
                    candidate = std::move(relaxed);
                }
            }
            if (candidate.success && (!plan.success || candidate.arrival < plan.arrival)) {
                plan = std::move(candidate);
                chosen = end;
            }
        }
        if (routine && schedule_valid_) {
            constexpr double kSwitchGain = 1.0;  // s earlier arrival needed to switch plans
            const double current = departs_.back() - now;
            const bool better = plan.success && plan.arrival < current - kSwitchGain;
            if (!better && view_->gapAgainst(predictions, predictScheduledMotion(waypoints_, departs_, target_,
                                                                                 position_, step_limit_, steps, now,
                                                                                 config_.dt)) >= kWaivedMargin) {
                last_plan_ = now;
                return;
            }
        }
        ReplanEvent event;
        event.time = now;
        event.from = cells_[chosen];
        event.status = plan.success ? PlanStatus::Success : PlanStatus::NoPath;
        event.expanded_nodes = static_cast<std::size_t>(expanded);
        event.planning_time_ms = planning_ms;
        last_plan_ = now;
        if (plan.success) {
            event.path = planCells(plan);
            event.cost = gridPathCost(event.path);
            // Between cells, the segment's other end becomes the waypoint behind.
            std::vector<GridCoord> cells;
            if (!at_cell) {
                cells.push_back(cells_[chosen == target_ ? target_ - 1 : target_]);
            }
            cells.insert(cells.end(), event.path.begin(), event.path.end());
            const bool back = !at_cell && chosen == target_ - 1;
            cells_ = std::move(cells);
            waypoints_.clear();
            departs_.clear();
            if (!at_cell) {
                waypoints_.push_back(map_.gridToWorld(cells_.front()));
                departs_.push_back(now);
            }
            for (const TimedWaypoint& w : plan.waypoints) {
                waypoints_.push_back(map_.gridToWorld(w.cell));
                departs_.push_back(now + w.depart);
            }
            target_ = 1;
            reverse_target_ = back ? 1 : kNoTarget;  // drive back without turning
            blocked_since_ = -1.0;
            schedule_valid_ = true;
        } else {
            schedule_valid_ = false;
        }
        result_.replans.push_back(std::move(event));
    }

    /// Livelock detection (ReplanningConfig::livelock_window): no progress towards
    /// the goal for the window switches to the other strategy, for one window (an
    /// escape), then back. One escape per stuck place: the detector re-arms only once
    /// the robot is 1 m closer to the goal than where it got stuck (switching back
    /// and forth would make it shuttle between the two strategies' routes).
    void detectLivelock(double now) {
        const GridCoord cell = map_.worldToGrid(position_);
        if (!map_.inBounds(cell)) {
            return;
        }
        const double remaining = cost_to_go_[cellIndex(map_, cell)];
        const double window = dynamic_->replanning.livelock_window;
        constexpr double kRearmProgress = 1.0;  // m beyond the stuck place
        if (!switch_armed_ && remaining < stuck_remaining_ - kRearmProgress) {
            switch_armed_ = true;
        }
        if (planner_mode_ != dynamic_->replanning.planner && now - switched_at_ >= window - 1e-9) {
            setStrategy(dynamic_->replanning.planner);  // the escape is over
            best_remaining_ = remaining;
            progress_since_ = now;
            return;
        }
        if (remaining < best_remaining_ - 1e-9) {
            best_remaining_ = remaining;
            progress_since_ = now;
        } else if (now - progress_since_ >= window - 1e-9) {
            if (switch_armed_) {
                ++result_.strategy_switches;
                switched_at_ = now;
                switch_armed_ = false;
                stuck_remaining_ = best_remaining_;
                setStrategy(planner_mode_ == DynamicPlanner::Reactive ? DynamicPlanner::SpaceTime
                                                                       : DynamicPlanner::Reactive);
            }
            // Progress is now measured from here: a full window before the next check.
            best_remaining_ = remaining;
            progress_since_ = now;
        }
    }

    void setStrategy(DynamicPlanner planner) {
        planner_mode_ = planner;
        schedule_valid_ = false;
        blocked_since_ = -1.0;
        gate_since_ = -1.0;
    }

    /// One iteration of the navigation loop.
    void stepOnce(const std::vector<MovingDisc>& others) {
        if (result_.steps >= max_steps_) {
            result_.status = NavigationStatus::Timeout;
            return;
        }
        const double now = static_cast<double>(result_.steps) * config_.dt;
        const double next = static_cast<double>(result_.steps + 1) * config_.dt;
        bool wait = false;
        if (dynamic_) {
            view_->observe(map_, position_, theta_, now, others);
        }
        if (dynamic_ && !cost_to_go_.empty()) {
            detectLivelock(now);
        }
        if (dynamic_ && planner_mode_ == DynamicPlanner::SpaceTime && spaceTimeStep(now, next)) {
            return;  // held at a cell until the planned departure
        }
        if (dynamic_ && !schedule_valid_) {
            GridMap& planning = view_->planningMap();
            // The goal itself never counts as blocked (it cannot be avoided, only
            // timed): the look-ahead gate decides when to drive into it.
            planning.setOccupied(cells_.back(), false);
            const bool blocked = std::any_of(cells_.begin() + static_cast<long>(target_), cells_.end(),
                                             [&](GridCoord c) { return planning.isOccupied(c); });
            if (!blocked) {
                blocked_since_ = -1.0;
            } else {
                if (blocked_since_ < 0.0) {
                    blocked_since_ = now;
                }
                const bool at_cell = distanceBetween(position_, waypoints_[target_ - 1]) <= kArrivalEpsilon;
                const bool target_blocked = planning.isOccupied(cells_[target_]);
                // Act at once when the robot itself is threatened (its own cell, or
                // the cell it is entering, is blocked) or the blockage outlasted the
                // patience. Until then it follows the free part of the path and waits
                // at the last free waypoint: a moving obstacle usually clears the
                // way, while detouring at once makes the robot chase the crossing the
                // obstacle is about to block, back and forth beside it.
                const bool urgent = (at_cell && planning.isOccupied(cells_[target_ - 1])) ||
                                    (!at_cell && target_blocked) ||
                                    now - blocked_since_ >= dynamic_->replanning.patience - 1e-9;
                if (!urgent) {
                    wait = at_cell && target_blocked;
                } else {
                    // Replan from the current cell if the robot stands on a waypoint,
                    // otherwise from the waypoint it is driving to. If that waypoint
                    // is blocked, retreat: replan from the waypoint it came from
                    // (statically free, and just left), driving back to it first.
                    bool retreat = !at_cell && target_blocked;
                    if (retreat && dynamic_->replanning.choose_retreat && planning.isOccupied(cells_[target_ - 1])) {
                        // Both ends of the segment are blocked (the robot is inside a
                        // moving obstacle's predicted sweep): backing up is no safer by
                        // rule. Head for the end that keeps the larger predicted gap.
                        const auto steps = static_cast<std::size_t>(
                            std::max(1.0, std::ceil(dynamic_->replanning.prediction_horizon / config_.dt - 1e-9)));
                        const double ahead = view_->predictedGap(
                            predictMotion({waypoints_[target_]}, 0, position_, step_limit_, steps));
                        const double back = view_->predictedGap(
                            predictMotion({waypoints_[target_ - 1]}, 0, position_, step_limit_, steps));
                        retreat = back >= 0.0 || back > ahead;
                    }
                    const std::size_t from = at_cell || retreat ? target_ - 1 : target_;
                    planning.setOccupied(cells_[from], false);  // the robot's own cell
                    const PlanResult plan = planner_.plan(planning, cells_[from], cells_.back(), connectivity_);
                    ReplanEvent event;
                    event.time = now;
                    event.from = cells_[from];
                    event.status = plan.status;
                    event.cost = plan.cost();
                    event.expanded_nodes = plan.expanded_nodes;
                    event.planning_time_ms = plan.planning_time_ms;
                    event.blocked_cells = view_->blockedCount();
                    if (plan.success()) {
                        event.path = plan.path;
                        schedule_valid_ = false;
                        cells_.resize(from + 1);
                        waypoints_.resize(from + 1);
                        reverse_target_ = kNoTarget;
                        if (retreat) {
                            // Back to the previous waypoint first (the next target).
                            cells_.push_back(cells_[from]);
                            waypoints_.push_back(waypoints_[from]);
                            reverse_target_ = target_;
                        }
                        for (std::size_t i = 1; i < plan.path.size(); ++i) {
                            cells_.push_back(plan.path[i]);
                            waypoints_.push_back(map_.gridToWorld(plan.path[i]));
                        }
                        blocked_since_ = -1.0;
                    } else if (retreat) {
                        // No way around yet, but the waypoint ahead is blocked: back off
                        // to the previous one and keep the old path for when it clears.
                        cells_.insert(cells_.begin() + static_cast<long>(target_), cells_[target_ - 1]);
                        waypoints_.insert(waypoints_.begin() + static_cast<long>(target_), waypoints_[target_ - 1]);
                        reverse_target_ = target_;
                    } else {
                        // No way around yet: keep following the old path while its next
                        // waypoint is free (the safety stop still guards every step), and
                        // wait on a waypoint whose successor is blocked.
                        wait = at_cell && target_blocked;
                    }
                    result_.replans.push_back(std::move(event));
                }
            }
        }
        if (wait) {
            turning_ = false;
            standStill(now, next);
            return;
        }
        if (diff_drive_) {
            // Turn-then-drive controller. While the heading is off the direction to
            // the next waypoint, rotate in place (v = 0) under the P heading
            // controller. Once aligned, drive straight to the waypoint (omega = 0),
            // so the robot's center follows the planned, collision-checked segments.
            // A retreat (dynamic obstacles) is driven backwards, without turning.
            const WorldPoint& waypoint = waypoints_[target_];
            const double ahead = std::atan2(waypoint.y - position_.y, waypoint.x - position_.x);
            const bool reversing = target_ == reverse_target_ ||
                                   (dynamic_ && std::fabs(wrapAngle(ahead - theta_)) > kPi / 2 + 1e-9);
            if (reversing) {
                reverse_target_ = target_;
            }
            const double desired = wrapAngle(std::atan2(waypoint.y - position_.y, waypoint.x - position_.x) +
                                             (reversing ? kPi : 0.0));
            const double error = wrapAngle(desired - theta_);
            if (std::fabs(error) > kDriveAlignment && dynamic_) {
                // Evasive step: turning in place is standing still. If that would
                // end in contact with a seen obstacle, step straight forward or
                // backward instead, whichever keeps the larger predicted gap.
                const auto step_is_free = [this](WorldPoint from, WorldPoint to) { return stepIsFree(from, to); };
                const WorldPoint evade =
                    evasiveStep(*view_, map_, position_, theta_, step_limit_, dynamic_->replanning, step_is_free);
                if (evade.x != position_.x || evade.y != position_.y) {
                    if (hitsObstacle(position_, evade, now, next)) {
                        collide();
                        return;
                    }
                    const double direction = (evade.x - position_.x) * std::cos(theta_) +
                                                         (evade.y - position_.y) * std::sin(theta_) < 0.0
                                                 ? -1.0
                                                 : 1.0;
                    position_ = evade;
                    result_.distance_traveled += step_limit_;
                    ++result_.steps;
                    turning_ = false;
                    record(next, direction * step_limit_ / config_.dt, 0.0);
                    return;
                }
            }
            if (std::fabs(error) > kDriveAlignment) {
                const double omega = headingControl(error, config_.heading_gain, config_.max_angular_speed,
                                                    config_.heading_tolerance, config_.dt);
                if (dynamic_ && hitsObstacle(position_, position_, now, next)) {
                    collide();
                    return;
                }
                theta_ = integrateUnicycle({position_.x, position_.y, theta_}, 0.0, omega, config_.dt).theta;
                // A disc rotating in place sweeps no new area: no collision check.
                result_.turns += turning_ ? 0 : 1;
                turning_ = true;
                result_.turn_time += config_.dt;
                result_.total_rotation += std::fabs(omega) * config_.dt;
                ++result_.steps;
                record(next, 0.0, omega);
                return;
            }
            turning_ = false;
            theta_ = desired;  // residual error <= kDriveAlignment is absorbed
        }
        const StepResult step = stepToward(position_, waypoints_[target_], step_limit_);
        const bool margin_waived = gate_since_ >= 0.0 && now - gate_since_ >= dynamic_patience_ - 1e-9;
        // Following a timed plan (spaceTimeStep checked it for contact, waits
        // included), the reactive gate stands aside.
        if (dynamic_ && !schedule_valid_ &&
            !moveIsSafe(*view_, waypoints_, target_, position_, step_limit_, dynamic_->replanning, margin_waived)) {
            if (gate_since_ < 0.0) {
                gate_since_ = now;
            }
            // Look-ahead gate: driving on would meet a seen obstacle within the
            // prediction horizon, and standing still is safer. Let it pass.
            standStill(now, next);
            return;
        }
        if (dynamic_ && dynamic_->perception.occlusion_aware) {
            // Creep round blind corners: take the step only if, from where it leads,
            // nothing hidden could reach the robot before it can react; unless the
            // robot has waited out the patience, then commit for a while.
            if (phantom_commit_left_ <= 0.0 && view_->exposedToPhantoms(map_, step.position) &&
                !view_->exposedToPhantoms(map_, position_)) {
                if (phantom_since_ < 0.0) {
                    phantom_since_ = now;
                }
                if (now - phantom_since_ < dynamic_patience_ - 1e-9) {
                    standStill(now, next);
                    return;
                }
                phantom_commit_left_ = dynamic_->perception.phantom_commit;
            }
            phantom_since_ = -1.0;
            phantom_commit_left_ -= step.distance;
        }
        gate_since_ = -1.0;
        if (dynamic_ && !stepIsFree(position_, step.position)) {
            // Moving obstacles only: evasive steps can leave the robot off its planned
            // segments, and the straight line to the next waypoint may then clip a
            // wall. Get back onto the grid (the center of its cell) and plan on from
            // there, instead of driving into the wall.
            recenter();
            standStill(now, next);
            return;
        }
        if (!stepIsFree(position_, step.position) || (dynamic_ && hitsObstacle(position_, step.position, now, next))) {
            // The unsafe step is not executed; the robot stops at its last safe pose.
            collide();
            return;
        }
        const double direction = target_ == reverse_target_ ? -1.0 : 1.0;  // backwards?
        position_ = step.position;
        result_.distance_traveled += step.distance;
        ++result_.steps;
        if (step.reached) {
            if (target_ == reverse_target_) {
                reverse_target_ = kNoTarget;  // the retreat is complete
            }
            target_ = skipReachedWaypoints(waypoints_, target_ + 1, position_);
        }
        // Time is computed from the step count (not accumulated) to avoid drift.
        record(next, diff_drive_ ? direction * step.distance / config_.dt : 0.0, 0.0);
    }

    /// Back onto the grid: drive to the center of the robot's cell next; if it is
    /// there already, replace the rest of the path by a static plan from that cell.
    void recenter() {
        const GridCoord own = map_.worldToGrid(position_);
        const WorldPoint center = map_.gridToWorld(own);
        schedule_valid_ = false;
        reverse_target_ = kNoTarget;
        if (distanceBetween(position_, center) > kArrivalEpsilon) {
            cells_.insert(cells_.begin() + static_cast<long>(target_), own);
            waypoints_.insert(waypoints_.begin() + static_cast<long>(target_), center);
            return;
        }
        const PlanResult plan = planner_.plan(*dynamic_->planning_map, own, cells_.back(), connectivity_);
        if (plan.success() && plan.path.size() > 1) {
            cells_.resize(target_);
            waypoints_.resize(target_);
            cells_.back() = own;
            waypoints_.back() = center;
            for (std::size_t i = 1; i < plan.path.size(); ++i) {
                cells_.push_back(plan.path[i]);
                waypoints_.push_back(map_.gridToWorld(plan.path[i]));
            }
        }
    }

    static constexpr std::size_t kNoTarget = static_cast<std::size_t>(-1);

    const GridMap& map_;
    MotionConfig config_;
    Connectivity connectivity_;
    const DynamicScenario* dynamic_;
    SimulationResult result_;
    std::vector<GridCoord> cells_;
    std::vector<WorldPoint> waypoints_;
    WorldPoint goal_;
    double step_limit_ = 0.0;
    std::size_t max_steps_ = 0;
    WorldPoint position_;
    WorldPoint previous_;
    bool diff_drive_ = false;
    double theta_ = 0.0;
    std::size_t target_ = 0;
    std::optional<DynamicView> view_;
    AStarPlanner planner_;  // one planner for every replan: its buffers are reused
    bool turning_ = false;
    double blocked_since_ = -1.0;  // dynamic: when the current path became blocked
    double gate_since_ = -1.0;     // dynamic: when the look-ahead gate started holding the robot
    double phantom_since_ = -1.0;  // occlusion-aware: when a blind corner started holding it
    double phantom_commit_left_ = 0.0;  // occlusion-aware: meters still driven ignoring phantoms
    double dynamic_patience_ = 0.0;
    DynamicPlanner planner_mode_ = DynamicPlanner::Reactive;  // the strategy in use
    std::vector<double> cost_to_go_;  // livelock detection: static distance to the goal per cell, m
    double best_remaining_ = std::numeric_limits<double>::infinity();
    double progress_since_ = 0.0;
    double switched_at_ = 0.0;  // when the robot last switched away from its usual strategy
    bool switch_armed_ = true;   // a switch is allowed (re-armed by progress past the stuck place)
    double stuck_remaining_ = 0.0;  // remaining distance where the last switch happened, m
    std::size_t reverse_target_ = kNoTarget;  // dynamic: retreat waypoint, driven backwards
    bool finished_ = false;
    // SpaceTime planner: the timed plan being followed.
    bool schedule_valid_ = false;
    std::vector<double> departs_;  // departure time per waypoint (absolute), parallel to cells_
    double last_plan_ = -1e300;
};

Navigator::Navigator(const GridMap& map, const std::vector<GridCoord>& path, const MotionConfig& config,
                     Connectivity connectivity, const DynamicScenario* dynamic)
    : state_(std::make_unique<NavigatorState>(map, path, config, connectivity, dynamic)) {}
Navigator::~Navigator() = default;
Navigator::Navigator(Navigator&&) noexcept = default;
Navigator& Navigator::operator=(Navigator&&) noexcept = default;
bool Navigator::finished() const { return state_->finished(); }
void Navigator::step(const std::vector<MovingDisc>& others) { state_->step(others); }
void Navigator::collideWithRobot(int other) { state_->collideWithRobot(other); }
WorldPoint Navigator::position() const { return state_->position(); }
WorldPoint Navigator::previousPosition() const { return state_->previous(); }
const SimulationResult& Navigator::result() const { return state_->result(); }

SimulationResult simulatePath(const GridMap& map, const std::vector<GridCoord>& path,
                              const MotionConfig& config, Connectivity connectivity,
                              const DynamicScenario* dynamic) {
    Navigator navigator(map, path, config, connectivity, dynamic);
    while (!navigator.finished()) {
        navigator.step();
    }
    return navigator.result();
}

}  // namespace amr
