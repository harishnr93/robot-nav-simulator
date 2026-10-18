#include "multi_robot.hpp"

#include <algorithm>
#include <cmath>

#include "astar.hpp"
#include "dynamic_obstacles.hpp"

namespace amr {

MultiRobotResult simulateRobots(const GridMap& map, const std::vector<RobotTask>& tasks,
                                const MultiRobotConfig& config) {
    MultiRobotResult out;
    const std::size_t n = tasks.size();
    if (n == 0 || config.scenario.planning_map == nullptr) {
        return out;
    }
    // Every robot plans its first path alone (it has not seen the others yet).
    std::vector<std::vector<GridCoord>> paths;
    for (const RobotTask& task : tasks) {
        const PlanResult plan = planAStar(*config.scenario.planning_map, task.start, task.goal, config.connectivity);
        if (!plan.success()) {
            return out;
        }
        paths.push_back(plan.path);
    }
    out.planned = true;
    // Per-robot scenarios (own noise seed, own livelock window); the navigators keep
    // pointers to them, so the vector is not resized after this.
    std::vector<DynamicScenario> scenarios(n, config.scenario);
    std::vector<Navigator> robots;
    robots.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        scenarios[i].perception.lidar.seed = config.scenario.perception.lidar.seed + static_cast<unsigned>(i);
        if (config.stagger_livelock) {
            scenarios[i].replanning.livelock_window *= 1.0 + 0.25 * static_cast<double>(i);
        }
        MotionConfig motion = config.motion;
        motion.initial_heading = tasks[i].initial_heading;
        robots.emplace_back(map, paths[i], motion, config.connectivity, &scenarios[i]);
    }
    const double radius = config.motion.robot_radius;
    const auto discsExcept = [&](std::size_t i) {
        std::vector<MovingDisc> others;
        for (std::size_t j = 0; j < n; ++j) {
            if (j != i) {
                others.push_back({static_cast<int>(j), robots[j].position(), radius});
            }
        }
        return others;
    };
    // Robots that start in contact.
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const WorldPoint a = robots[i].position(), b = robots[j].position();
            if (std::hypot(a.x - b.x, a.y - b.y) < 2.0 * radius) {
                robots[i].collideWithRobot(static_cast<int>(j));
                robots[j].collideWithRobot(static_cast<int>(i));
            }
        }
    }
    std::vector<WorldPoint> before(n), after(n);
    while (std::any_of(robots.begin(), robots.end(), [](const Navigator& r) { return !r.finished(); })) {
        // Everyone decides from where the others are now, then all move.
        std::vector<std::vector<MovingDisc>> views(n);
        for (std::size_t i = 0; i < n; ++i) {
            before[i] = robots[i].position();
            if (!robots[i].finished()) {
                views[i] = discsExcept(i);
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (!robots[i].finished()) {
                robots[i].step(views[i]);
            }
            after[i] = robots[i].position();
        }
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                if (closestApproach(before[i], after[i], before[j], after[j]) < 2.0 * radius - 1e-9) {
                    robots[i].collideWithRobot(static_cast<int>(j));
                    robots[j].collideWithRobot(static_cast<int>(i));
                }
            }
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        const SimulationResult& r = robots[i].result();
        out.reached += r.success() ? 1 : 0;
        out.robot_collisions += r.collided_robot >= 0 ? 1 : 0;
        out.obstacle_collisions += r.status == NavigationStatus::Collision && r.collided_robot < 0 ? 1 : 0;
        out.timeouts += r.status == NavigationStatus::Timeout ? 1 : 0;
        out.strategy_switches += r.strategy_switches;
        if (!r.trajectory.empty()) {
            out.makespan = std::max(out.makespan, r.trajectory.back().state.time);
        }
        out.robots.push_back(r);
    }
    return out;
}

}  // namespace amr
