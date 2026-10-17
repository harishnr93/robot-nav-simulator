// Several robots that all replan: each robot navigates as in simulatePath (its own
// planner, perception and safety rules) and perceives the others like moving
// obstacles (oracle sightings, or discs in its LiDAR scans that it tracks).
//
// The robots are stepped together: every step, each robot decides from where the
// others are at the start of the step, then all move. Contact between two robots is
// checked by the caller, here, from how both moved: their centers move linearly over
// the step, and the closest approach of the two segments below the sum of the radii
// is a collision (for both). A robot that reached its goal, or crashed, stays where
// it is: a static disc for the others.
//
// Nobody coordinates: no shared plans, priorities or messages. What decides whether
// they pass each other is how their individual rules interact; two robots with the
// same rules in a symmetric situation react symmetrically (both back off, both come
// back: a livelock). The livelock switch (ReplanningConfig::livelock_window) changes
// strategy when a robot stops making progress; with `stagger_livelock` (default),
// each robot's window is a little longer than the previous robot's, so that two
// robots stuck together do not switch together (symmetry breaking).
#pragma once

#include <vector>

#include "grid_map.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace amr {

struct RobotTask {
    GridCoord start;
    GridCoord goal;
    double initial_heading = 0.0;  ///< rad (differential drive)
};

struct MultiRobotConfig {
    MotionConfig motion;                     ///< shared by all robots (radius > 0)
    Connectivity connectivity = Connectivity::Four;
    /// Replanning and perception (and any scripted obstacles), shared; planning_map
    /// must be the map inflated by the robot radius. Robot i's LiDAR noise seed is
    /// the scenario's seed + i.
    DynamicScenario scenario;
    /// Symmetry breaking: robot i's livelock window is (1 + 0.25 i) times the
    /// configured one. Without it, two robots stuck in a mirrored situation detect
    /// the livelock at the same instant, switch together and stay mirrored.
    bool stagger_livelock = true;
};

struct MultiRobotResult {
    std::vector<SimulationResult> robots;  ///< per task, in order
    bool planned = false;                  ///< every robot had an initial path
    int reached = 0;
    int robot_collisions = 0;              ///< robots that ended in contact with another robot
    int obstacle_collisions = 0;           ///< robots that hit a wall or a scripted obstacle
    int timeouts = 0;
    double makespan = 0.0;                 ///< time until the last robot finished, s
    int strategy_switches = 0;             ///< livelocks detected, all robots
};

/// Simulate all tasks together on `map` (planning on `scenario.planning_map`).
MultiRobotResult simulateRobots(const GridMap& map, const std::vector<RobotTask>& tasks,
                                const MultiRobotConfig& config);

}  // namespace amr
