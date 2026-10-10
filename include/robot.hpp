// Point-robot state, motion parameters and single-step kinematics.
//
// The robot is a point with no orientation, mass or footprint. Each timestep it
// moves in a straight line toward its current target waypoint at constant speed:
//
//     direction     = target - position
//     distance      = |direction|
//     step_distance = min(speed * dt, distance)        (clamped: no overshoot)
//     position_next = position + direction / distance * step_distance
//
// When the clamped step reaches the target the robot is placed exactly on it, so
// position errors never accumulate from one waypoint to the next. Any leftover
// step distance is not carried over to the next waypoint; the robot simply arrives
// mid-timestep, so its average speed over that timestep is below `speed`.
#pragma once

#include <string>

#include "types.hpp"

namespace amr {

/// Robot pose sample: position in meters and simulation time in seconds.
struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double time = 0.0;
};

/// Motion-simulation parameters. These are configurable defaults, not measured
/// hardware specifications.
struct MotionConfig {
    double speed = 0.5;            ///< linear speed, m/s
    double dt = 0.1;               ///< simulation timestep, s
    double goal_tolerance = 0.05;  ///< max final distance to the goal center, m
    double max_time = 600.0;       ///< simulation time limit, s (prevents endless runs)
};

/// Upper bound on max_time / dt, which bounds the trajectory's memory use.
constexpr double kMaxSimulationSteps = 1e7;

/// Distances at or below this are treated as "already at the target", m.
constexpr double kArrivalEpsilon = 1e-9;

/// Returns an empty string if the configuration is valid, otherwise the reason.
std::string validateMotionConfig(const MotionConfig& config);

/// Result of one kinematic step.
struct StepResult {
    WorldPoint position;     ///< position after the step
    double distance = 0.0;   ///< distance actually moved, m (<= max_step)
    bool reached = false;    ///< true if the robot is now exactly on the target
};

/// Move from `current` toward `target` by at most `max_step` meters.
/// Zero-distance targets are handled safely (no division by zero).
StepResult stepToward(WorldPoint current, WorldPoint target, double max_step);

}  // namespace amr
