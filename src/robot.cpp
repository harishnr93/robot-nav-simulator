#include "robot.hpp"

#include <cmath>

namespace amr {

namespace {

bool isPositiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

}  // namespace

std::string validateMotionConfig(const MotionConfig& config) {
    if (!isPositiveFinite(config.speed)) {
        return "speed must be a positive finite number";
    }
    if (!isPositiveFinite(config.dt)) {
        return "timestep dt must be a positive finite number";
    }
    if (!std::isfinite(config.goal_tolerance) || config.goal_tolerance < 0.0) {
        return "goal tolerance must be a non-negative finite number";
    }
    if (!isPositiveFinite(config.max_time)) {
        return "maximum simulation time must be a positive finite number";
    }
    if (!std::isfinite(config.robot_radius) || config.robot_radius < 0.0 ||
        config.robot_radius > kMaxRobotRadius) {
        return "robot radius must be between 0 and 100 m";
    }
    if (config.model == RobotModel::DiffDrive) {  // parameters used only by this model
        if (!isPositiveFinite(config.max_angular_speed)) {
            return "maximum angular speed must be a positive finite number";
        }
        if (!isPositiveFinite(config.heading_gain) || config.heading_gain * config.dt > 1.0) {
            return "heading gain must be positive with heading_gain * dt <= 1 (no overshoot)";
        }
        if (!isPositiveFinite(config.heading_tolerance) ||
            config.heading_tolerance > config.max_angular_speed * config.dt) {
            return "heading tolerance must be positive and at most max_angular_speed * dt";
        }
        if (!std::isfinite(config.initial_heading)) {
            return "initial heading must be finite";
        }
    }
    if (config.max_time / config.dt > kMaxSimulationSteps) {
        return "max_time / dt exceeds the supported number of simulation steps";
    }
    return {};
}

StepResult stepToward(WorldPoint current, WorldPoint target, double max_step) {
    const double dx = target.x - current.x;
    const double dy = target.y - current.y;
    const double distance = std::hypot(dx, dy);

    StepResult result;
    if (distance <= max_step + kArrivalEpsilon) {
        // Final (clamped) step: land exactly on the target.
        result.position = target;
        result.distance = distance;
        result.reached = true;
        return result;
    }
    const double scale = max_step / distance;  // distance > max_step >= 0, so no div by 0
    result.position = WorldPoint{current.x + dx * scale, current.y + dy * scale};
    result.distance = max_step;
    result.reached = false;
    return result;
}

}  // namespace amr
