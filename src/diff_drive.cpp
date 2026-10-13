#include "diff_drive.hpp"

#include <algorithm>
#include <cmath>

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;
/// Below this |omega * dt| the straight-line formula is used (the arc formula
/// divides by omega).
constexpr double kStraightThreshold = 1e-12;

}  // namespace

double wrapAngle(double angle) {
    double wrapped = std::remainder(angle, 2.0 * kPi);  // in [-pi, pi]
    if (wrapped <= -kPi) {
        wrapped += 2.0 * kPi;
    }
    return wrapped;
}

Pose integrateUnicycle(const Pose& pose, double v, double omega, double dt) {
    const double turn = omega * dt;
    Pose next;
    if (std::fabs(turn) < kStraightThreshold) {
        next.x = pose.x + v * dt * std::cos(pose.theta);
        next.y = pose.y + v * dt * std::sin(pose.theta);
    } else {
        // Circular arc of radius v / omega.
        const double radius = v / omega;
        next.x = pose.x + radius * (std::sin(pose.theta + turn) - std::sin(pose.theta));
        next.y = pose.y - radius * (std::cos(pose.theta + turn) - std::cos(pose.theta));
    }
    next.theta = wrapAngle(pose.theta + turn);
    return next;
}

double headingControl(double error, double gain, double max_omega, double tolerance, double dt) {
    if (std::fabs(error) <= tolerance) {
        return error / dt;  // final settling step: lands exactly on the target heading
    }
    return std::clamp(gain * error, -max_omega, max_omega);
}

}  // namespace amr
