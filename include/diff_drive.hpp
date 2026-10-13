// Differential-drive (unicycle) kinematics and the heading controller.
//
// State: pose (x, y, theta) in meters / radians, in the world frame (+x right,
// +y down, as everywhere in the project). theta = 0 faces +x; theta grows from
// +x toward +y.
//
// Control inputs: linear velocity v (m/s) and angular velocity omega (rad/s):
//
//     dx/dt = v cos(theta),   dy/dt = v sin(theta),   dtheta/dt = omega
//
// For inputs held constant over a timestep the equations have a closed-form
// solution (a straight line if omega == 0, otherwise a circular arc), which
// integrateUnicycle() uses: there is no integration error to accumulate.
#pragma once

namespace amr {

/// Robot pose: position in meters and heading in radians.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/// Angle wrapped to (-pi, pi].
double wrapAngle(double angle);

/// Exact pose after moving for dt seconds with constant v and omega.
/// The returned heading is wrapped to (-pi, pi].
Pose integrateUnicycle(const Pose& pose, double v, double omega, double dt);

/// Proportional heading controller with saturation:
///
///     omega = clamp(gain * error, -max_omega, +max_omega)
///
/// Once |error| <= tolerance, a final settling step returns error / dt, which
/// cancels the remaining error exactly in one timestep (callers guarantee that
/// this is within max_omega). With gain * dt <= 1 the heading approaches its
/// target monotonically, without overshoot.
double headingControl(double error, double gain, double max_omega, double tolerance, double dt);

}  // namespace amr
