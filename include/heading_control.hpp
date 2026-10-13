// Heading control study: P vs. PID on a robot heading with actuator dynamics.
//
// Plant (the robot's heading, in rad):
//
//     dtheta/dt   = omega + d
//     domega/dt   = (u - omega) / tau          (first-order motor lag)
//
// u is the commanded turn rate, limited to |u| <= max_omega. omega is the actual
// turn rate, which follows the command with time constant tau (tau = 0: no lag,
// omega = u). d is a constant disturbance turn rate (e.g. one wheel slipping). With
// u held constant over a control period (zero-order hold) the plant has an exact
// discrete solution, which HeadingPlant::step() uses.
//
// Controllers (error e = wrap(theta_ref - theta)):
//
//     P:    u = Kp e
//     PID:  u = Kp e + Ki * integral(e dt) - Kd * dtheta/dt
//
// The derivative acts on the measured heading rather than on the error, so a step
// in the reference does not produce a spike in u ("derivative kick"). Integral
// anti-windup: the integral is not advanced while the output is saturated in the
// direction of the error (conditional integration). Both outputs are clamped to
// +-max_omega.
#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace amr {

/// Heading plant with first-order actuator lag and a constant disturbance.
struct HeadingPlant {
    double tau = 0.0;          ///< actuator time constant, s (0 = no lag)
    double disturbance = 0.0;  ///< constant extra turn rate d, rad/s
    double max_omega = 1.5;    ///< command limit |u|, rad/s

    double theta = 0.0;        ///< heading, rad (not wrapped)
    double omega = 0.0;        ///< actual turn rate, rad/s

    /// Advance the plant by dt with command u (clamped to +-max_omega), exactly.
    void step(double u, double dt);
};

enum class HeadingControllerKind { P, PID };

struct HeadingGains {
    double kp = 4.0;   ///< 1/s
    double ki = 0.0;   ///< 1/s^2 (PID only)
    double kd = 0.0;   ///< dimensionless (PID only): u gets -kd * dtheta/dt
};

/// P or PID heading controller with output saturation and anti-windup.
class HeadingController {
public:
    HeadingController(HeadingControllerKind kind, const HeadingGains& gains, double max_omega);

    /// Command for this control period. theta_ref and theta in rad, dt in s.
    double update(double theta_ref, double theta, double dt);
    void reset();

    HeadingControllerKind kind() const { return kind_; }
    const HeadingGains& gains() const { return gains_; }
    double integral() const { return integral_; }

private:
    HeadingControllerKind kind_;
    HeadingGains gains_;
    double max_omega_;
    double integral_ = 0.0;
    double previous_theta_ = 0.0;
    bool has_previous_ = false;
};

/// One sample of a closed-loop heading simulation.
struct HeadingSample {
    double time = 0.0;
    double reference = 0.0;  ///< theta_ref, rad
    double theta = 0.0;      ///< heading, rad
    double error = 0.0;      ///< wrap(theta_ref - theta), rad
    double command = 0.0;    ///< u applied from this sample to the next, rad/s
    double omega = 0.0;      ///< actual turn rate at this sample, rad/s
};

/// Simulate the closed loop for `duration` seconds with control period dt.
/// reference(t) gives theta_ref; the plant starts at rest with heading theta0.
std::vector<HeadingSample> simulateHeadingLoop(const std::function<double(double)>& reference,
                                               HeadingPlant plant, HeadingController controller,
                                               double theta0, double dt, double duration);

/// Performance metrics of a closed-loop run.
struct HeadingMetrics {
    // Step metrics (meaningful for a step reference of size step_size != 0).
    double overshoot_pct = 0.0;      ///< max excursion beyond the target, % of the step
    double overshoot_rad = 0.0;
    double rise_time = -1.0;         ///< 10 % -> 90 % of the step, s (-1: never)
    double settling_time = -1.0;     ///< last time |e| > band, s (-1: never settles)
    // Any reference.
    double steady_state_error = 0.0; ///< mean e over the final steady_window seconds, rad
    double iae = 0.0;                ///< integral of |e| dt, rad s
    double rms_error = 0.0;          ///< sqrt(mean e^2), rad
    double max_abs_error_after = 0.0;///< max |e| after the first settle_after seconds, rad
    double effort_abs = 0.0;         ///< integral of |u| dt, rad
    double effort_sq = 0.0;          ///< integral of u^2 dt, rad^2/s
    double command_variation = 0.0;  ///< sum of |u_k - u_(k-1)|, rad/s
    double saturated_fraction = 0.0; ///< share of periods with |u| = max_omega
};

/// Compute metrics. step_size: size of the reference step (0 for non-step
/// references); band: settling band, rad; steady_window / settle_after in s.
HeadingMetrics computeHeadingMetrics(const std::vector<HeadingSample>& samples, double step_size,
                                     double band, double steady_window, double settle_after,
                                     double max_omega);

std::string toString(HeadingControllerKind kind);

}  // namespace amr
