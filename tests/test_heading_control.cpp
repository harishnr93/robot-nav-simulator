// Tests for the heading-control study: plant model, P and PID controllers, and
// the performance metrics.
#include <cmath>
#include <vector>

#include "heading_control.hpp"
#include "test_framework.hpp"

using amr::HeadingController;
using amr::HeadingControllerKind;
using amr::HeadingGains;
using amr::HeadingPlant;
using amr::HeadingSample;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTau = 0.25;
constexpr double kMaxOmega = 1.5;

std::vector<HeadingSample> run(HeadingControllerKind kind, HeadingGains gains, double tau,
                               double disturbance, double target, double dt, double duration) {
    HeadingPlant plant{tau, disturbance, kMaxOmega};
    HeadingController controller(kind, gains, kMaxOmega);
    return amr::simulateHeadingLoop([target](double) { return target; }, plant, controller, 0.0,
                                    dt, duration);
}

void testPlantIsExact() {
    // Constant command: compare the closed form with a very fine Euler integration.
    HeadingPlant exact{kTau, 0.1, 5.0};
    exact.omega = -0.4;
    HeadingPlant euler = exact;
    exact.step(1.2, 0.5);
    const int substeps = 500000;
    const double h = 0.5 / substeps;
    for (int i = 0; i < substeps; ++i) {
        euler.theta += (euler.omega + euler.disturbance) * h;
        euler.omega += (1.2 - euler.omega) / kTau * h;
    }
    CHECK_NEAR(exact.theta, euler.theta, 1e-5);
    CHECK_NEAR(exact.omega, euler.omega, 1e-5);
    // Splitting a step in two gives the same result (exact zero-order-hold solution).
    HeadingPlant once{kTau, 0.2, 5.0};
    HeadingPlant twice = once;
    once.step(0.9, 0.2);
    twice.step(0.9, 0.1);
    twice.step(0.9, 0.1);
    CHECK_NEAR(once.theta, twice.theta, 1e-12);
    CHECK_NEAR(once.omega, twice.omega, 1e-12);
}

void testPlantWithoutLagAndSaturation() {
    HeadingPlant plant{0.0, 0.05, kMaxOmega};
    plant.step(1.0, 0.1);
    CHECK_NEAR(plant.theta, 0.105, 1e-12);  // (u + d) * dt
    CHECK_NEAR(plant.omega, 1.0, 1e-12);
    plant.step(10.0, 0.1);                   // clamped to 1.5 rad/s
    CHECK_NEAR(plant.omega, kMaxOmega, 1e-12);
    CHECK_NEAR(plant.theta, 0.105 + 0.155, 1e-12);
}

// With actuator lag, P control gives the closed loop tau s^2 + s + Kp: a damped
// second-order system with zeta = 1 / (2 sqrt(Kp tau)) and overshoot
// exp(-zeta pi / sqrt(1 - zeta^2)). A small step keeps the command unsaturated.
void testPOvershootMatchesTheory() {
    for (const double kp : {2.0, 4.0, 8.0}) {
        const auto samples = run(HeadingControllerKind::P, {kp, 0, 0}, kTau, 0.0, 0.1, 0.0005, 10.0);
        const auto m = amr::computeHeadingMetrics(samples, 0.1, 0.002, 1.0, 0.0, kMaxOmega);
        const double zeta = 1.0 / (2.0 * std::sqrt(kp * kTau));
        const double theory = 100.0 * std::exp(-zeta * kPi / std::sqrt(1.0 - zeta * zeta));
        CHECK_NEAR(m.overshoot_pct, theory, 0.1);  // percentage points
        CHECK(m.saturated_fraction == 0.0);
    }
}

void testSteadyStateErrors() {
    // Constant disturbance d: P settles at e = -d / Kp; PID removes it.
    const double d = 0.3;
    const auto p = run(HeadingControllerKind::P, {4, 0, 0}, kTau, d, kPi / 2, 0.01, 20.0);
    const auto pid = run(HeadingControllerKind::PID, {4, 2, 0.4}, kTau, d, kPi / 2, 0.01, 30.0);
    const auto mp = amr::computeHeadingMetrics(p, kPi / 2, 0.0314, 1.0, 0.0, kMaxOmega);
    const auto mpid = amr::computeHeadingMetrics(pid, kPi / 2, 0.0314, 1.0, 0.0, kMaxOmega);
    CHECK_NEAR(mp.steady_state_error, -d / 4.0, 1e-4);
    CHECK(std::fabs(mpid.steady_state_error) < 1e-4);
    CHECK(mp.settling_time < 0.0);       // never enters the 2 % band (|e| = 0.075 > 0.0314)
    CHECK(mpid.settling_time > 0.0);

    // Ramp reference at rate w: P lags by w / Kp; PID tracks it.
    const double w = 0.5;
    HeadingPlant plant{kTau, 0.0, kMaxOmega};
    const auto ramp = [w](double t) { return w * t; };
    const auto pr = amr::simulateHeadingLoop(
        ramp, plant, HeadingController(HeadingControllerKind::P, {4, 0, 0}, kMaxOmega), 0.0, 0.01,
        20.0);
    const auto pidr = amr::simulateHeadingLoop(
        ramp, plant, HeadingController(HeadingControllerKind::PID, {4, 2, 0.4}, kMaxOmega), 0.0,
        0.01, 30.0);
    CHECK_NEAR(amr::computeHeadingMetrics(pr, 0, 0, 1.0, 0, kMaxOmega).steady_state_error,
               w / 4.0, 1e-4);
    CHECK(std::fabs(amr::computeHeadingMetrics(pidr, 0, 0, 1.0, 0, kMaxOmega).steady_state_error) <
          1e-4);
}

void testNoDerivativeKick() {
    // The reference steps while the heading is at rest: the first PID command
    // contains no derivative term (the derivative acts on the measured heading).
    HeadingController pid(HeadingControllerKind::PID, {1.0, 0.5, 5.0}, 100.0);
    const double u0 = pid.update(0.2, 0.0, 0.01);
    CHECK_NEAR(u0, 1.0 * 0.2 + 0.5 * 0.2 * 0.01, 1e-12);
    // A moving heading is damped: theta increased by 0.01 rad in 0.01 s -> -Kd * 1 rad/s.
    const double u1 = pid.update(0.2, 0.01, 0.01);
    CHECK_NEAR(u1, 1.0 * 0.19 + 0.5 * (0.002 + 0.0019) - 5.0 * 1.0, 1e-12);
}

void testAntiWindup() {
    // Saturated and pushing further into the limit: the integral must not grow.
    HeadingController pid(HeadingControllerKind::PID, {4, 2, 0.4}, kMaxOmega);
    for (int i = 0; i < 100; ++i) {
        CHECK_NEAR(pid.update(kPi / 2, 0.0, 0.01), kMaxOmega, 1e-12);
    }
    CHECK_NEAR(pid.integral(), 0.0, 1e-12);
    // Unsaturated: it integrates normally.
    HeadingController small(HeadingControllerKind::PID, {4, 2, 0.4}, kMaxOmega);
    small.update(0.05, 0.0, 0.01);
    small.update(0.05, 0.0, 0.01);
    CHECK_NEAR(small.integral(), 0.001, 1e-12);
}

void testPidWithoutIandDEqualsP() {
    const auto p = run(HeadingControllerKind::P, {4, 0, 0}, kTau, 0.1, kPi / 2, 0.01, 5.0);
    const auto pid = run(HeadingControllerKind::PID, {4, 0, 0}, kTau, 0.1, kPi / 2, 0.01, 5.0);
    REQUIRE(p.size() == pid.size());
    for (std::size_t i = 0; i < p.size(); ++i) {
        CHECK_EQ(p[i].theta, pid[i].theta);
        CHECK_EQ(p[i].command, pid[i].command);
    }
}

void testCommandsRespectTheLimit() {
    for (const auto kind : {HeadingControllerKind::P, HeadingControllerKind::PID}) {
        const auto samples = run(kind, {8, 4, 0.2}, kTau, 0.3, kPi, 0.01, 10.0);
        for (const auto& s : samples) {
            CHECK(std::fabs(s.command) <= kMaxOmega + 1e-12);
            CHECK(std::isfinite(s.theta));
        }
    }
}

void testMetricsOnKnownSignals() {
    // Hand-made step response (step 1, dt 1): 0, 0.2, 0.5, 0.95, 1.1, 1.05, 0.99, 1.0, 1.0
    const std::vector<double> theta{0, 0.2, 0.5, 0.95, 1.1, 1.05, 0.99, 1.0, 1.0};
    std::vector<HeadingSample> samples;
    for (std::size_t k = 0; k < theta.size(); ++k) {
        samples.push_back({static_cast<double>(k), 1.0, theta[k], 1.0 - theta[k],
                           k < 3 ? 2.0 : 0.5, 0.0});
    }
    const auto m = amr::computeHeadingMetrics(samples, 1.0, 0.02, 2.0, 4.0, 2.0);
    CHECK_NEAR(m.overshoot_pct, 10.0, 1e-9);
    CHECK_NEAR(m.overshoot_rad, 0.1, 1e-9);
    CHECK_NEAR(m.rise_time, 3.0 - 1.0, 1e-9);     // 10 % at t = 1, 90 % at t = 3
    CHECK_NEAR(m.settling_time, 6.0, 1e-9);       // last |e| > 0.02 at t = 5 (|e| = 0.05)
    CHECK_NEAR(m.steady_state_error, (0.01 + 0.0 + 0.0) / 3.0, 1e-9);  // t = 6, 7, 8
    CHECK_NEAR(m.iae, 1.0 + 0.8 + 0.5 + 0.05 + 0.1 + 0.05 + 0.01 + 0.0, 1e-9);
    CHECK_NEAR(m.effort_abs, 3 * 2.0 + 5 * 0.5, 1e-9);   // the last command is not applied
    CHECK_NEAR(m.command_variation, 1.5, 1e-9);
    CHECK_NEAR(m.saturated_fraction, 3.0 / 8.0, 1e-9);
    CHECK_NEAR(m.max_abs_error_after, 0.1, 1e-9);        // from t = 4 on
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Plant: exact solution (vs. Euler, split step)", testPlantIsExact},
        {"Plant without lag; command saturation", testPlantWithoutLagAndSaturation},
        {"P overshoot matches 2nd-order theory (Kp 2, 4, 8)", testPOvershootMatchesTheory},
        {"Steady-state errors: P = d/Kp and w/Kp, PID = 0", testSteadyStateErrors},
        {"No derivative kick; derivative damps motion", testNoDerivativeKick},
        {"Integral anti-windup", testAntiWindup},
        {"PID with Ki = Kd = 0 is exactly P", testPidWithoutIandDEqualsP},
        {"Commands respect the angular-speed limit", testCommandsRespectTheLimit},
        {"Metrics on a hand-made response", testMetricsOnKnownSignals},
    });
}
