#include "heading_control.hpp"

#include <algorithm>
#include <cmath>

#include "diff_drive.hpp"

namespace amr {

void HeadingPlant::step(double u, double dt) {
    u = std::clamp(u, -max_omega, max_omega);
    if (tau <= 0.0) {
        theta += (u + disturbance) * dt;
        omega = u;
        return;
    }
    // omega(t) = u + (omega0 - u) e^{-t/tau}; integrate it over [0, dt].
    const double decay = std::exp(-dt / tau);
    theta += (u + disturbance) * dt + (omega - u) * tau * (1.0 - decay);
    omega = u + (omega - u) * decay;
}

HeadingController::HeadingController(HeadingControllerKind kind, const HeadingGains& gains,
                                     double max_omega)
    : kind_(kind), gains_(gains), max_omega_(max_omega) {}

void HeadingController::reset() {
    integral_ = 0.0;
    previous_theta_ = 0.0;
    has_previous_ = false;
}

double HeadingController::update(double theta_ref, double theta, double dt) {
    const double error = wrapAngle(theta_ref - theta);
    if (kind_ == HeadingControllerKind::P) {
        return std::clamp(gains_.kp * error, -max_omega_, max_omega_);
    }
    // Derivative on the measurement: no kick when the reference steps.
    const double rate = has_previous_ ? wrapAngle(theta - previous_theta_) / dt : 0.0;
    previous_theta_ = theta;
    has_previous_ = true;

    const double unsaturated_without_new_integral =
        gains_.kp * error + gains_.ki * integral_ - gains_.kd * rate;
    // Conditional integration (anti-windup): do not integrate further into a
    // saturated output.
    const bool pushing_into_limit =
        (unsaturated_without_new_integral >= max_omega_ && error > 0.0) ||
        (unsaturated_without_new_integral <= -max_omega_ && error < 0.0);
    if (!pushing_into_limit) {
        integral_ += error * dt;
    }
    const double u = gains_.kp * error + gains_.ki * integral_ - gains_.kd * rate;
    return std::clamp(u, -max_omega_, max_omega_);
}

std::vector<HeadingSample> simulateHeadingLoop(const std::function<double(double)>& reference,
                                               HeadingPlant plant, HeadingController controller,
                                               double theta0, double dt, double duration) {
    plant.theta = theta0;
    plant.omega = 0.0;
    controller.reset();
    const auto steps = static_cast<std::size_t>(std::llround(duration / dt));
    std::vector<HeadingSample> samples;
    samples.reserve(steps + 1);
    for (std::size_t k = 0; k <= steps; ++k) {
        const double t = static_cast<double>(k) * dt;  // no accumulated rounding
        const double ref = reference(t);
        HeadingSample sample{t, ref, plant.theta, wrapAngle(ref - plant.theta), 0.0, plant.omega};
        if (k < steps) {
            sample.command = std::clamp(controller.update(ref, plant.theta, dt),
                                        -plant.max_omega, plant.max_omega);
            plant.step(sample.command, dt);
        }
        samples.push_back(sample);
    }
    return samples;
}

HeadingMetrics computeHeadingMetrics(const std::vector<HeadingSample>& samples, double step_size,
                                     double band, double steady_window, double settle_after,
                                     double max_omega) {
    HeadingMetrics m;
    if (samples.size() < 2) {
        return m;
    }
    const double dt = samples[1].time - samples[0].time;
    const double end_time = samples.back().time;
    double sum_sq = 0.0;
    double steady_sum = 0.0;
    int steady_count = 0;
    int saturated = 0;
    for (std::size_t k = 0; k < samples.size(); ++k) {
        const HeadingSample& s = samples[k];
        sum_sq += s.error * s.error;
        if (k + 1 < samples.size()) {  // the command of the last sample is never applied
            m.iae += std::fabs(s.error) * dt;
            m.effort_abs += std::fabs(s.command) * dt;
            m.effort_sq += s.command * s.command * dt;
            saturated += std::fabs(s.command) >= max_omega - 1e-12;
            if (k > 0) {
                m.command_variation += std::fabs(s.command - samples[k - 1].command);
            }
        }
        if (s.time >= end_time - steady_window - 1e-12) {
            steady_sum += s.error;
            ++steady_count;
        }
        if (s.time >= settle_after - 1e-12) {
            m.max_abs_error_after = std::max(m.max_abs_error_after, std::fabs(s.error));
        }
    }
    m.rms_error = std::sqrt(sum_sq / static_cast<double>(samples.size()));
    m.steady_state_error = steady_count > 0 ? steady_sum / steady_count : 0.0;
    m.saturated_fraction = static_cast<double>(saturated) / static_cast<double>(samples.size() - 1);

    if (step_size != 0.0) {
        const double start = samples.front().theta;
        double peak = 0.0;     // largest progress toward the target, as a fraction of the step
        double t10 = -1.0;     // first time progress reaches 10 % / 90 %
        double t90 = -1.0;
        for (const HeadingSample& s : samples) {
            const double progress = (s.theta - start) / step_size;
            peak = std::max(peak, progress);
            if (t10 < 0.0 && progress >= 0.1) {
                t10 = s.time;
            }
            if (t90 < 0.0 && progress >= 0.9) {
                t90 = s.time;
            }
        }
        if (t10 >= 0.0 && t90 >= 0.0) {
            m.rise_time = t90 - t10;
        }
        m.overshoot_pct = std::max(0.0, (peak - 1.0) * 100.0);
        m.overshoot_rad = std::max(0.0, (peak - 1.0) * std::fabs(step_size));
        // Settling: the time after which |e| stays within the band.
        double last_outside = -1.0;
        for (const HeadingSample& s : samples) {
            if (std::fabs(s.error) > band) {
                last_outside = s.time;
            }
        }
        if (last_outside < 0.0) {
            m.settling_time = 0.0;
        } else if (last_outside < end_time - 1e-12) {
            m.settling_time = last_outside + dt;
        }  // else: still outside the band at the end, stays -1
    }
    return m;
}

std::string toString(HeadingControllerKind kind) {
    return kind == HeadingControllerKind::P ? "P" : "PID";
}

}  // namespace amr
