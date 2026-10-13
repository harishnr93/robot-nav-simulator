#include "ekf.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace amr {

namespace {

/// Grazing beams (|cos| or |sin| of the angle to the face normal below this,
/// i.e. incidence above ~78 deg) are not used: their range changes too fast with
/// the pose for the linearization to hold.
constexpr double kMinIncidenceCosine = 0.2;

Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
    Matrix3 c{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                c[3 * i + j] += a[3 * i + k] * b[3 * k + j];
            }
        }
    }
    return c;
}

Matrix3 transpose(const Matrix3& a) {
    return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

/// Average with the transpose: removes the rounding asymmetry of a covariance.
Matrix3 symmetrize(const Matrix3& a) {
    Matrix3 s = a;
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            const double mean = 0.5 * (a[3 * i + j] + a[3 * j + i]);
            s[3 * i + j] = mean;
            s[3 * j + i] = mean;
        }
    }
    return s;
}

/// Inverse by cofactors. The callers only invert covariance / information
/// matrices, which are positive definite.
Matrix3 inverse(const Matrix3& m) {
    const Matrix3 cofactor{
        m[4] * m[8] - m[5] * m[7], m[5] * m[6] - m[3] * m[8], m[3] * m[7] - m[4] * m[6],
        m[2] * m[7] - m[1] * m[8], m[0] * m[8] - m[2] * m[6], m[1] * m[6] - m[0] * m[7],
        m[1] * m[5] - m[2] * m[4], m[2] * m[3] - m[0] * m[5], m[0] * m[4] - m[1] * m[3]};
    const double det = m[0] * cofactor[0] + m[1] * cofactor[1] + m[2] * cofactor[2];
    Matrix3 inv = transpose(cofactor);
    for (double& v : inv) {
        v /= det;
    }
    return inv;
}

double quadratic(const std::array<double, 3>& v, const Matrix3& m) {
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            sum += v[i] * m[3 * i + j] * v[j];
        }
    }
    return sum;
}

std::array<double, 3> poseError(const Pose& truth, const Pose& estimate) {
    return {truth.x - estimate.x, truth.y - estimate.y, wrapAngle(truth.theta - estimate.theta)};
}

Pose poseOf(const RobotState& state) {
    return {state.x, state.y, state.theta};
}

bool validNoise(double value) {
    return std::isfinite(value) && value >= 0.0 && value <= kMaxOdometryNoise;
}

}  // namespace

std::string validateEkfConfig(const EkfConfig& config) {
    if (!config.enabled) {
        return {};
    }
    const OdometryNoise& noise = config.odometry;
    if (!validNoise(noise.distance) || !validNoise(noise.turn) || !validNoise(noise.drift)) {
        return "odometry noise must be between 0 and 10";
    }
    if (!std::isfinite(config.range_sigma) || config.range_sigma <= 0.0 ||
        config.range_sigma > kMaxRangeSigma) {
        return "EKF range sigma must be in (0, 100] m";
    }
    if (!std::isfinite(config.gate) || config.gate <= 0.0) {
        return "EKF gate must be a positive number of standard deviations";
    }
    if (!std::isfinite(config.initial_position_sigma) || config.initial_position_sigma <= 0.0 ||
        !std::isfinite(config.initial_heading_sigma) || config.initial_heading_sigma <= 0.0) {
        return "initial EKF uncertainty must be positive";
    }
    return {};
}

OdometryIncrement trueIncrement(const RobotState& from, const RobotState& to) {
    // Negative distance when the robot backed up (a retreat from a moving obstacle).
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const bool backwards = (to.x - from.x) * std::cos(to.theta) +
                               (to.y - from.y) * std::sin(to.theta) < 0.0;
    return {backwards ? -distance : distance, wrapAngle(to.theta - from.theta)};
}

Pose applyIncrement(const Pose& pose, const OdometryIncrement& increment) {
    const double theta = pose.theta + increment.rotation;
    return {pose.x + increment.distance * std::cos(theta),
            pose.y + increment.distance * std::sin(theta), wrapAngle(theta)};
}

std::array<double, 2> odometryVariances(const OdometryIncrement& increment,
                                        const OdometryNoise& noise) {
    const double distance = std::fabs(increment.distance);
    const double rotation = std::fabs(increment.rotation);
    return {noise.distance * noise.distance * distance,
            noise.turn * noise.turn * rotation + noise.drift * noise.drift * distance};
}

std::array<double, 3> rangeJacobian(RayFace face, double angle, double range) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    if (face == RayFace::Vertical) {
        return {-1.0 / c, 0.0, range * s / c};
    }
    if (face == RayFace::Horizontal) {
        return {0.0, -1.0 / s, -range * c / s};
    }
    return {0.0, 0.0, 0.0};
}

PoseEkf::PoseEkf(const Pose& initial, const Matrix3& covariance)
    : pose_(initial), covariance_(covariance) {}

void PoseEkf::predict(const OdometryIncrement& measured, const std::array<double, 2>& m) {
    const double ds = measured.distance;
    const double theta = pose_.theta + measured.rotation;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    // F = d(pose') / d(pose); V = d(pose') / d(ds, dphi).
    const Matrix3 f{1.0, 0.0, -ds * s, 0.0, 1.0, ds * c, 0.0, 0.0, 1.0};
    const double v[3][2] = {{c, -ds * s}, {s, ds * c}, {0.0, 1.0}};
    Matrix3 predicted = multiply(multiply(f, covariance_), transpose(f));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            predicted[3 * i + j] += v[i][0] * m[0] * v[j][0] + v[i][1] * m[1] * v[j][1];
        }
    }
    covariance_ = symmetrize(predicted);
    pose_ = applyIncrement(pose_, measured);
}

UpdateStats PoseEkf::update(const GridMap& map, const std::vector<double>& offsets,
                            const std::vector<double>& ranges, const std::vector<bool>& hits,
                            double max_range, double range_sigma, double gate) {
    UpdateStats stats;
    const double r = range_sigma * range_sigma;
    Matrix3 information{};               // sum H^T H / R
    std::array<double, 3> correction{};  // sum H^T nu / R
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        if (!hits[i]) {
            continue;
        }
        const double angle = pose_.theta + offsets[i];
        const RayHit predicted = castRay(map, {pose_.x, pose_.y}, angle, max_range);
        if (!predicted.hit || predicted.range <= 0.0) {
            continue;
        }
        const double incidence = predicted.face == RayFace::Vertical     ? std::cos(angle)
                                 : predicted.face == RayFace::Horizontal ? std::sin(angle)
                                                                         : 0.0;
        if (std::fabs(incidence) < kMinIncidenceCosine) {
            continue;  // corner or grazing beam
        }
        const std::array<double, 3> h = rangeJacobian(predicted.face, angle, predicted.range);
        const double innovation = ranges[i] - predicted.range;
        const double variance = quadratic(h, covariance_) + r;
        if (innovation * innovation > gate * gate * variance) {
            ++stats.rejected;
            continue;
        }
        ++stats.used;
        for (int a = 0; a < 3; ++a) {
            correction[a] += h[a] * innovation / r;
            for (int b = 0; b < 3; ++b) {
                information[3 * a + b] += h[a] * h[b] / r;
            }
        }
    }
    if (stats.used == 0) {
        return stats;
    }
    Matrix3 posterior_information = inverse(covariance_);
    for (int k = 0; k < 9; ++k) {
        posterior_information[k] += information[k];
    }
    covariance_ = symmetrize(inverse(posterior_information));
    double delta[3] = {0.0, 0.0, 0.0};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            delta[a] += covariance_[3 * a + b] * correction[b];
        }
    }
    pose_ = {pose_.x + delta[0], pose_.y + delta[1], wrapAngle(pose_.theta + delta[2])};
    return stats;
}

std::vector<LocalizationSample> runLocalization(const GridMap& map,
                                                const std::vector<TrajectorySample>& trajectory,
                                                const std::vector<LidarScan>& scans,
                                                const LidarConfig& lidar,
                                                const EkfConfig& config) {
    std::vector<LocalizationSample> samples;
    if (!config.enabled || trajectory.empty()) {
        return samples;
    }
    const bool with_lidar = !scans.empty() && scans.size() == trajectory.size();
    const std::vector<double> offsets = beamOffsets(lidar);
    const double position_var = config.initial_position_sigma * config.initial_position_sigma;
    const double heading_var = config.initial_heading_sigma * config.initial_heading_sigma;
    // The start pose is known only to within the initial uncertainty: the filter
    // (and dead reckoning) start from the true pose plus an error drawn from it.
    std::mt19937 rng(config.odometry.seed);
    std::normal_distribution<double> unit(0.0, 1.0);
    const Pose truth0 = poseOf(trajectory.front().state);
    const double dx = config.initial_position_sigma * unit(rng);
    const double dy = config.initial_position_sigma * unit(rng);
    const double dtheta = config.initial_heading_sigma * unit(rng);
    const Pose start{truth0.x + dx, truth0.y + dy, wrapAngle(truth0.theta + dtheta)};
    PoseEkf ekf(start, {position_var, 0.0, 0.0, 0.0, position_var, 0.0, 0.0, 0.0, heading_var});
    Pose odometry = start;

    samples.reserve(trajectory.size());
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        LocalizationSample sample;
        sample.time = trajectory[k].state.time;
        sample.truth = poseOf(trajectory[k].state);
        if (k > 0) {
            // Noisy odometry of the true motion since the previous sample. The error
            // variances follow the commanded motion (see the header).
            const OdometryIncrement truth =
                trueIncrement(trajectory[k - 1].state, trajectory[k].state);
            const double dt = trajectory[k].state.time - trajectory[k - 1].state.time;
            const OdometryIncrement commanded{trajectory[k].linear_velocity * dt,
                                              trajectory[k].angular_velocity * dt};
            const std::array<double, 2> var = odometryVariances(commanded, config.odometry);
            const double distance_noise = std::sqrt(var[0]) * unit(rng);
            const double rotation_noise = std::sqrt(var[1]) * unit(rng);
            const OdometryIncrement measured{truth.distance + distance_noise,
                                             truth.rotation + rotation_noise};
            odometry = applyIncrement(odometry, measured);
            ekf.predict(measured, var);
        }
        if (with_lidar) {
            sample.update = ekf.update(map, offsets, scans[k].ranges, scans[k].hits,
                                       lidar.max_range, config.range_sigma, config.gate);
        }
        sample.odometry = odometry;
        sample.estimate = ekf.pose();
        sample.covariance = ekf.covariance();
        sample.nees = quadratic(poseError(sample.truth, sample.estimate),
                                inverse(sample.covariance));
        samples.push_back(sample);
    }
    return samples;
}

LocalizationSummary summarizeLocalization(const std::vector<LocalizationSample>& samples) {
    LocalizationSummary summary;
    if (samples.empty()) {
        return summary;
    }
    double est_sq = 0.0, odom_sq = 0.0, est_heading_sq = 0.0, odom_heading_sq = 0.0;
    double nees = 0.0;
    for (const LocalizationSample& s : samples) {
        const std::array<double, 3> est = poseError(s.truth, s.estimate);
        const std::array<double, 3> odom = poseError(s.truth, s.odometry);
        const double est_pos = std::hypot(est[0], est[1]);
        const double odom_pos = std::hypot(odom[0], odom[1]);
        est_sq += est_pos * est_pos;
        odom_sq += odom_pos * odom_pos;
        est_heading_sq += est[2] * est[2];
        odom_heading_sq += odom[2] * odom[2];
        summary.estimate.position_max = std::max(summary.estimate.position_max, est_pos);
        summary.odometry.position_max = std::max(summary.odometry.position_max, odom_pos);
        summary.estimate.position_final = est_pos;
        summary.odometry.position_final = odom_pos;
        summary.beams_used += s.update.used;
        summary.beams_rejected += s.update.rejected;
        nees += s.nees;
    }
    const double n = static_cast<double>(samples.size());
    summary.estimate.position_rms = std::sqrt(est_sq / n);
    summary.odometry.position_rms = std::sqrt(odom_sq / n);
    summary.estimate.heading_rms = std::sqrt(est_heading_sq / n);
    summary.odometry.heading_rms = std::sqrt(odom_heading_sq / n);
    summary.mean_nees = nees / n;
    return summary;
}

}  // namespace amr
