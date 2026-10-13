// EKF localization: noisy wheel odometry, corrected by LiDAR ranges on the known map.
//
// State: robot pose (x, y, theta), in the world frame (+y down, theta from +x toward
// +y, as everywhere in the project).
//
// Odometry. Between two trajectory samples the differential-drive robot first turns
// by dphi and then drives ds straight ahead (the turn-then-drive controller never
// does both in one step, and when it drives the residual turn is applied first), so
//
//     theta' = theta + dphi,   x' = x + ds cos(theta'),   y' = y + ds sin(theta')
//
// reproduces the true trajectory exactly from the true increments. The simulated
// wheel odometry reports the increments with zero-mean Gaussian errors whose variance
// grows with the motion, as for wheel-encoder errors that accumulate per meter
// driven and per radian turned:
//
//     var(ds)   = k_dist^2  |ds|
//     var(dphi) = k_turn^2  |dphi| + k_drift^2 |ds|
//
// (k_drift: heading drift while driving straight, e.g. from unequal wheel radii.)
// The variances are evaluated at the commanded increments (v dt, omega dt), which
// the controller knows exactly. The measured increments would not do: while the
// robot drives straight, the measured turn is pure drift noise, and treating it as
// a turn adds variance that was never there (in a Monte Carlo check this made the
// filter's heading variance 35 % too large).
// Integrating the noisy increments alone is dead reckoning; its error grows without
// bound.
//
// Filter. Prediction propagates the estimate with the measured increments and the
// covariance with P' = F P F^T + V M V^T (F, V: Jacobians of the motion model with
// respect to the pose and to the increments; M: the odometry variances above).
// Correction uses the LiDAR scan taken at the new sample: for each beam with a
// return, the expected range is cast from the estimated pose on the known map
// (lidar.hpp). Where the predicted beam ends on a cell face x = X (or y = Y), the
// range is r = (X - x) / cos(phi) (or (Y - y) / sin(phi)), phi = theta + beam
// offset, which gives the exact Jacobian
//
//     vertical face:    H = [ -1 / cos(phi),  0,              r tan(phi) ]
//     horizontal face:  H = [  0,            -1 / sin(phi),  -r / tan(phi) ]
//
// Beams are skipped where this linearization is poor: predicted corner hits, grazing
// beams (incidence angle above ~78 deg) and beams without a predicted or measured
// return. A beam whose innovation exceeds `gate` standard deviations is rejected as
// an outlier (typically the true beam hit a different face than predicted). The
// accepted beams are fused in one batch update in information form,
//
//     P+ = (P^-1 + sum H_i^T H_i / R)^-1,   x+ = x + P+ sum H_i^T nu_i / R,
//
// which equals the standard Kalman update of the stacked measurements but only
// inverts 3 x 3 matrices.
//
// The filter and dead reckoning start from the true start pose plus an error drawn
// from the initial uncertainty (initial_*_sigma), so the stated covariance is honest
// from the first sample on.
//
// The estimate does not feed back into the motion: the robot is still controlled
// with its true pose, and the filter runs alongside it, like the LiDAR.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "diff_drive.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "simulator.hpp"

namespace amr {

/// 3 x 3 matrix, row-major.
using Matrix3 = std::array<double, 9>;

/// Odometry error model (see the file comment).
struct OdometryNoise {
    double distance = 0.05;  ///< k_dist: distance error std after 1 m, m / sqrt(m)
    double turn = 0.05;      ///< k_turn: heading error std after 1 rad turned, rad / sqrt(rad)
    double drift = 0.02;     ///< k_drift: heading error std after 1 m driven, rad / sqrt(m)
    std::uint32_t seed = 0;  ///< noise generator seed
};

struct EkfConfig {
    bool enabled = false;
    OdometryNoise odometry;
    double range_sigma = 0.05;           ///< LiDAR range std assumed by the filter, m
    double gate = 3.0;                   ///< innovation gate, standard deviations
    double initial_position_sigma = 0.01;  ///< std of the initial x and y error, m
    double initial_heading_sigma = 0.01;   ///< std of the initial heading error, rad
};

/// Upper bounds for the noise parameters (keep them physically meaningful).
constexpr double kMaxOdometryNoise = 10.0;
constexpr double kMaxRangeSigma = 100.0;

/// Empty string if the configuration is valid, otherwise the reason.
std::string validateEkfConfig(const EkfConfig& config);

/// Motion between two samples: turn by `rotation`, then drive `distance` straight.
struct OdometryIncrement {
    double distance = 0.0;  ///< m
    double rotation = 0.0;  ///< rad
};

/// The true increment between two consecutive trajectory states.
OdometryIncrement trueIncrement(const RobotState& from, const RobotState& to);

/// Pose after applying an increment (heading wrapped to (-pi, pi]).
Pose applyIncrement(const Pose& pose, const OdometryIncrement& increment);

/// Odometry variances (distance, rotation) for an increment.
std::array<double, 2> odometryVariances(const OdometryIncrement& increment,
                                        const OdometryNoise& noise);

/// Result of one measurement update.
struct UpdateStats {
    std::size_t used = 0;      ///< beams fused into the estimate
    std::size_t rejected = 0;  ///< beams rejected by the innovation gate
};

/// Extended Kalman filter for the pose.
class PoseEkf {
public:
    PoseEkf(const Pose& initial, const Matrix3& covariance);

    /// Prediction with a measured odometry increment and the variances of its
    /// distance and rotation errors (odometryVariances of the commanded increment).
    void predict(const OdometryIncrement& measured, const std::array<double, 2>& variances);

    /// Correction with a LiDAR scan. `offsets` are the beam angles relative to the
    /// heading (beamOffsets(lidar)); `ranges` and `hits` the measured scan.
    UpdateStats update(const GridMap& map, const std::vector<double>& offsets,
                       const std::vector<double>& ranges, const std::vector<bool>& hits,
                       double max_range, double range_sigma, double gate);

    const Pose& pose() const { return pose_; }
    const Matrix3& covariance() const { return covariance_; }

private:
    Pose pose_;
    Matrix3 covariance_;
};

/// Range Jacobian dr/d(x, y, theta) of a beam at world angle `angle` that hits a
/// cell face of the given kind at distance `range`.
std::array<double, 3> rangeJacobian(RayFace face, double angle, double range);

/// One sample of a localization run.
struct LocalizationSample {
    double time = 0.0;
    Pose truth;
    Pose odometry;     ///< dead reckoning: noisy odometry integrated without correction
    Pose estimate;     ///< EKF estimate
    Matrix3 covariance{};
    UpdateStats update;
    double nees = 0.0; ///< normalized estimation error squared, e^T P^-1 e
};

/// Run the filter along a trajectory. `scans` must be empty (prediction only) or
/// hold one scan per trajectory sample, taken with `lidar`.
std::vector<LocalizationSample> runLocalization(const GridMap& map,
                                                const std::vector<TrajectorySample>& trajectory,
                                                const std::vector<LidarScan>& scans,
                                                const LidarConfig& lidar,
                                                const EkfConfig& config);

/// Error statistics of one pose source against the truth.
struct PoseErrorStats {
    double position_rms = 0.0;  ///< m
    double position_max = 0.0;  ///< m
    double position_final = 0.0;///< m
    double heading_rms = 0.0;   ///< rad
};

struct LocalizationSummary {
    PoseErrorStats estimate;
    PoseErrorStats odometry;
    std::size_t beams_used = 0;
    std::size_t beams_rejected = 0;
    double mean_nees = 0.0;
};

LocalizationSummary summarizeLocalization(const std::vector<LocalizationSample>& samples);

}  // namespace amr
