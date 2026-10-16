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
//
// Correlated range errors (MeasurementModelConfig). The update above treats the
// beams as independent. Errors that several beams share break that assumption: a
// range bias common to a whole scan (lidar.hpp), or a map that is only a
// discretization of the real walls (a wall 7 cm behind its cell face shortens every
// beam that hits it, scan after scan). Fusing such beams as independent counts the
// same error many times, and the filter becomes overconfident. The exact remedy
// for correlated (coloured) noise is to make the shared error part of the state:
//
//   * a range-bias state b (first-order Gauss-Markov, std bias_sigma, correlation
//     time bias_time; 0 = constant): every beam reads h(pose) + b;
//   * map-error states per straight wall: a wall the beams hit gets an offset a and
//     a tilt b, so the true surface lies e(s) = a + b (s - s0) behind the mapped one
//     at position s along the wall, and a beam reads h + e(s) / cos(alpha) (alpha:
//     incidence angle). On the line surface (surface_model.hpp) a wall is a fitted
//     segment. On the grid surface, hits belong to the same wall when their normals
//     agree within 10 deg, their distance n . p from the origin within half a cell,
//     and they lie within 1 m of the wall's extent so far. Measured on rasterized
//     worlds (perception_experiment maperror), offset + tilt per wall explain about
//     80 % of the map error's RMS: a wall rasterized half a cell off shortens every
//     beam that hits it, scan after scan, which the per-wall states capture across
//     beams and across time. What remains is modeled by independent residual
//     states per cell-sized patch (map_residual). Beyond max_map_states map-error
//     states, the least recently used walls and patches are dropped.
//
// With these states the remaining range noise is white, and the beams are fused one
// at a time (equal to the batch update, all linearized at the prior estimate).
//
// Surface (surface_model.hpp): predicted ranges on the grid's cell faces, or on the
// smooth (C1) interpolation of the occupancy grid, whose normals vary continuously
// at cell edges and follow slanted walls instead of their staircase.
//
// Grazing beams (GrazingPolicy). The cutoff policy drops beams whose incidence
// exceeds max_incidence (78.5 deg, the classic rule). The model policy keeps them
// and lets their variance say how much they are worth: the linearization error of
// a beam with range Hessian G (surface_model.hpp) under the pose covariance P is
// added to its noise, 0.5 tr(G P G P), the exact variance of the quadratic term for
// a Gaussian pose error, and the map error enters as u / cos(alpha). Both grow
// without bound as alpha -> 90 deg, so grazing beams weight themselves down
// instead of being cut at an arbitrary angle (only |cos alpha| < 0.02 is skipped,
// for numerical safety). One grazing risk is not a variance: the hit slides along
// the wall by (normal error) x tan(alpha), and past the wall's end the real beam hits
// something else. On the line surface, where wall ends are known, a beam is skipped
// when its hit is closer to the end than 2 sigma x tan(alpha) (sigma: pose error
// along the normal and the wall's offset std).
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
#include "surface_model.hpp"

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

enum class GrazingPolicy {
    Cutoff,  ///< drop beams beyond max_incidence
    Model,   ///< keep them, with their linearization and map-error variance
};

/// Measurement model of the LiDAR update (see the file comment). Defaults: grid
/// faces, independent beams, grazing beams modeled. With grazing = Cutoff and the
/// rest at its defaults it is the classic filter (PoseEkf).
struct MeasurementModelConfig {
    SurfaceKind surface = SurfaceKind::Grid;
    GrazingPolicy grazing = GrazingPolicy::Model;
    double max_incidence = 1.3694384060045659;  ///< rad, acos(0.2) = 78.46 deg (cutoff policy)
    double bias_sigma = 0.0;   ///< range-bias state std, m (0 = no bias state)
    double bias_time = 0.0;    ///< bias correlation time, s (0 = constant)
    double map_sigma = 0.0;    ///< wall offset std, m (0 = no map-error states)
    double map_tilt = 0.02;    ///< wall tilt std, rad
    double map_residual = 0.0; ///< residual std per cell-sized patch, m (0 = none)
    std::size_t max_map_states = 200;

    /// True for the classic model (grid faces, cutoff at 78.46 deg, no nuisance
    /// states): the 3-state filter PoseEkf is used.
    bool classic() const;
};

struct EkfConfig {
    bool enabled = false;
    OdometryNoise odometry;
    MeasurementModelConfig model;
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

/// The true increment between two consecutive trajectory states (negative
/// distance when the robot drove backwards).
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

/// EKF with the pose plus the nuisance states of MeasurementModelConfig (range bias,
/// map-error patches). With no nuisance states it is the 3-state filter with
/// sequential updates (the same result as PoseEkf up to rounding, for the grid
/// surface and the cutoff policy).
class AugmentedEkf {
public:
    AugmentedEkf(const Pose& initial, const Matrix3& covariance, const SurfaceModel& surface,
                 const MeasurementModelConfig& model, double map_resolution);

    /// Prediction (as PoseEkf::predict); `dt` advances the bias process.
    void predict(const OdometryIncrement& measured, const std::array<double, 2>& variances,
                 double dt);

    /// Correction with a LiDAR scan (arguments as PoseEkf::update).
    UpdateStats update(const std::vector<double>& offsets, const std::vector<double>& ranges,
                       const std::vector<bool>& hits, double max_range, double range_sigma,
                       double gate);

    Pose pose() const;
    Matrix3 covariance() const;  ///< pose block
    double bias() const;         ///< estimated range bias (0 without a bias state)
    double biasVariance() const;
    std::size_t mapStates() const { return n_ - map_begin_; }
    std::size_t walls() const { return walls_.size(); }
    std::size_t stateSize() const { return n_; }

private:
    struct Wall {
        int feature;            ///< LineSurface segment, or -1 (matched by geometry)
        WorldPoint normal;      ///< unit normal into free space (of the first hit)
        double rho;             ///< normal . p of the first hit
        double s0;              ///< along-wall coordinate of the first hit
        double s_min, s_max;    ///< extent seen so far
        std::size_t state;      ///< offset state; tilt is state + 1
        std::size_t last_used;
    };
    struct Patch {
        long long key;
        std::size_t state;
        std::size_t last_used;
    };
    /// One state index and its coefficient in a beam's H.
    struct Term {
        std::size_t index;
        double h;
    };
    double& at(std::size_t i, std::size_t j) { return p_[i * n_ + j]; }
    double at(std::size_t i, std::size_t j) const { return p_[i * n_ + j]; }
    /// Append an independent zero-mean state with the given variance; its index.
    std::size_t addState(double variance);
    /// The map-error terms of a beam hitting `point` (normal, incidence cosine),
    /// adding wall and patch states as needed.
    void mapErrorTerms(const SurfaceHit& hit, double cosine, std::vector<Term>& terms);
    void evictMapStates();

    const SurfaceModel& surface_;
    MeasurementModelConfig model_;
    double resolution_;
    std::size_t n_;
    std::vector<double> x_;   ///< x, y, theta, [bias], map errors
    std::vector<double> p_;   ///< n x n covariance, row-major
    bool has_bias_;
    std::size_t map_begin_;   ///< index of the first map-error state
    std::vector<Wall> walls_;
    std::vector<Patch> patches_;
    std::size_t scan_count_ = 0;
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
    double bias = 0.0;           ///< estimated range bias (bias state only), m
    std::size_t map_states = 0;  ///< map-error states held (map-error model only)
};

/// Run the filter along a trajectory. `scans` must be empty (prediction only) or
/// hold one scan per trajectory sample, taken with `lidar`. `kidnapped[k]` true:
/// between samples k - 1 and k the robot was carried away (the odometry reports no
/// motion, the noise sequence is unchanged).
std::vector<LocalizationSample> runLocalization(const GridMap& map,
                                                const std::vector<TrajectorySample>& trajectory,
                                                const std::vector<LidarScan>& scans,
                                                const LidarConfig& lidar,
                                                const EkfConfig& config,
                                                const std::vector<bool>& kidnapped = {});

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
