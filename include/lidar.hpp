// Simulated 2D LiDAR: exact ray casting against the occupancy grid.
//
// Each beam starts at the sensor position and travels in a straight line until it
// reaches the first occupied cell or the map border (the area outside the map
// counts as a wall, as everywhere in this project). The distance is computed
// exactly with a grid traversal (Amanatides & Woo, "A Fast Voxel Traversal
// Algorithm", 1987): the ray is followed from one cell boundary to the next, so no
// obstacle can be skipped, however thin the ray's path through it. A beam that
// passes exactly through the corner of an occupied cell registers a hit there
// (touching counts, as in the corner check of the simulator).
//
// Beam angles are relative to the robot's heading (the world frame for the point
// robot, which has no heading). Optional zero-mean Gaussian range noise uses a
// seeded generator, so scans are reproducible.
//
// Range errors can also be correlated: a range bias shared by every beam of a scan
// (calibration, temperature), drifting over time as a first-order Gauss-Markov
// process with stationary standard deviation bias_stddev and correlation time
// bias_time (0: one constant bias for the whole run):
//
//     b_0 ~ N(0, s^2),   b_k = a b_(k-1) + sqrt(1 - a^2) s w_k,   a = exp(-dt / tau)
//
// Every beam of scan k with a return reads range + b_k + white noise. Correlated
// errors are what an EKF that treats beams as independent gets wrong (ekf.hpp).
#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "lidar_config.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace amr {

/// Which kind of cell boundary a ray hit (used for the range Jacobian of the EKF).
enum class RayFace {
    None,        ///< no hit, or the sensor is inside an obstacle
    Vertical,    ///< a boundary x = const (the ray entered a cell left or right)
    Horizontal,  ///< a boundary y = const (the ray entered a cell above or below)
    Corner,      ///< a cell corner (both at once)
};

/// Result of one ray cast.
struct RayHit {
    bool hit = false;   ///< true if an obstacle or the map border is within max_range
    double range = 0.0; ///< distance to the hit, m (max_range when there is no hit)
    RayFace face = RayFace::None;
};

/// Cast one ray from origin in direction `angle` (rad, world frame).
RayHit castRay(const GridMap& map, WorldPoint origin, double angle, double max_range);

/// Beam angles relative to the robot heading: evenly spaced over the field of
/// view, centred on the heading. A full circle has no duplicate beam at +-pi.
std::vector<double> beamOffsets(const LidarConfig& config);

/// One LiDAR scan.
struct LidarScan {
    double time = 0.0;
    WorldPoint origin;
    double heading = 0.0;            ///< rad
    std::vector<double> angles;      ///< world-frame beam angles, rad
    std::vector<double> ranges;      ///< measured ranges (with noise), m
    std::vector<bool> hits;          ///< false: no return within max_range
};

/// A scan at every trajectory sample, from the sample's position and heading.
std::vector<LidarScan> scanTrajectory(const GridMap& map,
                                      const std::vector<TrajectorySample>& trajectory,
                                      const LidarConfig& config);

/// Exact range of one beam in some world: hit, range (max_range when nothing is hit).
struct BeamRange {
    bool hit = false;
    double range = 0.0;
};

/// The same for any world: `cast(origin, angle, max_range, time)` gives the exact
/// range of one beam (e.g. a continuous world, or the map plus moving obstacles).
/// Noise and bias are applied exactly as in scanTrajectory (same generator order),
/// so scanTrajectory(map, ...) equals this with castRay on the map.
template <typename Caster>
std::vector<LidarScan> scanTrajectoryWith(const Caster& cast,
                                          const std::vector<TrajectorySample>& trajectory,
                                          const LidarConfig& config);

/// Range errors of one scan's beams (white noise and the scan's bias); internal
/// helper for scanTrajectoryWith, exposed so the template can stay in the header.
class RangeErrorModel {
public:
    explicit RangeErrorModel(const LidarConfig& config);
    /// Advance the bias to the scan at `time` (call once per scan, in time order).
    void nextScan(double time);
    /// Measured range for an exact range with a return.
    double measure(double exact);
    double bias() const { return bias_; }

private:
    LidarConfig config_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_;
    std::mt19937 bias_rng_;
    std::normal_distribution<double> unit_{0.0, 1.0};
    double bias_ = 0.0;
    double last_time_ = 0.0;
    bool first_ = true;
};

template <typename Caster>
std::vector<LidarScan> scanTrajectoryWith(const Caster& cast,
                                          const std::vector<TrajectorySample>& trajectory,
                                          const LidarConfig& config) {
    std::vector<LidarScan> scans;
    if (!config.enabled()) {
        return scans;
    }
    const std::vector<double> offsets = beamOffsets(config);
    RangeErrorModel errors(config);
    scans.reserve(trajectory.size());
    for (const TrajectorySample& sample : trajectory) {
        LidarScan scan;
        scan.time = sample.state.time;
        scan.origin = {sample.state.x, sample.state.y};
        scan.heading = sample.state.theta;
        errors.nextScan(scan.time);
        for (const double offset : offsets) {
            const double angle = scan.heading + offset;
            const BeamRange ray = cast(scan.origin, angle, config.max_range, scan.time);
            scan.angles.push_back(angle);
            scan.ranges.push_back(ray.hit ? errors.measure(ray.range) : ray.range);
            scan.hits.push_back(ray.hit);
        }
        scans.push_back(std::move(scan));
    }
    return scans;
}

}  // namespace amr
