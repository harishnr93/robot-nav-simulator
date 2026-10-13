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
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace amr {

struct LidarConfig {
    int beams = 0;               ///< number of beams; 0 = LiDAR disabled
    double fov = 2.0 * 3.14159265358979323846;  ///< field of view, rad (2 pi = full circle)
    double max_range = 5.0;      ///< m; beams that hit nothing within it report no return
    double noise_stddev = 0.0;   ///< Gaussian range noise, m (0 = exact ranges)
    std::uint32_t seed = 0;      ///< noise generator seed

    bool enabled() const { return beams > 0; }
};

/// Upper bounds that keep scans and their CSV output reasonably sized.
constexpr int kMaxLidarBeams = 3600;
constexpr double kMaxLidarRange = 1000.0;

/// Empty string if the configuration is valid, otherwise the reason.
std::string validateLidarConfig(const LidarConfig& config);

/// Result of one ray cast.
struct RayHit {
    bool hit = false;   ///< true if an obstacle or the map border is within max_range
    double range = 0.0; ///< distance to the hit, m (max_range when there is no hit)
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

}  // namespace amr
