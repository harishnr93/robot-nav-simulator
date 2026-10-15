// LiDAR configuration (lidar.hpp), in a header of its own so the simulator's
// dynamic-obstacle settings can hold one without including the LiDAR itself.
#pragma once

#include <cstdint>
#include <string>

namespace amr {

struct LidarConfig {
    int beams = 0;               ///< number of beams; 0 = LiDAR disabled
    double fov = 2.0 * 3.14159265358979323846;  ///< field of view, rad (2 pi = full circle)
    double max_range = 5.0;      ///< m; beams that hit nothing within it report no return
    double noise_stddev = 0.0;   ///< Gaussian range noise, m (0 = exact ranges)
    double bias_stddev = 0.0;    ///< range bias shared by a scan's beams, m (0 = none)
    double bias_time = 0.0;      ///< bias correlation time, s (0 = constant bias)
    std::uint32_t seed = 0;      ///< noise generator seed

    bool enabled() const { return beams > 0; }
};

/// Upper bounds that keep scans and their CSV output reasonably sized.
constexpr int kMaxLidarBeams = 3600;
constexpr double kMaxLidarRange = 1000.0;

/// Empty string if the configuration is valid, otherwise the reason.
std::string validateLidarConfig(const LidarConfig& config);

}  // namespace amr
