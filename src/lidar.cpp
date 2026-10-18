#include "lidar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInfinity = std::numeric_limits<double>::infinity();
/// Boundary crossings closer than this (relative) count as simultaneous: the ray
/// passes through a cell corner.
constexpr double kCornerTolerance = 1e-12;

bool blocked(const GridMap& map, int x, int y) {
    return map.isOccupied({x, y});  // outside the map counts as occupied
}

}  // namespace

std::string validateLidarConfig(const LidarConfig& config) {
    if (config.beams < 0 || config.beams > kMaxLidarBeams) {
        return "number of LiDAR beams must be between 0 and 3600";
    }
    if (!config.enabled()) {
        return {};
    }
    if (!std::isfinite(config.fov) || config.fov <= 0.0 || config.fov > 2.0 * kPi + 1e-12) {
        return "LiDAR field of view must be in (0, 360] degrees";
    }
    if (!std::isfinite(config.max_range) || config.max_range <= 0.0 ||
        config.max_range > kMaxLidarRange) {
        return "LiDAR range must be in (0, 1000] m";
    }
    if (!std::isfinite(config.noise_stddev) || config.noise_stddev < 0.0) {
        return "LiDAR noise must be a non-negative finite number";
    }
    if (!std::isfinite(config.bias_stddev) || config.bias_stddev < 0.0 ||
        config.bias_stddev > config.max_range) {
        return "LiDAR bias must be between 0 and the LiDAR range";
    }
    if (!std::isfinite(config.bias_time) || config.bias_time < 0.0) {
        return "LiDAR bias correlation time must be a non-negative number of seconds";
    }
    return {};
}

RayHit castRay(const GridMap& map, WorldPoint origin, double angle, double max_range) {
    const double res = map.resolution();
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    int cx = static_cast<int>(std::floor(origin.x / res));
    int cy = static_cast<int>(std::floor(origin.y / res));
    if (blocked(map, cx, cy)) {
        return {true, 0.0, RayFace::None};  // the sensor is inside an obstacle (or off the map)
    }
    // Distance along the ray to the next vertical / horizontal cell boundary, and the
    // distance between consecutive boundaries.
    const int step_x = dx > 0.0 ? 1 : -1;
    const int step_y = dy > 0.0 ? 1 : -1;
    double t_max_x = dx == 0.0 ? kInfinity
                     : ((dx > 0.0 ? cx + 1 : cx) * res - origin.x) / dx;
    double t_max_y = dy == 0.0 ? kInfinity
                     : ((dy > 0.0 ? cy + 1 : cy) * res - origin.y) / dy;
    const double t_delta_x = dx == 0.0 ? kInfinity : res / std::fabs(dx);
    const double t_delta_y = dy == 0.0 ? kInfinity : res / std::fabs(dy);

    while (true) {
        const double t = std::min(t_max_x, t_max_y);
        if (t > max_range) {
            return {false, max_range, RayFace::None};
        }
        const bool corner =
            std::fabs(t_max_x - t_max_y) <= kCornerTolerance * std::max(1.0, t);
        if (corner) {
            // Through a corner: the ray touches both side cells and the diagonal one.
            if (blocked(map, cx + step_x, cy) || blocked(map, cx, cy + step_y) ||
                blocked(map, cx + step_x, cy + step_y)) {
                return {true, t, RayFace::Corner};
            }
            cx += step_x;
            cy += step_y;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
        } else if (t_max_x < t_max_y) {
            cx += step_x;
            t_max_x += t_delta_x;
            if (blocked(map, cx, cy)) {
                return {true, t, RayFace::Vertical};
            }
        } else {
            cy += step_y;
            t_max_y += t_delta_y;
            if (blocked(map, cx, cy)) {
                return {true, t, RayFace::Horizontal};
            }
        }
    }
}

std::vector<double> beamOffsets(const LidarConfig& config) {
    std::vector<double> offsets;
    offsets.reserve(static_cast<std::size_t>(std::max(config.beams, 0)));
    const bool full_circle = config.fov >= 2.0 * kPi - 1e-12;
    for (int i = 0; i < config.beams; ++i) {
        if (config.beams == 1) {
            offsets.push_back(0.0);
        } else if (full_circle) {
            // -pi .. pi - spacing: N distinct directions, no duplicate at +-pi.
            offsets.push_back(-kPi + 2.0 * kPi * i / config.beams);
        } else {
            offsets.push_back(-config.fov / 2 + config.fov * i / (config.beams - 1));
        }
    }
    return offsets;
}

RangeErrorModel::RangeErrorModel(const LidarConfig& config)
    : config_(config), rng_(config.seed), noise_(0.0, config.noise_stddev),
      // A separate stream, so adding a bias leaves the white noise unchanged.
      bias_rng_(config.seed ^ 0x9e3779b9u) {}

void RangeErrorModel::nextScan(double time) {
    if (config_.bias_stddev <= 0.0) {
        return;
    }
    if (first_) {
        bias_ = config_.bias_stddev * unit_(bias_rng_);
    } else if (config_.bias_time > 0.0) {
        const double a = std::exp(-std::max(time - last_time_, 0.0) / config_.bias_time);
        bias_ = a * bias_ + std::sqrt(1.0 - a * a) * config_.bias_stddev * unit_(bias_rng_);
    }
    first_ = false;
    last_time_ = time;
}

double RangeErrorModel::measure(double exact) {
    double range = exact + bias_;
    if (config_.noise_stddev > 0.0) {
        range += noise_(rng_);
    }
    return range == exact ? exact : std::clamp(range, 0.0, config_.max_range);
}

std::vector<LidarScan> scanTrajectory(const GridMap& map,
                                      const std::vector<TrajectorySample>& trajectory,
                                      const LidarConfig& config) {
    return scanTrajectoryWith(
        [&map](WorldPoint origin, double angle, double max_range, double /*time*/) {
            const RayHit ray = castRay(map, origin, angle, max_range);
            return BeamRange{ray.hit, ray.range};
        },
        trajectory, config);
}

}  // namespace amr
