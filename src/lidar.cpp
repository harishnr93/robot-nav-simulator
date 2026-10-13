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
    return {};
}

RayHit castRay(const GridMap& map, WorldPoint origin, double angle, double max_range) {
    const double res = map.resolution();
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    int cx = static_cast<int>(std::floor(origin.x / res));
    int cy = static_cast<int>(std::floor(origin.y / res));
    if (blocked(map, cx, cy)) {
        return {true, 0.0};  // the sensor itself is inside an obstacle (or off the map)
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
            return {false, max_range};
        }
        const bool corner =
            std::fabs(t_max_x - t_max_y) <= kCornerTolerance * std::max(1.0, t);
        if (corner) {
            // Through a corner: the ray touches both side cells and the diagonal one.
            if (blocked(map, cx + step_x, cy) || blocked(map, cx, cy + step_y) ||
                blocked(map, cx + step_x, cy + step_y)) {
                return {true, t};
            }
            cx += step_x;
            cy += step_y;
            t_max_x += t_delta_x;
            t_max_y += t_delta_y;
        } else if (t_max_x < t_max_y) {
            cx += step_x;
            t_max_x += t_delta_x;
            if (blocked(map, cx, cy)) {
                return {true, t};
            }
        } else {
            cy += step_y;
            t_max_y += t_delta_y;
            if (blocked(map, cx, cy)) {
                return {true, t};
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

std::vector<LidarScan> scanTrajectory(const GridMap& map,
                                      const std::vector<TrajectorySample>& trajectory,
                                      const LidarConfig& config) {
    std::vector<LidarScan> scans;
    if (!config.enabled()) {
        return scans;
    }
    const std::vector<double> offsets = beamOffsets(config);
    std::mt19937 rng(config.seed);
    std::normal_distribution<double> noise(0.0, config.noise_stddev);
    scans.reserve(trajectory.size());
    for (const TrajectorySample& sample : trajectory) {
        LidarScan scan;
        scan.time = sample.state.time;
        scan.origin = {sample.state.x, sample.state.y};
        scan.heading = sample.state.theta;
        for (const double offset : offsets) {
            const double angle = scan.heading + offset;
            const RayHit ray = castRay(map, scan.origin, angle, config.max_range);
            double range = ray.range;
            if (ray.hit && config.noise_stddev > 0.0) {
                range = std::clamp(range + noise(rng), 0.0, config.max_range);
            }
            scan.angles.push_back(angle);
            scan.ranges.push_back(range);
            scan.hits.push_back(ray.hit);
        }
        scans.push_back(std::move(scan));
    }
    return scans;
}

}  // namespace amr
