#include "dynamic_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "lidar.hpp"
#include "simulator.hpp"

namespace amr {

namespace {

double distanceBetween(WorldPoint a, WorldPoint b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

/// Arc length of every route point from the first one.
std::vector<double> cumulativeLengths(const DynamicObstacle& obstacle) {
    std::vector<double> cumulative{0.0};
    for (std::size_t i = 1; i < obstacle.route.size(); ++i) {
        cumulative.push_back(cumulative.back() +
                             distanceBetween(obstacle.route[i - 1], obstacle.route[i]));
    }
    return cumulative;
}

/// Point at arc length s in [0, length] along the route.
WorldPoint pointAtArc(const DynamicObstacle& obstacle, const std::vector<double>& cumulative,
                      double s) {
    const auto it = std::upper_bound(cumulative.begin(), cumulative.end(), s);
    const std::size_t i = std::clamp<std::size_t>(
        static_cast<std::size_t>(it - cumulative.begin()), 1, cumulative.size() - 1);
    const double leg = cumulative[i] - cumulative[i - 1];
    const double f = leg > 0.0 ? std::clamp((s - cumulative[i - 1]) / leg, 0.0, 1.0) : 0.0;
    const WorldPoint& p = obstacle.route[i - 1];
    const WorldPoint& q = obstacle.route[i];
    return {p.x + (q.x - p.x) * f, p.y + (q.y - p.y) * f};
}

}  // namespace

double closestApproach(WorldPoint p0, WorldPoint p1, WorldPoint q0, WorldPoint q1) {
    const double rx = p0.x - q0.x;
    const double ry = p0.y - q0.y;
    const double vx = (p1.x - p0.x) - (q1.x - q0.x);
    const double vy = (p1.y - p0.y) - (q1.y - q0.y);
    const double vv = vx * vx + vy * vy;
    const double s = vv > 0.0 ? std::clamp(-(rx * vx + ry * vy) / vv, 0.0, 1.0) : 0.0;
    return std::hypot(rx + vx * s, ry + vy * s);
}

double pointSegmentDistance(WorldPoint p, WorldPoint a, WorldPoint b) {
    return closestApproach(p, p, a, b);
}

double routeLength(const DynamicObstacle& obstacle) {
    return cumulativeLengths(obstacle).back();
}

WorldPoint obstaclePosition(const DynamicObstacle& obstacle, double t) {
    const std::vector<double> cumulative = cumulativeLengths(obstacle);
    const double length = cumulative.back();
    if (length <= 0.0) {
        return obstacle.route.front();
    }
    // Ping-pong: arc s runs 0 -> length -> 0 with period 2 length / speed.
    double s = std::fmod(obstacle.speed * std::max(t, 0.0), 2.0 * length);
    if (s > length) {
        s = 2.0 * length - s;
    }
    return pointAtArc(obstacle, cumulative, s);
}

std::vector<double> obstacleBreakpoints(const DynamicObstacle& obstacle, double t0, double t1) {
    std::vector<double> times;
    const std::vector<double> cumulative = cumulativeLengths(obstacle);
    const double length = cumulative.back();
    if (length <= 0.0 || t1 <= t0) {
        return times;
    }
    // Route points on the unfolded arc [0, 2 length): out and back again.
    std::vector<double> arcs(cumulative.begin(), cumulative.end());
    for (std::size_t i = cumulative.size() - 1; i-- > 1;) {
        arcs.push_back(2.0 * length - cumulative[i]);
    }
    const double period = 2.0 * length / obstacle.speed;
    const double first_cycle = std::floor(std::max(t0, 0.0) / period);
    for (double cycle = first_cycle;; cycle += 1.0) {
        for (const double arc : arcs) {
            const double t = cycle * period + arc / obstacle.speed;
            if (t >= t1) {
                return times;
            }
            if (t > t0) {
                times.push_back(t);
            }
        }
    }
}

double minDistanceDuring(const DynamicObstacle& obstacle, WorldPoint a, WorldPoint b,
                         double t0, double t1) {
    if (t1 <= t0) {
        return distanceBetween(a, obstaclePosition(obstacle, t0));
    }
    std::vector<double> times{t0};
    const std::vector<double> breaks = obstacleBreakpoints(obstacle, t0, t1);
    times.insert(times.end(), breaks.begin(), breaks.end());
    times.push_back(t1);
    const auto robotAt = [&](double t) {
        const double f = (t - t0) / (t1 - t0);
        return WorldPoint{a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f};
    };
    double best = distanceBetween(a, obstaclePosition(obstacle, t0));
    for (std::size_t i = 1; i < times.size(); ++i) {
        best = std::min(best, closestApproach(robotAt(times[i - 1]), robotAt(times[i]),
                                              obstaclePosition(obstacle, times[i - 1]),
                                              obstaclePosition(obstacle, times[i])));
    }
    return best;
}

std::string validateDynamicObstacle(const GridMap& map, const DynamicObstacle& obstacle) {
    if (obstacle.route.size() < 2) {
        return "a route needs at least two points";
    }
    if (!std::isfinite(obstacle.speed) || obstacle.speed <= 0.0 || obstacle.speed > 100.0) {
        return "speed must be in (0, 100] m/s";
    }
    if (!std::isfinite(obstacle.radius) || obstacle.radius <= 0.0 ||
        obstacle.radius > kMaxRobotRadius) {
        return "radius must be in (0, 100] m";
    }
    for (std::size_t i = 0; i < obstacle.route.size(); ++i) {
        if (!map.isFree(map.worldToGrid(obstacle.route[i]))) {
            return "route point " + std::to_string(i) + " is not a free cell of the map";
        }
        if (i > 0 && !isSegmentCollisionFree(map, obstacle.route[i - 1], obstacle.route[i])) {
            return "route segment " + std::to_string(i - 1) + "-" + std::to_string(i) +
                   " crosses a static obstacle";
        }
    }
    return {};
}

DynamicObstacleLoadResult loadDynamicObstacles(const std::string& path, const GridMap& map) {
    DynamicObstacleLoadResult result;
    std::ifstream in(path);
    if (!in) {
        result.error = "cannot open dynamic obstacle file '" + path + "'";
        return result;
    }
    std::string line;
    int line_number = 0;
    while (std::getline(in, line)) {
        ++line_number;
        const std::string where = "'" + path + "' line " + std::to_string(line_number) + ": ";
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        std::vector<double> values;
        std::stringstream fields(line);
        std::string field;
        while (std::getline(fields, field, ',')) {
            char* end = nullptr;
            const double value = std::strtod(field.c_str(), &end);
            const bool blank_tail = end != field.c_str() &&
                                    std::string(end).find_first_not_of(" \t\r") == std::string::npos;
            if (!blank_tail || !std::isfinite(value)) {
                result.error = where + "'" + field + "' is not a number";
                return result;
            }
            values.push_back(value);
        }
        if (values.size() < 6 || values.size() % 2 != 0) {
            result.error = where + "expected radius, speed and at least two x, y cell pairs";
            return result;
        }
        DynamicObstacle obstacle;
        obstacle.radius = values[0];
        obstacle.speed = values[1];
        for (std::size_t i = 2; i < values.size(); i += 2) {
            if (values[i] != std::floor(values[i]) || values[i + 1] != std::floor(values[i + 1])) {
                result.error = where + "route points must be whole cell coordinates";
                return result;
            }
            const GridCoord cell{static_cast<int>(values[i]), static_cast<int>(values[i + 1])};
            if (!map.inBounds(cell)) {
                result.error = where + "route point outside the map";
                return result;
            }
            obstacle.route.push_back(map.gridToWorld(cell));
        }
        const std::string invalid = validateDynamicObstacle(map, obstacle);
        if (!invalid.empty()) {
            result.error = where + invalid;
            return result;
        }
        result.obstacles.push_back(std::move(obstacle));
    }
    result.ok = true;
    return result;
}

bool isObstacleVisible(const GridMap& map, WorldPoint robot, WorldPoint center, double radius,
                       double range) {
    const double distance = distanceBetween(robot, center);
    if (distance - radius > range) {
        return false;
    }
    if (distance <= 0.0) {
        return true;
    }
    const double angle = std::atan2(center.y - robot.y, center.x - robot.x);
    return !castRay(map, robot, angle, distance).hit;
}

}  // namespace amr
