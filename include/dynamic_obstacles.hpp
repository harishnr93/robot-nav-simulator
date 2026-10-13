// Dynamic obstacles: discs moving back and forth along fixed routes.
//
// Each obstacle is a disc of radius `radius` whose center moves at constant `speed`
// along a polyline route through cell centers, reverses at the last point, returns
// to the first, and repeats ("ping-pong"). Its position is a closed-form function
// of time, so the simulator can evaluate it at any instant.
//
// Collision with the robot is exact: within a timestep the robot center moves in a
// straight line (as recorded in the trajectory and drawn by the animation) and the
// obstacle center is piecewise linear (it may reverse mid-step). Splitting the step
// at the obstacle's reversal and corner times leaves intervals where both move
// linearly; the closest approach of two linear motions has a closed form.
//
// The robot does not know the routes. It only sees obstacles within its sensing
// range and in line of sight (the segment from the robot to the obstacle center
// does not cross a static obstacle), at their current positions. From two
// consecutive sightings it estimates an obstacle's velocity (simulator.hpp).
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

struct DynamicObstacle {
    std::vector<WorldPoint> route;  ///< at least two points, world frame, m
    double speed = 0.5;             ///< m/s along the route
    double radius = 0.25;           ///< m
};

/// Route length (one way), m.
double routeLength(const DynamicObstacle& obstacle);

/// Center position at time t >= 0.
WorldPoint obstaclePosition(const DynamicObstacle& obstacle, double t);

/// Times in (t0, t1) at which the obstacle passes a route point (corners and
/// reversals), in increasing order. Between them its motion is linear.
std::vector<double> obstacleBreakpoints(const DynamicObstacle& obstacle, double t0, double t1);

/// Closest approach of p(s) = p0 + (p1 - p0) s and q(s) = q0 + (q1 - q0) s for s in
/// [0, 1]: two points moving linearly over the same time interval.
double closestApproach(WorldPoint p0, WorldPoint p1, WorldPoint q0, WorldPoint q1);

/// Distance from p to the segment a-b.
double pointSegmentDistance(WorldPoint p, WorldPoint a, WorldPoint b);

/// Minimum distance between the robot center, moving linearly from a (at t0) to
/// b (at t1), and the obstacle center over [t0, t1]. Exact.
double minDistanceDuring(const DynamicObstacle& obstacle, WorldPoint a, WorldPoint b,
                         double t0, double t1);

/// Empty string if the obstacle is well-formed and its route stays in free cells.
std::string validateDynamicObstacle(const GridMap& map, const DynamicObstacle& obstacle);

/// Result of loading an obstacle file.
struct DynamicObstacleLoadResult {
    bool ok = false;
    std::vector<DynamicObstacle> obstacles;
    std::string error;
};

/// Load obstacles from a CSV file, one obstacle per line:
///
///     radius_m, speed_mps, x0, y0, x1, y1 [, x2, y2 ...]
///
/// x, y are grid cells (the route passes through their centers). Blank lines and
/// lines starting with '#' are ignored. Every obstacle is validated on the map.
DynamicObstacleLoadResult loadDynamicObstacles(const std::string& path, const GridMap& map);

/// True if the obstacle at `center` is within `range` of the robot (measured to
/// its edge) and the segment robot -> center crosses no static obstacle.
bool isObstacleVisible(const GridMap& map, WorldPoint robot, WorldPoint center, double radius,
                       double range);

}  // namespace amr
