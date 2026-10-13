#include "footprint.hpp"

#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace amr {

namespace {

/// Euclidean distance from p to the axis-aligned square [x0, x1] x [y0, y1]
/// (zero if p lies inside or on the square).
double distanceToSquare(WorldPoint p, double x0, double y0, double x1, double y1) {
    const double dx = std::max({x0 - p.x, 0.0, p.x - x1});
    const double dy = std::max({y0 - p.y, 0.0, p.y - y1});
    return std::hypot(dx, dy);
}

double pointSegmentDistance(WorldPoint p, WorldPoint a, WorldPoint b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (length_sq > 0.0) {
        t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq, 0.0, 1.0);
    }
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

/// Slab test: does segment a->b touch the closed square [x0, x1] x [y0, y1]?
bool segmentIntersectsSquare(WorldPoint a, WorldPoint b, double x0, double y0, double x1,
                             double y1) {
    double t_min = 0.0;
    double t_max = 1.0;
    const double origin[2] = {a.x, a.y};
    const double direction[2] = {b.x - a.x, b.y - a.y};
    const double low[2] = {x0, y0};
    const double high[2] = {x1, y1};
    for (int axis = 0; axis < 2; ++axis) {
        if (direction[axis] == 0.0) {
            if (origin[axis] < low[axis] || origin[axis] > high[axis]) {
                return false;
            }
            continue;
        }
        double t1 = (low[axis] - origin[axis]) / direction[axis];
        double t2 = (high[axis] - origin[axis]) / direction[axis];
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        if (t_min > t_max) {
            return false;
        }
    }
    return true;
}

/// Exact distance between segment a->b and the square [x0, x1] x [y0, y1].
double segmentSquareDistance(WorldPoint a, WorldPoint b, double x0, double y0, double x1,
                             double y1) {
    if (segmentIntersectsSquare(a, b, x0, y0, x1, y1)) {
        return 0.0;
    }
    // Two disjoint convex shapes: the closest pair involves a vertex of one of them.
    double d = std::min(distanceToSquare(a, x0, y0, x1, y1), distanceToSquare(b, x0, y0, x1, y1));
    const WorldPoint corners[4] = {{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}};
    for (const WorldPoint& corner : corners) {
        d = std::min(d, pointSegmentDistance(corner, a, b));
    }
    return d;
}

/// True if some point of segment a->b is within `limit` of an occupied cell or of
/// the map exterior: distance < limit, or <= limit when `inclusive`. `radius` bounds
/// the cells that need to be examined (cells farther away cannot be within limit).
bool segmentWithin(const GridMap& map, WorldPoint a, WorldPoint b, double radius,
                   double limit, bool inclusive) {
    const double res = map.resolution();
    const auto closer = [limit, inclusive](double d) { return inclusive ? d <= limit : d < limit; };

    // Map border: the region outside the map counts as an obstacle. The distance to
    // a border is linear along the segment, so checking both endpoints suffices.
    const double map_w = map.width() * res;
    const double map_h = map.height() * res;
    for (const WorldPoint& p : {a, b}) {
        if (closer(p.x) || closer(p.y) || closer(map_w - p.x) || closer(map_h - p.y)) {
            return true;
        }
    }

    // Only cells within the swept disc's bounding box can be closer than the radius.
    const auto cell_floor = [res](double v) { return static_cast<int>(std::floor(v / res)); };
    const int x_min = std::max(0, cell_floor(std::min(a.x, b.x) - radius));
    const int x_max = std::min(map.width() - 1, cell_floor(std::max(a.x, b.x) + radius));
    const int y_min = std::max(0, cell_floor(std::min(a.y, b.y) - radius));
    const int y_max = std::min(map.height() - 1, cell_floor(std::max(a.y, b.y) + radius));
    for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
            if (map.isOccupied({x, y}) &&
                closer(segmentSquareDistance(a, b, x * res, y * res, (x + 1) * res,
                                             (y + 1) * res))) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace

bool sweptDiscOverlapsObstacle(const GridMap& map, WorldPoint a, WorldPoint b, double radius) {
    return segmentWithin(map, a, b, radius, radius - kFootprintTolerance, false);
}

bool segmentTouchesObstacle(const GridMap& map, WorldPoint a, WorldPoint b) {
    return segmentWithin(map, a, b, kFootprintTolerance, kFootprintTolerance, true);
}

bool discOverlapsObstacle(const GridMap& map, WorldPoint p, double radius) {
    return sweptDiscOverlapsObstacle(map, p, p, radius);
}

PlanResult planWithFootprint(const GridMap& map, const GridMap& inflated, GridCoord start,
                             GridCoord goal, Connectivity connectivity) {
    PlanResult result = planAStar(inflated, start, goal, connectivity);
    if (result.status == PlanStatus::StartOccupied && map.isFree(start)) {
        result.status = PlanStatus::StartTooCloseToObstacle;
    } else if (result.status == PlanStatus::GoalOccupied && map.isFree(goal)) {
        result.status = PlanStatus::GoalTooCloseToObstacle;
    }
    return result;
}

GridMap inflateObstacles(const GridMap& map, double radius) {
    if (radius <= 0.0) {
        return map;
    }
    std::vector<std::uint8_t> cells(map.data(), map.data() + map.cellCount());
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            const GridCoord c{x, y};
            if (map.isFree(c) && discOverlapsObstacle(map, map.gridToWorld(c), radius)) {
                cells[map.index(c)] = GridMap::kOccupied;
            }
        }
    }
    return GridMap(map.width(), map.height(), map.resolution(), std::move(cells));
}

}  // namespace amr
