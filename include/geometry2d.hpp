// Continuous 2D geometry for convex polygon obstacles: containment, distances,
// overlap with an axis-aligned cell, clipped area and ray intersection. Used to
// build "true" worlds and rasterize them into occupancy grids (alignment.hpp).
#pragma once

#include <vector>

#include "types.hpp"

namespace amr {

/// Convex polygon; vertices in order (either orientation).
struct Polygon {
    std::vector<WorldPoint> vertices;
};

/// Axis-aligned square / rectangle [x0, x1] x [y0, y1].
struct Box {
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
};

/// Rectangle of the given size centered at `center`, rotated by `angle` (rad).
Polygon makeRectangle(WorldPoint center, double width, double height, double angle = 0.0);

/// Point rotated by `angle` about the origin, then translated.
WorldPoint transformPoint(WorldPoint p, double angle, WorldPoint translation);
Polygon transformPolygon(const Polygon& polygon, double angle, WorldPoint translation);

double distanceToSegment(WorldPoint p, WorldPoint a, WorldPoint b);

/// True if p is inside the polygon or on its boundary.
bool polygonContains(const Polygon& polygon, WorldPoint p);

/// Euclidean distance from p to the polygon (0 inside).
double distanceToPolygon(const Polygon& polygon, WorldPoint p);

/// True if the polygon and the box share an area > 0 (touching edges do not count).
bool polygonOverlapsBox(const Polygon& polygon, const Box& box);

/// Area of polygon intersect box.
double intersectionArea(const Polygon& polygon, const Box& box);

double polygonArea(const Polygon& polygon);

/// Distance along the ray origin + t (cos a, sin a), t >= 0, to where it first meets
/// the polygon: 0 if the origin is inside, infinity if it misses.
double rayToPolygon(const Polygon& polygon, WorldPoint origin, double angle);

/// Distance along the ray to the first point of the circle (center, radius):
/// 0 if the origin is inside, infinity if it misses.
double rayToCircle(WorldPoint center, double radius, WorldPoint origin, double angle);

}  // namespace amr
