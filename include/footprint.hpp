// Circular robot footprint and obstacle inflation.
//
// Robot model: a disc of radius r (meters) centered on the robot position.
// The disc collides with the environment when it OVERLAPS an occupied cell or
// extends beyond the map border:
//
//     collision  <=>  distance(center, nearest occupied cell or map exterior) < r
//
// Touching is allowed (distance == r is free), so a robot whose diameter equals
// the cell size still fits through a one-cell gap. Occupied cells are the full
// squares they cover; distances are Euclidean distances to those squares.
//
// Planning with a footprint: A* runs on an inflated grid in which every cell whose
// CENTER is closer than r to an obstacle or to the map border is blocked. Moving in
// a straight line between the centers of two adjacent free cells of the inflated
// grid keeps the whole disc collision-free (see docs/algorithms.md, section 8).
//
// r == 0 is the point robot: inflation returns the map unchanged and the
// simulator keeps using the exact point-robot check.
#pragma once

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// Distances within this tolerance of the radius count as touching (not a collision).
constexpr double kFootprintTolerance = 1e-9;

/// True if a disc of the given radius centered at p overlaps an occupied cell or
/// extends outside the map. Precondition: radius > 0.
bool discOverlapsObstacle(const GridMap& map, WorldPoint p, double radius);

/// True if the disc overlaps an occupied cell or the map exterior anywhere while
/// its center moves in a straight line from a to b. Exact (no sampling): the
/// distance from a segment to a square is 0 if they intersect, and otherwise is
/// attained at a segment endpoint or a square corner. Precondition: radius > 0.
bool sweptDiscOverlapsObstacle(const GridMap& map, WorldPoint a, WorldPoint b, double radius);

/// A* on the inflated map. Statuses refer to the real map where that is more
/// informative: a start/goal that is free in `map` but blocked in `inflated` is
/// reported as StartTooCloseToObstacle / GoalTooCloseToObstacle.
PlanResult planWithFootprint(const GridMap& map, const GridMap& inflated, GridCoord start,
                             GridCoord goal);

/// Map in which every cell whose center is closer than radius to an occupied cell
/// or to the map border is occupied. radius == 0 returns an identical copy.
/// Precondition: radius >= 0 and finite.
GridMap inflateObstacles(const GridMap& map, double radius);

}  // namespace amr
