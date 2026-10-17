// Surface models for LiDAR range prediction: where a beam meets the mapped obstacles,
// and how that range changes with the robot's pose (the EKF's measurement Jacobian).
//
// GridSurface: the occupancy grid itself (cells are exact squares). The range is
// exact for a world that really is the grid. For a real wall that was rasterized,
// it is not: a slanted wall becomes a staircase, and every step has its own
// axis-aligned normal. The range derivatives then jump at every cell edge (dr/dy is
// 0 on one step and -1/sin(phi) on the next), and a small pose change makes the
// beam hit another step, where the range itself jumps.
//
// LineSurface: sub-cell geometry recovered from the grid. The boundary of every
// obstacle (including the map border) is traced as a closed chain of cell faces,
// and the chain's face midpoints are split into straight runs (split-and-merge,
// tolerance 0.6 cells) and refitted by total least squares. Neighbouring lines are
// intersected to give the corners. A rasterized straight wall's face midpoints
// scatter around the true wall by up to half a cell, so the fitted line follows the
// true wall, slant included, far better than any single face. Each fitted segment
// has one normal, so along a wall the range and its derivatives are smooth; they
// change only at the wall's real corners. Each segment is a "feature" the EKF can
// attach map-error states to (one offset and tilt per wall, ekf.hpp).
//
// For either surface, a beam from o in direction d(phi) = (cos phi, sin phi) hits at
// p = o + r d with surface normal n there. Moving o by do or rotating the beam
// changes r so that p stays on the surface (to first order):
//
//     dr/dx = -n_x / (n . d),   dr/dy = -n_y / (n . d),   dr/dtheta = -r (n . d_perp) / (n . d)
//
// with d_perp = (-sin phi, cos phi). For a vertical face this is exactly
// [-1/cos phi, 0, r tan phi]. The second derivatives (for a locally flat surface)
// are used to account for the linearization error of grazing beams (ekf.hpp):
//
//     d2r/dx dtheta = n_x E / D^2,   d2r/dy dtheta = n_y E / D^2,
//     d2r/dtheta^2  = r (1 + 2 E^2 / D^2),   D = n . d,  E = n . d_perp
#pragma once

#include <array>
#include <memory>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// Where one beam meets a surface.
struct SurfaceHit {
    bool hit = false;       ///< a surface within max_range (the sensor is not inside one)
    double range = 0.0;     ///< m (max_range when there is no hit)
    WorldPoint point;       ///< hit point
    WorldPoint normal;      ///< unit normal at the hit, pointing into free space
    bool smooth = false;    ///< false: the normal is undefined (a grid corner)
    int feature = -1;       ///< LineSurface: index of the segment hit
    /// Distance along the surface from the hit to the end of the straight piece it
    /// belongs to (LineSurface: the segment's nearer end), m; infinite if unknown.
    double edge_distance = 1e300;
};

/// Incidence cosine |n . d| of a beam at world angle `angle` on the hit's surface:
/// 1 head-on, 0 grazing.
double incidenceCosine(const SurfaceHit& hit, double angle);

/// dr/d(x, y, theta) of the beam at world angle `angle` (see the file comment).
std::array<double, 3> surfaceJacobian(const SurfaceHit& hit, double angle);

/// Second derivatives of the range for a locally flat surface, row-major 3 x 3.
std::array<double, 9> surfaceHessian(const SurfaceHit& hit, double angle);

class SurfaceModel {
public:
    virtual ~SurfaceModel() = default;
    /// Cast one beam from `origin` at world angle `angle`.
    virtual SurfaceHit cast(WorldPoint origin, double angle, double max_range) const = 0;
};

/// The grid's cell faces (castRay, lidar.hpp).
class GridSurface : public SurfaceModel {
public:
    explicit GridSurface(const GridMap& map) : map_(map) {}
    SurfaceHit cast(WorldPoint origin, double angle, double max_range) const override;

private:
    const GridMap& map_;
};

/// A straight piece of the recovered obstacle boundary.
struct WallSegment {
    WorldPoint a;
    WorldPoint b;
    WorldPoint normal;  ///< unit, into free space
};

/// Closed boundary chains of the grid's obstacles (outside the map counts as
/// occupied), as face midpoints in order, free space on the left of the direction
/// of travel when +y points down (i.e. clockwise around obstacles on screen).
/// Diagonally touching obstacle cells belong to one chain.
std::vector<std::vector<WorldPoint>> boundaryChains(const GridMap& map);

/// Fit straight segments to the boundary chains (see the file comment).
std::vector<WallSegment> fitWallSegments(const GridMap& map);

/// The fitted wall segments as a surface.
class LineSurface : public SurfaceModel {
public:
    explicit LineSurface(const GridMap& map);
    SurfaceHit cast(WorldPoint origin, double angle, double max_range) const override;
    const std::vector<WallSegment>& segments() const { return segments_; }

private:
    const GridMap& map_;
    std::vector<WallSegment> segments_;
};

enum class SurfaceKind { Grid, Lines };

/// The surface model of the given kind for `map` (which must outlive it).
std::unique_ptr<SurfaceModel> makeSurface(const GridMap& map, SurfaceKind kind);

}  // namespace amr
