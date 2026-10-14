// Grid alignment: why the same doorway can come out passable or blocked depending
// on the rotation and sub-cell offset of the occupancy grid, an alignment-robust
// alternative, and telling discretization artifacts from sensor ghosts.
//
// Worlds are continuous (convex polygon obstacles). A grid is laid over them with a
// pose (rotation + offset). Everything below works in the grid frame: the grid's
// cells are axis-aligned squares [x res, (x + 1) res] x [y res, (y + 1) res], and the
// world's polygons are transformed into that frame.
//
// Rasterization rules (occupied cell):
//   AnyOverlap    the cell shares a positive area with an obstacle (conservative)
//   CenterInside  the cell center lies in an obstacle (point sampling)
//   Majority      more than half of the cell's area is covered
//   (plus maps built from simulated LiDAR scans, ghost_perception.hpp)
// The planner then inflates by the robot radius at cell centers (footprint.hpp).
// Both steps quantize: obstacles to cells, configuration space to cell centers.
//
// Alignment-robust planning (planOnClearance). The LiDAR's hit points are
// continuous measurements of obstacle surfaces, independent of the grid. Consecutive
// hits of one scan closer than 2 r are joined into segments (a disc of radius r
// cannot pass between two points closer than 2 r anyway, so joining never closes a
// passage the robot could use); where a chain ends, the surface may continue unseen
// up to the next beam, so its end point gets a margin of range x beam spacing. The
// robot plans on a fine lattice (spacing h) whose nodes keep at least r (+ margin)
// from every segment and the map border, and at least r from every cell the LiDAR
// never observed (unseen faces). A path found keeps at least r - h/2 from what was
// measured; the answer depends on alignment only through h, not the cell size.
//
// Point-level evidence (PointMap). Problem 1's ghost evidence counts beams passing
// through a *cell*. Cells on an obstacle's edge are partly free, so beams pass
// through them as well as hit them: cell-level evidence can call a real wall cell a
// ghost. Here a hit point is contradicted only by a beam that passes within
// kContradictionRadius of the point itself and continues at least
// kContradictionBeyond past it; a point with kContradictionsToInvalidate
// contradictions is dropped. The radius is tiny on purpose: beams graze past the
// corners of door jambs, and a wider radius would erode them (doors would look
// wider than they are). Random spurious returns, which beams almost never cross that
// precisely, are removed by an outlier filter instead (kMinSupport).
//
// Artifact or ghost? When the grid says NO_PATH, plan on the points first: if a path
// exists, the failure was a discretization artifact (no sensing needed). Otherwise
// the measurements themselves block the way: explain with the minimum ghost set
// (ghost_cells.hpp) on the points' cells, re-sense by aiming beams exactly at the
// suspect hits from the most different bearing, repeat (recoverWithPoints).
#pragma once

#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "geometry2d.hpp"
#include "ghost_perception.hpp"
#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// Pose of a grid over the world: world = rotate(grid, angle) + offset.
struct GridPose {
    double angle = 0.0;
    WorldPoint offset;
};

WorldPoint worldToGridFrame(const GridPose& pose, WorldPoint world);
Polygon polygonToGridFrame(const GridPose& pose, const Polygon& polygon);

/// A square grid over the world square of side `extent` centered at `center`, large
/// enough for any rotation: the square's center lands on the grid's center, shifted
/// by `subcell` (each component in [0, resolution)).
struct GridPlacement {
    GridPose pose;
    int cells = 0;  ///< width = height
};
GridPlacement placeGrid(WorldPoint center, double extent, double resolution, double angle,
                        WorldPoint subcell);

enum class RasterRule { AnyOverlap, CenterInside, Majority };
std::string toString(RasterRule rule);

/// Occupancy grid of polygons given in the grid frame.
GridMap rasterize(const std::vector<Polygon>& obstacles, int width, int height, double resolution,
                  RasterRule rule);

/// A view-dependent ghost (reflection) in free space: in-sector beams hit its disc
/// with the given probability, all others pass through.
struct GhostDisc {
    WorldPoint center;
    double radius = 0.04;
    double direction = 0.0;
    double half_width = 0.9;
    double hit_probability = 0.8;
};

/// Continuous world in the grid frame, with an exact LiDAR (rays against polygons).
class ContinuousWorld : public SensorWorld {
public:
    ContinuousWorld(std::vector<Polygon> obstacles, std::vector<GhostDisc> ghosts, int width,
                    int height, double resolution);

    BeamTrace trace(WorldPoint origin, double angle, double range, const SensorModel& sensor,
                    std::mt19937& rng) const override;
    /// Exact: every segment between consecutive cell centers keeps at least
    /// radius - kSafetyTolerance from every obstacle.
    bool pathIsSafe(const std::vector<GridCoord>& path, double radius,
                    Connectivity connectivity) const override;
    bool isReal(std::size_t index) const override;

    /// The same for a polyline in grid-frame coordinates.
    bool polylineIsSafe(const std::vector<WorldPoint>& path, double radius) const;
    /// Distance from p to the nearest obstacle (0 inside one).
    double clearance(WorldPoint p) const;

    const std::vector<Polygon>& obstacles() const { return obstacles_; }
    const std::vector<GhostDisc>& ghosts() const { return ghosts_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

private:
    std::vector<Polygon> obstacles_;
    std::vector<GhostDisc> ghosts_;
    int width_;
    int height_;
    double resolution_;
};

/// Paths that come this close to violating the radius still count as safe, m.
constexpr double kSafetyTolerance = 1e-6;

/// Distance between segment a-b and a convex polygon (0 if they meet).
double segmentToPolygon(WorldPoint a, WorldPoint b, const Polygon& polygon);

struct Segment {
    WorldPoint a;
    WorldPoint b;          ///< a == b for an isolated point
    double margin = 0.0;   ///< extra clearance needed (unsampled surface beyond a chain end), m
};

constexpr double kContradictionRadius = 0.005;  ///< a beam this close passes "through" a hit, m
constexpr double kContradictionBeyond = 0.2;    ///< ...if it continues at least this far, m
constexpr int kContradictionsToInvalidate = 2;
/// Outlier filter: a point counts as an obstacle only with at least kMinSupport
/// valid hits (itself included) within kSupportRadius. Spurious returns are
/// isolated; surfaces and reflections give clusters. (A real obstacle must return at
/// least two hits to count.)
constexpr double kSupportRadius = 0.05;
constexpr int kMinSupport = 2;

/// LiDAR hit points with point-level evidence.
struct HitPoint {
    WorldPoint position;
    float range = 0.0f;     ///< distance from the sensor, m
    float bearing = 0.0f;
    int contradictions = 0;
    int scan = 0;   ///< scan (or re-sensing burst) number
    int beam = 0;   ///< beam number within it
};

class PointMap {
public:
    PointMap(int width, int height, double resolution);
    /// Contradict the points the beam passed through, then add its hit point.
    void integrate(const BeamTrace& trace, int scan, int beam);
    bool valid(const HitPoint& p) const { return p.contradictions < kContradictionsToInvalidate; }
    /// Valid and supported by enough valid neighbours (not an isolated outlier).
    bool usable(std::size_t index) const;
    const std::vector<HitPoint>& points() const { return points_; }
    /// Indices of the usable points in a cell.
    std::vector<int> validPointsIn(std::size_t cell) const;
    /// Obstacle geometry: usable points, joined to the next beam's usable point of the
    /// same scan when closer than `link`. A point that ends such a chain (or stands
    /// alone) gets a margin of range x `angular_step`: the surface may continue
    /// unseen up to the next beam (a corner between two beams).
    std::vector<Segment> segments(double link, double angular_step) const;
    int nextScan() { return next_scan_++; }

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<HitPoint> points_;
    std::vector<std::vector<int>> by_cell_;
    int next_scan_ = 0;
};

/// A full scan from `origin` into both the occupancy belief and the point map.
int scanInto(OccupancyBelief& belief, PointMap& points, const SensorWorld& world,
             WorldPoint origin, const SensorModel& sensor, std::mt19937& rng);

struct ClearancePlan {
    bool found = false;
    std::vector<WorldPoint> path;      ///< lattice nodes, start to goal
    std::vector<WorldPoint> reached;   ///< every lattice node reached from the start
};

/// Disc of `radius` on a lattice of spacing `spacing` over the grid domain: nodes
/// are free when at least `radius` (+ the segment's margin) from every segment and
/// from the domain border, and (when `observed` is given, one flag per grid cell)
/// at least `radius` from every cell the LiDAR has never observed: points sample
/// only the surfaces in view, so unobserved space may hide an obstacle (faces no
/// viewpoint saw made paths unsafe in testing). Obstacle interiors are never
/// observed, but lie behind their observed faces, so this never blocks more than
/// the true geometry next to a face that was seen.
ClearancePlan planOnClearance(const std::vector<Segment>& segments, int width, int height,
                              double resolution, WorldPoint start, WorldPoint goal, double radius,
                              double spacing, const std::vector<bool>* observed = nullptr);

/// Cells some beam passed through or hit.
std::vector<bool> observedCells(const OccupancyBelief& belief);

/// The same against the true polygons (exact distances): the reference for how
/// far lattice sampling alone moves the answer.
ClearancePlan planOnPolygons(const ContinuousWorld& world, WorldPoint start, WorldPoint goal,
                             double radius, double spacing);

/// Ground truth for continuous passability, decided on a fine lattice with two
/// thresholds: Passable / Blocked when both agree, Borderline otherwise.
enum class Passability { Passable, Blocked, Borderline };
Passability truePassability(const ContinuousWorld& world, WorldPoint start, WorldPoint goal,
                            double radius, double spacing);

/// Result of the point-aware recovery.
struct PointRecoveryResult {
    RecoveryOutcome outcome = RecoveryOutcome::GaveUp;
    bool artifact = false;  ///< the points had a path from the start: no sensing needed
    int rounds = 0;
    long beams = 0;
    double travel_m = 0.0;
    std::string reason;  ///< why it gave up: round_limit, no_explanation_<status>, cannot_see
    std::vector<WorldPoint> path;
};

/// Suspect hits aimed at per re-sensed cell.
constexpr std::size_t kMaxAimedPoints = 8;

struct PointRecoveryConfig {
    double robot_radius = 0.3;
    double lattice_spacing = 0.025;
    int max_rounds = 10;
    int beams_per_point = 2;  ///< beams aimed at each suspect hit (two contradictions drop it)
};

/// Grid NO_PATH -> plan on the points; if blocked, explain with the minimum ghost set
/// on the points' cells, re-sense those points from the most different bearing, and
/// repeat. Start and goal are grid-frame positions.
PointRecoveryResult recoverWithPoints(OccupancyBelief belief, PointMap points,
                                      const ContinuousWorld& world, WorldPoint start,
                                      WorldPoint goal, const SensorModel& sensor,
                                      const PointRecoveryConfig& config, std::mt19937& rng);

/// A room with a dividing wall and one doorway, in world coordinates.
struct DoorwayScene {
    std::vector<Polygon> obstacles;
    WorldPoint start;
    WorldPoint goal;
    std::vector<WorldPoint> viewpoints;  ///< mapping positions (both sides of the wall)
    WorldPoint door_center;
};

/// Square room of side `room` (walls `wall` thick) split by a wall at mid-height with
/// a doorway of `door_width` centered horizontally.
DoorwayScene makeDoorwayScene(double door_width, double room = 4.0, double wall = 0.2);

}  // namespace amr
