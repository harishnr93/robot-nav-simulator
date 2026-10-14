#include "alignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>

#include "diff_drive.hpp"
#include "footprint.hpp"
#include "ghost_cells.hpp"

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

Box cellBox(int x, int y, double res) {
    return {x * res, y * res, (x + 1) * res, (y + 1) * res};
}

Box boundingBox(const Polygon& polygon) {
    Box b{kInfinity, kInfinity, -kInfinity, -kInfinity};
    for (const WorldPoint& v : polygon.vertices) {
        b.x0 = std::min(b.x0, v.x);
        b.y0 = std::min(b.y0, v.y);
        b.x1 = std::max(b.x1, v.x);
        b.y1 = std::max(b.y1, v.y);
    }
    return b;
}

bool segmentsIntersect(WorldPoint p1, WorldPoint p2, WorldPoint q1, WorldPoint q2) {
    const auto orient = [](WorldPoint a, WorldPoint b, WorldPoint c) {
        const double v = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        return (v > 0.0) - (v < 0.0);
    };
    const int o1 = orient(p1, p2, q1);
    const int o2 = orient(p1, p2, q2);
    const int o3 = orient(q1, q2, p1);
    const int o4 = orient(q1, q2, p2);
    return o1 != o2 && o3 != o4;
}

double segmentToSegment(WorldPoint a, WorldPoint b, WorldPoint c, WorldPoint d) {
    if (segmentsIntersect(a, b, c, d)) {
        return 0.0;
    }
    return std::min({distanceToSegment(a, c, d), distanceToSegment(b, c, d),
                     distanceToSegment(c, a, b), distanceToSegment(d, a, b)});
}

/// Breadth-first search on the lattice (i + 1/2) h, (j + 1/2) h over the domain,
/// 4-connected, with a lazily evaluated free test.
template <typename IsFree>
ClearancePlan latticeSearch(double domain_w, double domain_h, WorldPoint start, WorldPoint goal,
                            double spacing, const IsFree& is_free) {
    ClearancePlan plan;
    const int nx = std::max(1, static_cast<int>(std::floor(domain_w / spacing)));
    const int ny = std::max(1, static_cast<int>(std::floor(domain_h / spacing)));
    const auto node = [&](WorldPoint p) {
        const int i = std::clamp(static_cast<int>(std::floor(p.x / spacing)), 0, nx - 1);
        const int j = std::clamp(static_cast<int>(std::floor(p.y / spacing)), 0, ny - 1);
        return j * nx + i;
    };
    const auto position = [&](int n) {
        return WorldPoint{(n % nx + 0.5) * spacing, (n / nx + 0.5) * spacing};
    };
    std::vector<signed char> free(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), -1);
    const auto check = [&](int n) {
        signed char& f = free[static_cast<std::size_t>(n)];
        if (f < 0) {
            f = is_free(position(n)) ? 1 : 0;
        }
        return f == 1;
    };
    const int s = node(start);
    const int g = node(goal);
    if (!check(s) || !check(g)) {
        return plan;
    }
    std::vector<int> parent(free.size(), -2);
    std::queue<int> open;
    open.push(s);
    parent[static_cast<std::size_t>(s)] = -1;
    while (!open.empty()) {
        const int n = open.front();
        open.pop();
        plan.reached.push_back(position(n));
        if (n == g) {
            for (int c = g; c >= 0; c = parent[static_cast<std::size_t>(c)]) {
                plan.path.push_back(position(c));
            }
            std::reverse(plan.path.begin(), plan.path.end());
            plan.found = true;
            return plan;
        }
        const int i = n % nx;
        const int j = n / nx;
        const int neighbours[4][2] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
        for (const auto& [a, b] : neighbours) {
            if (a < 0 || b < 0 || a >= nx || b >= ny) {
                continue;
            }
            const int m = b * nx + a;
            if (parent[static_cast<std::size_t>(m)] == -2 && check(m)) {
                parent[static_cast<std::size_t>(m)] = n;
                open.push(m);
            }
        }
    }
    return plan;
}

bool clearOfBorder(WorldPoint p, double domain_w, double domain_h, double radius) {
    return p.x >= radius && p.y >= radius && domain_w - p.x >= radius && domain_h - p.y >= radius;
}

}  // namespace

WorldPoint worldToGridFrame(const GridPose& pose, WorldPoint world) {
    return transformPoint({world.x - pose.offset.x, world.y - pose.offset.y}, -pose.angle, {0.0, 0.0});
}

Polygon polygonToGridFrame(const GridPose& pose, const Polygon& polygon) {
    Polygon out;
    for (const WorldPoint& v : polygon.vertices) {
        out.vertices.push_back(worldToGridFrame(pose, v));
    }
    return out;
}

GridPlacement placeGrid(WorldPoint center, double extent, double resolution, double angle,
                        WorldPoint subcell) {
    GridPlacement placement;
    placement.cells = static_cast<int>(std::ceil(extent * std::sqrt(2.0) / resolution)) + 4;
    const double half = placement.cells * resolution / 2.0;
    const WorldPoint anchor = transformPoint({half + subcell.x, half + subcell.y}, angle, {0.0, 0.0});
    placement.pose.angle = angle;
    placement.pose.offset = {center.x - anchor.x, center.y - anchor.y};
    return placement;
}

std::string toString(RasterRule rule) {
    switch (rule) {
        case RasterRule::AnyOverlap:   return "any_overlap";
        case RasterRule::CenterInside: return "center_inside";
        case RasterRule::Majority:     return "majority";
    }
    return "unknown";
}

GridMap rasterize(const std::vector<Polygon>& obstacles, int width, int height, double resolution,
                  RasterRule rule) {
    GridMap map(width, height, resolution);
    for (const Polygon& polygon : obstacles) {
        const Box b = boundingBox(polygon);
        const int x0 = std::max(0, static_cast<int>(std::floor(b.x0 / resolution)) - 1);
        const int x1 = std::min(width - 1, static_cast<int>(std::floor(b.x1 / resolution)) + 1);
        const int y0 = std::max(0, static_cast<int>(std::floor(b.y0 / resolution)) - 1);
        const int y1 = std::min(height - 1, static_cast<int>(std::floor(b.y1 / resolution)) + 1);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const Box cell = cellBox(x, y, resolution);
                bool occupied = false;
                switch (rule) {
                    case RasterRule::AnyOverlap:
                        occupied = polygonOverlapsBox(polygon, cell);
                        break;
                    case RasterRule::CenterInside:
                        occupied = polygonContains(polygon, map.gridToWorld({x, y}));
                        break;
                    case RasterRule::Majority:
                        // Obstacles in the scenes do not overlap, so per-polygon areas add up.
                        occupied = intersectionArea(polygon, cell) > 0.5 * resolution * resolution;
                        break;
                }
                if (occupied) {
                    map.setOccupied({x, y}, true);
                }
            }
        }
    }
    if (rule == RasterRule::Majority) {
        // Second pass for cells split between two touching obstacles.
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (map.isOccupied({x, y})) {
                    continue;
                }
                double area = 0.0;
                for (const Polygon& polygon : obstacles) {
                    area += intersectionArea(polygon, cellBox(x, y, resolution));
                }
                if (area > 0.5 * resolution * resolution) {
                    map.setOccupied({x, y}, true);
                }
            }
        }
    }
    return map;
}

ContinuousWorld::ContinuousWorld(std::vector<Polygon> obstacles, std::vector<GhostDisc> ghosts,
                                 int width, int height, double resolution)
    : obstacles_(std::move(obstacles)), ghosts_(std::move(ghosts)), width_(width),
      height_(height), resolution_(resolution) {}

BeamTrace ContinuousWorld::trace(WorldPoint origin, double angle, double range,
                                 const SensorModel& sensor, std::mt19937& rng) const {
    BeamTrace trace;
    trace.angle = angle;
    trace.origin = origin;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double spurious_at =
        unit(rng) < sensor.spurious_probability ? unit(rng) * range : kInfinity;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    // Where the beam leaves the grid domain.
    const double domain_w = width_ * resolution_;
    const double domain_h = height_ * resolution_;
    double exit = kInfinity;
    if (c > 0) exit = std::min(exit, (domain_w - origin.x) / c);
    if (c < 0) exit = std::min(exit, -origin.x / c);
    if (s > 0) exit = std::min(exit, (domain_h - origin.y) / s);
    if (s < 0) exit = std::min(exit, -origin.y / s);
    // First real surface along the beam.
    double wall = kInfinity;
    for (const Polygon& p : obstacles_) {
        wall = std::min(wall, rayToPolygon(p, origin, angle));
    }
    // Ghosts in front of it, nearest first.
    std::vector<std::pair<double, const GhostDisc*>> crossed;
    for (const GhostDisc& g : ghosts_) {
        const double t = rayToCircle(g.center, g.radius, origin, angle);
        if (t < wall && std::fabs(wrapAngle(angle - g.direction)) <= g.half_width) {
            crossed.emplace_back(t, &g);
        }
    }
    std::sort(crossed.begin(), crossed.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    double stop = std::min(range, exit);
    bool hit = false;
    for (const auto& [t, ghost] : crossed) {
        if (t >= stop) {
            break;
        }
        if (unit(rng) < ghost->hit_probability) {
            stop = t;
            hit = true;
            break;
        }
    }
    if (!hit && wall < stop) {
        stop = wall;
        hit = unit(rng) >= sensor.miss_probability;  // a miss: no return, no information past
    }
    if (spurious_at < stop) {
        stop = spurious_at;
        hit = true;
    }
    trace.free_length = stop;
    trace.hit_point = {origin.x + stop * c, origin.y + stop * s};
    // Cells passed in free space (stepping as the grid tracer does).
    const double step = resolution_ / 16.0;
    const auto cell_at = [&](double t) -> int {
        const double px = origin.x + t * c;
        const double py = origin.y + t * s;
        const int x = static_cast<int>(std::floor(px / resolution_));
        const int y = static_cast<int>(std::floor(py / resolution_));
        if (x < 0 || y < 0 || x >= width_ || y >= height_) {
            return -1;
        }
        return y * width_ + x;
    };
    // The beam's own starting cell is free space the sensor is in: observed too.
    int previous = cell_at(0.0);
    if (previous >= 0 && stop > 0.0) {
        trace.passed.push_back(previous);
    }
    for (double t = step; t < stop; t += step) {
        const int index = cell_at(t);
        if (index >= 0 && index != previous) {
            trace.passed.push_back(index);
        }
        previous = index;
    }
    if (hit) {
        trace.hit = cell_at(stop + 1e-9);
        if (trace.hit >= 0 && !trace.passed.empty() && trace.passed.back() == trace.hit) {
            trace.passed.pop_back();  // the hit cell is not a pass
        }
    }
    return trace;
}

double segmentToPolygon(WorldPoint a, WorldPoint b, const Polygon& polygon) {
    if (polygonContains(polygon, a) || polygonContains(polygon, b)) {
        return 0.0;
    }
    const auto& v = polygon.vertices;
    double d = kInfinity;
    for (std::size_t i = 0; i < v.size(); ++i) {
        d = std::min(d, segmentToSegment(a, b, v[i], v[(i + 1) % v.size()]));
    }
    return d;
}

bool ContinuousWorld::polylineIsSafe(const std::vector<WorldPoint>& path, double radius) const {
    for (std::size_t i = 0; i < path.size(); ++i) {
        const WorldPoint& a = path[i];
        const WorldPoint& b = path[std::min(i + 1, path.size() - 1)];
        for (const Polygon& p : obstacles_) {
            if (segmentToPolygon(a, b, p) < radius - kSafetyTolerance) {
                return false;
            }
        }
    }
    return true;
}

bool ContinuousWorld::pathIsSafe(const std::vector<GridCoord>& path, double radius,
                                 Connectivity /*connectivity*/) const {
    std::vector<WorldPoint> centers;
    for (const GridCoord& c : path) {
        centers.push_back({(c.x + 0.5) * resolution_, (c.y + 0.5) * resolution_});
    }
    return polylineIsSafe(centers, radius);
}

bool ContinuousWorld::isReal(std::size_t index) const {
    const int x = static_cast<int>(index % static_cast<std::size_t>(width_));
    const int y = static_cast<int>(index / static_cast<std::size_t>(width_));
    const Box cell = cellBox(x, y, resolution_);
    return std::any_of(obstacles_.begin(), obstacles_.end(),
                       [&](const Polygon& p) { return polygonOverlapsBox(p, cell); });
}

double ContinuousWorld::clearance(WorldPoint p) const {
    double d = kInfinity;
    for (const Polygon& polygon : obstacles_) {
        d = std::min(d, distanceToPolygon(polygon, p));
    }
    return d;
}

PointMap::PointMap(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution),
      by_cell_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {}

void PointMap::integrate(const BeamTrace& trace, int scan, int beam) {
    const double c = std::cos(trace.angle);
    const double s = std::sin(trace.angle);
    for (const int cell : trace.passed) {
        for (const int i : by_cell_[static_cast<std::size_t>(cell)]) {
            HitPoint& p = points_[static_cast<std::size_t>(i)];
            const double rx = p.position.x - trace.origin.x;
            const double ry = p.position.y - trace.origin.y;
            const double along = rx * c + ry * s;
            const double across = std::fabs(-rx * s + ry * c);
            if (across < kContradictionRadius && along <= trace.free_length - kContradictionBeyond) {
                ++p.contradictions;
            }
        }
    }
    if (trace.hit >= 0) {
        by_cell_[static_cast<std::size_t>(trace.hit)].push_back(static_cast<int>(points_.size()));
        points_.push_back({trace.hit_point, static_cast<float>(trace.free_length),
                           static_cast<float>(trace.angle), 0, scan, beam});
    }
}

bool PointMap::usable(std::size_t index) const {
    const HitPoint& p = points_[index];
    if (!valid(p)) {
        return false;
    }
    const int cx = static_cast<int>(std::floor(p.position.x / resolution_));
    const int cy = static_cast<int>(std::floor(p.position.y / resolution_));
    int support = 0;
    for (int y = std::max(0, cy - 1); y <= std::min(height_ - 1, cy + 1); ++y) {
        for (int x = std::max(0, cx - 1); x <= std::min(width_ - 1, cx + 1); ++x) {
            for (const int j : by_cell_[static_cast<std::size_t>(y * width_ + x)]) {
                const HitPoint& q = points_[static_cast<std::size_t>(j)];
                if (valid(q) && std::hypot(q.position.x - p.position.x, q.position.y - p.position.y) <=
                                    kSupportRadius) {
                    if (++support >= kMinSupport) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

std::vector<int> PointMap::validPointsIn(std::size_t cell) const {
    std::vector<int> out;
    for (const int i : by_cell_[cell]) {
        if (usable(static_cast<std::size_t>(i))) {
            out.push_back(i);
        }
    }
    return out;
}

std::vector<Segment> PointMap::segments(double link, double angular_step) const {
    std::vector<Segment> out;
    std::vector<bool> use(points_.size());
    for (std::size_t i = 0; i < points_.size(); ++i) {
        use[i] = usable(i);
    }
    // linked[i]: point i is joined to point i + 1 (consecutive beams of one scan).
    std::vector<bool> linked(points_.size(), false);
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const HitPoint& p = points_[i];
        const HitPoint& q = points_[i + 1];
        linked[i] = use[i] && use[i + 1] && q.scan == p.scan && q.beam == p.beam + 1 &&
                    std::hypot(q.position.x - p.position.x, q.position.y - p.position.y) < link;
    }
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const HitPoint& p = points_[i];
        if (!use[i]) {
            continue;
        }
        const bool joined_before = i > 0 && linked[i - 1];
        const bool joined_after = linked[i];
        // Inside a chain the segments follow the surface; at an end, it may go on.
        const double margin = joined_before && joined_after ? 0.0 : p.range * angular_step;
        out.push_back({p.position, p.position, margin});
        if (joined_after) {
            out.push_back({p.position, points_[i + 1].position, 0.0});
        }
    }
    return out;
}

int scanInto(OccupancyBelief& belief, PointMap& points, const SensorWorld& world,
             WorldPoint origin, const SensorModel& sensor, std::mt19937& rng) {
    const int scan = points.nextScan();
    for (int b = 0; b < sensor.beams; ++b) {
        const double angle = -kPi + 2.0 * kPi * b / sensor.beams;
        const BeamTrace trace = world.trace(origin, angle, sensor.max_range, sensor, rng);
        integrateBeam(belief, trace);
        points.integrate(trace, scan, b);
    }
    return sensor.beams;
}

std::vector<bool> observedCells(const OccupancyBelief& belief) {
    std::vector<bool> seen(belief.log_odds.size());
    for (std::size_t i = 0; i < seen.size(); ++i) {
        seen[i] = belief.hits[i] > 0 || belief.passes[i] > 0;
    }
    return seen;
}

ClearancePlan planOnClearance(const std::vector<Segment>& segments, int width, int height,
                              double resolution, WorldPoint start, WorldPoint goal, double radius,
                              double spacing, const std::vector<bool>* observed) {
    const double domain_w = width * resolution;
    const double domain_h = height * resolution;
    // Spatial hash of the segments, buckets of side `radius`.
    const double bucket = std::max(radius, resolution);
    const int bw = static_cast<int>(std::ceil(domain_w / bucket)) + 1;
    const int bh = static_cast<int>(std::ceil(domain_h / bucket)) + 1;
    std::vector<std::vector<int>> buckets(static_cast<std::size_t>(bw) * static_cast<std::size_t>(bh));
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const Segment& sg = segments[i];
        const int x0 = std::clamp(static_cast<int>(std::floor(std::min(sg.a.x, sg.b.x) / bucket)), 0, bw - 1);
        const int x1 = std::clamp(static_cast<int>(std::floor(std::max(sg.a.x, sg.b.x) / bucket)), 0, bw - 1);
        const int y0 = std::clamp(static_cast<int>(std::floor(std::min(sg.a.y, sg.b.y) / bucket)), 0, bh - 1);
        const int y1 = std::clamp(static_cast<int>(std::floor(std::max(sg.a.y, sg.b.y) / bucket)), 0, bh - 1);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                buckets[static_cast<std::size_t>(y * bw + x)].push_back(static_cast<int>(i));
            }
        }
    }
    double max_margin = 0.0;
    for (const Segment& sg : segments) {
        max_margin = std::max(max_margin, sg.margin);
    }
    const int reach = static_cast<int>(std::ceil((radius + max_margin) / bucket));
    const int cell_reach = static_cast<int>(std::ceil(radius / resolution)) + 1;
    const auto is_free = [&](WorldPoint p) {
        if (!clearOfBorder(p, domain_w, domain_h, radius)) {
            return false;
        }
        if (observed != nullptr) {
            // Keep the radius from every cell never observed.
            const int cx = std::min(width - 1, static_cast<int>(p.x / resolution));
            const int cy = std::min(height - 1, static_cast<int>(p.y / resolution));
            for (int y = std::max(0, cy - cell_reach); y <= std::min(height - 1, cy + cell_reach); ++y) {
                for (int x = std::max(0, cx - cell_reach); x <= std::min(width - 1, cx + cell_reach); ++x) {
                    if (!(*observed)[static_cast<std::size_t>(y * width + x)]) {
                        const double dx = std::max({x * resolution - p.x, 0.0, p.x - (x + 1) * resolution});
                        const double dy = std::max({y * resolution - p.y, 0.0, p.y - (y + 1) * resolution});
                        if (std::hypot(dx, dy) < radius) {
                            return false;
                        }
                    }
                }
            }
        }
        const int bx = static_cast<int>(std::floor(p.x / bucket));
        const int by = static_cast<int>(std::floor(p.y / bucket));
        for (int y = std::max(0, by - reach); y <= std::min(bh - 1, by + reach); ++y) {
            for (int x = std::max(0, bx - reach); x <= std::min(bw - 1, bx + reach); ++x) {
                for (const int i : buckets[static_cast<std::size_t>(y * bw + x)]) {
                    const Segment& sg = segments[static_cast<std::size_t>(i)];
                    if (distanceToSegment(p, sg.a, sg.b) < radius + sg.margin) {
                        return false;
                    }
                }
            }
        }
        return true;
    };
    return latticeSearch(domain_w, domain_h, start, goal, spacing, is_free);
}

ClearancePlan planOnPolygons(const ContinuousWorld& world, WorldPoint start, WorldPoint goal,
                             double radius, double spacing) {
    const double domain_w = world.width() * world.resolution();
    const double domain_h = world.height() * world.resolution();
    return latticeSearch(domain_w, domain_h, start, goal, spacing, [&](WorldPoint p) {
        return clearOfBorder(p, domain_w, domain_h, radius) && world.clearance(p) >= radius;
    });
}

Passability truePassability(const ContinuousWorld& world, WorldPoint start, WorldPoint goal,
                            double radius, double spacing) {
    // A path on the lattice with clearance >= r + h is a real path (between nodes the
    // clearance drops by at most h / 2). If no lattice path keeps r - h, there is no
    // real path either (a real one passes within h / sqrt(2) of a chain of nodes).
    if (planOnPolygons(world, start, goal, radius + spacing, spacing).found) {
        return Passability::Passable;
    }
    if (!planOnPolygons(world, start, goal, radius - spacing, spacing).found) {
        return Passability::Blocked;
    }
    return Passability::Borderline;
}

PointRecoveryResult recoverWithPoints(OccupancyBelief belief, PointMap points,
                                      const ContinuousWorld& world, WorldPoint start,
                                      WorldPoint goal, const SensorModel& sensor,
                                      const PointRecoveryConfig& config, std::mt19937& rng) {
    PointRecoveryResult result;
    const double r = config.robot_radius;
    const double res = world.resolution();
    const GridCoord start_cell{static_cast<int>(start.x / res), static_cast<int>(start.y / res)};
    const GridCoord goal_cell{static_cast<int>(goal.x / res), static_cast<int>(goal.y / res)};
    std::map<std::size_t, std::vector<double>> used;  // bearings looked from, per cell
    WorldPoint robot = start;
    for (int round = 0;; ++round) {
        const std::vector<Segment> segments = points.segments(2.0 * r, 2.0 * kPi / sensor.beams);
        const std::vector<bool> observed = observedCells(belief);
        const ClearancePlan plan = planOnClearance(segments, world.width(), world.height(), res,
                                                   start, goal, r, config.lattice_spacing, &observed);
        if (plan.found) {
            result.path = plan.path;
            result.artifact = round == 0;
            // The lattice keeps r at its nodes; between them it may lose h / 2.
            result.outcome = world.polylineIsSafe(plan.path, r - config.lattice_spacing / 2.0)
                                 ? RecoveryOutcome::Recovered
                                 : RecoveryOutcome::Unsafe;
            return result;
        }
        if (round >= config.max_rounds) {
            result.reason = "round_limit";
            return result;
        }
        ++result.rounds;
        // Explain on the cells that hold valid points, weighted by their evidence.
        GridMap occupied(world.width(), world.height(), res);
        GhostQuery query;
        query.robot_radius = r;
        query.weights.assign(occupied.cellCount(), 0.0);
        for (std::size_t i = 0; i < occupied.cellCount(); ++i) {
            const auto valid = points.validPointsIn(i);
            if (!valid.empty()) {
                occupied.setOccupied(occupied.coordFromIndex(i), true);
                query.weights[i] = ghostEvidenceWeight(static_cast<int>(valid.size()), 0);
            }
        }
        const GhostResult explanation = findGhostCells(occupied, start_cell, goal_cell, query);
        if (explanation.status != GhostStatus::Found) {
            result.reason = "no_explanation_" + toString(explanation.status);
            return result;
        }
        // Viewpoints: the lattice nodes the robot can reach, thinned out.
        std::vector<WorldPoint> candidates;
        for (std::size_t i = 0; i < plan.reached.size(); i += 8) {
            candidates.push_back(plan.reached[i]);
        }
        long fired = 0;
        for (const GridCoord& cell : explanation.solutions.front()) {
            const std::size_t index = occupied.index(cell);
            const auto valid = points.validPointsIn(index);
            if (valid.empty()) {
                continue;
            }
            WorldPoint target{0.0, 0.0};
            const double count = static_cast<double>(valid.size());
            std::vector<double>& bearings = used[index];
            for (const int i : valid) {
                target.x += points.points()[static_cast<std::size_t>(i)].position.x / count;
                target.y += points.points()[static_cast<std::size_t>(i)].position.y / count;
                bearings.push_back(points.points()[static_cast<std::size_t>(i)].bearing);
            }
            // The viewpoint in range and line of sight with the most different bearing.
            int best = -1;
            double best_score = -kInfinity;
            for (std::size_t v = 0; v < candidates.size(); ++v) {
                const WorldPoint& from = candidates[v];
                const double distance = std::hypot(target.x - from.x, target.y - from.y);
                if (distance > sensor.max_range - 0.5 || distance < 2.0 * res) {
                    continue;
                }
                // Line of sight: nothing measured in front of the target, up to one
                // cell short of it (the target's own surface does not block).
                const WorldPoint near{target.x - (target.x - from.x) * res / distance,
                                      target.y - (target.y - from.y) * res / distance};
                bool blocked = false;
                for (const Segment& sg : segments) {
                    if (segmentToSegment(from, near, sg.a, sg.b) < kContradictionRadius) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) {
                    continue;
                }
                const double bearing = std::atan2(target.y - from.y, target.x - from.x);
                double score = kPi;
                for (const double b : bearings) {
                    score = std::min(score, std::fabs(wrapAngle(bearing - b)));
                }
                score -= 0.01 * std::hypot(from.x - robot.x, from.y - robot.y);
                if (score > best_score) {
                    best_score = score;
                    best = static_cast<int>(v);
                }
            }
            if (best < 0) {
                continue;
            }
            const WorldPoint from = candidates[static_cast<std::size_t>(best)];
            result.travel_m += std::hypot(from.x - robot.x, from.y - robot.y);
            robot = from;
            bearings.push_back(std::atan2(target.y - from.y, target.x - from.x));
            // Aim exactly at the suspect hits themselves (a contradiction must pass
            // within kContradictionRadius of a hit): up to kMaxAimedPoints of them,
            // spread over the cell's list, beams_per_point each.
            const int scan = points.nextScan();
            const std::size_t stride = (valid.size() + kMaxAimedPoints - 1) / kMaxAimedPoints;
            int beam = 0;
            for (std::size_t k = 0; k < valid.size(); k += stride) {
                const WorldPoint aim = points.points()[static_cast<std::size_t>(valid[k])].position;
                const double distance = std::hypot(aim.x - from.x, aim.y - from.y);
                const double angle = std::atan2(aim.y - from.y, aim.x - from.x);
                for (int b = 0; b < config.beams_per_point; ++b) {
                    // Far enough past the hit for a pass to count as a contradiction.
                    const BeamTrace trace = world.trace(from, angle, distance + kContradictionBeyond + 0.1,
                                                        sensor, rng);
                    integrateBeam(belief, trace);
                    points.integrate(trace, scan, beam++);
                }
            }
            fired += beam;
        }
        result.beams += fired;
        if (fired == 0) {
            result.reason = "cannot_see";
            return result;
        }
    }
}

DoorwayScene makeDoorwayScene(double door_width, double room, double wall) {
    DoorwayScene scene;
    const double mid = room / 2.0;
    // Outer walls (outside the room) and the dividing wall's two halves.
    scene.obstacles.push_back(makeRectangle({mid, -wall / 2}, room + 2 * wall, wall));
    scene.obstacles.push_back(makeRectangle({mid, room + wall / 2}, room + 2 * wall, wall));
    scene.obstacles.push_back(makeRectangle({-wall / 2, mid}, wall, room));
    scene.obstacles.push_back(makeRectangle({room + wall / 2, mid}, wall, room));
    const double half = std::min(door_width, room) / 2.0;
    const double piece = mid - half;
    if (piece > 0.0) {
        scene.obstacles.push_back(makeRectangle({piece / 2.0, mid}, piece, wall));
        scene.obstacles.push_back(makeRectangle({room - piece / 2.0, mid}, piece, wall));
    }
    scene.start = {mid, room / 4.0};
    scene.goal = {mid, 3.0 * room / 4.0};
    scene.door_center = {mid, mid};
    scene.viewpoints = {scene.start, scene.goal, {room / 4, room / 4}, {3 * room / 4, room / 4},
                        {room / 4, 3 * room / 4}, {3 * room / 4, 3 * room / 4}};
    return scene;
}

}  // namespace amr
