#include "surface_model.hpp"

#include <algorithm>
#include <cmath>

#include "lidar.hpp"

namespace amr {

namespace {

double cross(WorldPoint a, WorldPoint b) {
    return a.x * b.y - a.y * b.x;
}

/// Total-least-squares line through points: a point on it and its unit direction,
/// oriented from the first point towards the last.
void fitLine(const std::vector<WorldPoint>& points, WorldPoint& center, WorldPoint& direction) {
    double mx = 0.0, my = 0.0;
    for (const WorldPoint& p : points) {
        mx += p.x;
        my += p.y;
    }
    const double n = static_cast<double>(points.size());
    mx /= n;
    my /= n;
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const WorldPoint& p : points) {
        sxx += (p.x - mx) * (p.x - mx);
        sxy += (p.x - mx) * (p.y - my);
        syy += (p.y - my) * (p.y - my);
    }
    const double angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);  // principal axis
    center = {mx, my};
    direction = {std::cos(angle), std::sin(angle)};
    const WorldPoint span{points.back().x - points.front().x, points.back().y - points.front().y};
    if (direction.x * span.x + direction.y * span.y < 0.0) {
        direction = {-direction.x, -direction.y};
    }
}

/// One cell face of a boundary chain: lattice vertices a -> b, free side normal.
struct Face {
    GridCoord a, b;
    GridCoord normal;
};

/// Boundary chains as faces (see boundaryChains).
std::vector<std::vector<Face>> traceFaces(const GridMap& map) {
    const int w = map.width();
    const int h = map.height();
    const auto occupied = [&](int x, int y) { return !map.inBounds({x, y}) || map.isOccupied({x, y}); };
    std::vector<Face> faces;
    for (int y = -1; y <= h; ++y) {
        for (int x = -1; x <= w; ++x) {
            if (!occupied(x, y)) {
                continue;
            }
            // Free neighbour -> face with free-side normal n, direction (-n.y, n.x).
            if (map.inBounds({x, y - 1}) && !occupied(x, y - 1)) {
                faces.push_back({{x, y}, {x + 1, y}, {0, -1}});
            }
            if (map.inBounds({x + 1, y}) && !occupied(x + 1, y)) {
                faces.push_back({{x + 1, y}, {x + 1, y + 1}, {1, 0}});
            }
            if (map.inBounds({x, y + 1}) && !occupied(x, y + 1)) {
                faces.push_back({{x + 1, y + 1}, {x, y + 1}, {0, 1}});
            }
            if (map.inBounds({x - 1, y}) && !occupied(x - 1, y)) {
                faces.push_back({{x, y + 1}, {x, y}, {-1, 0}});
            }
        }
    }
    const auto key = [&](GridCoord v) {
        return static_cast<std::size_t>((v.y + 2) * (w + 4) + (v.x + 2));
    };
    std::vector<std::vector<std::size_t>> starting(static_cast<std::size_t>((w + 4) * (h + 4)));
    for (std::size_t i = 0; i < faces.size(); ++i) {
        starting[key(faces[i].a)].push_back(i);
    }
    std::vector<bool> used(faces.size(), false);
    std::vector<std::vector<Face>> chains;
    for (std::size_t start = 0; start < faces.size(); ++start) {
        if (used[start]) {
            continue;
        }
        std::vector<Face> chain;
        std::size_t current = start;
        while (!used[current]) {
            used[current] = true;
            const Face& f = faces[current];
            chain.push_back(f);
            // At a saddle (two candidates) turn towards the free side, around the
            // diagonal obstacle cell: diagonally touching cells stay one obstacle.
            const GridCoord d_in{f.b.x - f.a.x, f.b.y - f.a.y};
            std::size_t next = current;
            int best = -1;
            for (const std::size_t candidate : starting[key(f.b)]) {
                if (used[candidate] && candidate != start) {
                    continue;
                }
                const Face& g = faces[candidate];
                const GridCoord d{g.b.x - g.a.x, g.b.y - g.a.y};
                const int rank = d == f.normal ? 3 : d == d_in ? 2 : 1;
                if (rank > best) {
                    best = rank;
                    next = candidate;
                }
            }
            current = next;
        }
        chains.push_back(std::move(chain));
    }
    return chains;
}

}  // namespace

double incidenceCosine(const SurfaceHit& hit, double angle) {
    return std::fabs(hit.normal.x * std::cos(angle) + hit.normal.y * std::sin(angle));
}

std::array<double, 3> surfaceJacobian(const SurfaceHit& hit, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double d = hit.normal.x * c + hit.normal.y * s;   // n . d
    const double e = -hit.normal.x * s + hit.normal.y * c;  // n . d_perp
    if (d == 0.0) {
        return {0.0, 0.0, 0.0};
    }
    return {-hit.normal.x / d, -hit.normal.y / d, -hit.range * e / d};
}

std::array<double, 9> surfaceHessian(const SurfaceHit& hit, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double d = hit.normal.x * c + hit.normal.y * s;
    const double e = -hit.normal.x * s + hit.normal.y * c;
    if (d == 0.0) {
        return {};
    }
    const double d2 = d * d;
    const double xt = hit.normal.x * e / d2;
    const double yt = hit.normal.y * e / d2;
    const double tt = hit.range * (1.0 + 2.0 * e * e / d2);
    return {0.0, 0.0, xt, 0.0, 0.0, yt, xt, yt, tt};
}

SurfaceHit GridSurface::cast(WorldPoint origin, double angle, double max_range) const {
    const RayHit ray = castRay(map_, origin, angle, max_range);
    SurfaceHit hit;
    hit.hit = ray.hit;
    hit.range = ray.range;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    hit.point = {origin.x + ray.range * c, origin.y + ray.range * s};
    if (!ray.hit || ray.range <= 0.0) {
        return hit;
    }
    if (ray.face == RayFace::Vertical) {
        hit.normal = {c > 0.0 ? -1.0 : 1.0, 0.0};
        hit.smooth = true;
    } else if (ray.face == RayFace::Horizontal) {
        hit.normal = {0.0, s > 0.0 ? -1.0 : 1.0};
        hit.smooth = true;
    }
    return hit;  // a corner: no normal
}

std::vector<std::vector<WorldPoint>> boundaryChains(const GridMap& map) {
    const double r = map.resolution();
    std::vector<std::vector<WorldPoint>> chains;
    for (const std::vector<Face>& faces : traceFaces(map)) {
        std::vector<WorldPoint> chain;
        for (const Face& f : faces) {
            chain.push_back({0.5 * (f.a.x + f.b.x) * r, 0.5 * (f.a.y + f.b.y) * r});
        }
        chains.push_back(std::move(chain));
    }
    return chains;
}

std::vector<WallSegment> fitWallSegments(const GridMap& map) {
    const double r = map.resolution();
    const auto world = [r](GridCoord v) { return WorldPoint{v.x * r, v.y * r}; };
    std::vector<WallSegment> segments;
    const auto add = [&segments](WorldPoint a, WorldPoint b) {
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        if (len > 1e-12) {
            segments.push_back({a, b, {(b.y - a.y) / len, -(b.x - a.x) / len}});
        }
    };
    for (const std::vector<Face>& faces : traceFaces(map)) {
        // 1. The exact outline: maximal runs of faces in one direction.
        struct Run {
            GridCoord a, b, direction;
            std::size_t first, count;  // faces[first .. first + count) (mod n)
        };
        const std::size_t n = faces.size();
        const auto dir = [&](std::size_t i) {
            const Face& f = faces[i % n];
            return GridCoord{f.b.x - f.a.x, f.b.y - f.a.y};
        };
        std::size_t origin = 0;  // a face that starts a run
        while (origin < n && dir(origin + n - 1) == dir(origin)) {
            ++origin;
        }
        std::vector<Run> runs;
        for (std::size_t i = origin; i < origin + n;) {
            std::size_t j = i + 1;
            while (j < origin + n && dir(j) == dir(i)) {
                ++j;
            }
            runs.push_back({faces[i % n].a, faces[(j - 1) % n].b, dir(i), i, j - i});
            i = j;
        }
        const std::size_t m = runs.size();
        // 2. Turn at the end of each run (+1 / -1). Inside a rasterized straight wall the
        //    turns alternate (a staircase); a real corner repeats a turn.
        std::vector<int> turn(m);
        for (std::size_t k = 0; k < m; ++k) {
            const GridCoord d0 = runs[k].direction;
            const GridCoord d1 = runs[(k + 1) % m].direction;
            turn[k] = d0.x * d1.y - d0.y * d1.x > 0 ? 1 : -1;
        }
        // Runs k and k + 1 are inside one staircase when the turn between them
        // differs from both neighbouring turns. Its end treads (next to a corner) are
        // added afterwards: there the corner repeats a turn.
        std::vector<bool> inside(m);
        for (std::size_t k = 0; k < m; ++k) {
            inside[k] = turn[k] != turn[(k + m - 1) % m] && turn[k] != turn[(k + 1) % m];
        }
        // 3. Walls: staircases (alternating runs that fit a straight line) or exact runs.
        struct Wall {
            std::size_t first, count;  // runs[first .. first + count) (mod m)
            bool staircase;
            WorldPoint start, end;     // endpoints on the wall's line
            WorldPoint center, direction;
        };
        std::vector<Wall> walls;
        std::size_t begin = 0;  // a run that starts a group
        while (begin < m && inside[(begin + m - 1) % m]) {
            ++begin;
        }
        if (begin == m) {
            begin = 0;  // cannot happen for a closed outline (it has 4 more turns one way)
        }
        begin += m;  // run indices are taken mod m: keep them clear of 0 when groups grow back
        // Groups [k, end) of runs joined by inside links; then grow each group of two
        // or more runs by one run at either end that no other group has taken.
        std::vector<std::pair<std::size_t, std::size_t>> groups;
        for (std::size_t k = begin; k < begin + m;) {
            std::size_t end = k + 1;
            while (end < begin + m && inside[(end - 1) % m]) {
                ++end;
            }
            groups.push_back({k, end});
            k = end;
        }
        for (std::size_t g = 0; g < groups.size(); ++g) {
            if (groups[g].second - groups[g].first < 2) {
                continue;
            }
            const std::size_t prev = (g + groups.size() - 1) % groups.size();
            const std::size_t next = (g + 1) % groups.size();
            if (prev != g && groups[prev].second - groups[prev].first == 1) {
                --groups[g].first;  // take the single run before
                groups[prev].second = groups[prev].first;  // now empty
            }
            if (next != g && next != prev && groups[next].second - groups[next].first == 1) {
                ++groups[g].second;
                groups[next].first = groups[next].second;
            }
        }
        const auto run_at = [&](std::size_t k) -> const Run& { return runs[k % m]; };
        const auto faces_of = [&](std::size_t k) { return run_at(k).count; };
        const auto exact = [&](std::size_t k) {
            Wall w{k, 1, false, world(run_at(k).a), world(run_at(k).b), {}, {}};
            return w;
        };
        // Read runs[lo, hi) as one staircase if it is one, else as exact runs; walls
        // are appended in order.
        const auto staircase = [&](std::size_t lo, std::size_t hi, GridCoord riser) {
            const auto is_riser = [&](std::size_t q) { return run_at(q).direction == riser; };
            std::size_t first = lo, last = hi;
            const auto longest_tread = [&]() {
                std::size_t longest = 0;
                for (std::size_t q = first + 1; q + 1 < last; ++q) {
                    longest = is_riser(q) ? longest : std::max(longest, faces_of(q));
                }
                return longest;
            };
            // Trim end treads longer than the interior's longest + 1: walls of their own.
            for (bool trimmed = true; trimmed && last - first > 3;) {
                trimmed = false;
                const std::size_t longest = longest_tread();
                if (!is_riser(first) && faces_of(first) > longest + 1) {
                    ++first;
                    trimmed = true;
                } else if (!is_riser(last - 1) && faces_of(last - 1) > longest + 1) {
                    --last;
                    trimmed = true;
                }
            }
            std::size_t shortest = 1000000, longest = 0, risers = 0;
            for (std::size_t q = first; q < last; ++q) {
                if (is_riser(q)) {
                    ++risers;
                } else if (q > first && q + 1 < last) {
                    shortest = std::min(shortest, faces_of(q));
                    longest = std::max(longest, faces_of(q));
                }
            }
            // Interior treads (if any) regular, at least one riser. A single step between
            // two treads of 2+ faces is read as a wall slightly rotated against the grid,
            // the usual case on a real map, rather than a one-cell jog.
            bool ok = last - first >= 3 && risers >= 1 && (longest == 0 || longest - shortest <= 1);
            if (ok && risers == 1) {
                for (std::size_t q = first; q < last; ++q) {
                    ok = ok && (is_riser(q) || faces_of(q) >= 2);
                }
            }
            Wall w{first, last - first, true, {}, {}, {}, {}};
            if (ok) {
                std::vector<WorldPoint> points;
                for (std::size_t q = first; q < last; ++q) {
                    const Run& run = run_at(q);
                    for (std::size_t f = 0; f < run.count; ++f) {
                        const Face& face = faces[(run.first + f) % n];
                        points.push_back({0.5 * (face.a.x + face.b.x) * r, 0.5 * (face.a.y + face.b.y) * r});
                    }
                }
                fitLine(points, w.center, w.direction);
                // A rasterized straight wall's face midpoints lie within half a cell of
                // it: a worse fit is not a straight wall after all.
                for (const WorldPoint& p : points) {
                    ok = ok && std::fabs(cross(w.direction, {p.x - w.center.x, p.y - w.center.y})) <=
                                   0.75 * r;
                }
            }
            if (!ok) {
                first = last = hi;
            }
            for (std::size_t q = lo; q < first; ++q) {
                walls.push_back(exact(q));
            }
            if (ok) {
                const auto project = [&w](WorldPoint p) {
                    const double t = (p.x - w.center.x) * w.direction.x + (p.y - w.center.y) * w.direction.y;
                    return WorldPoint{w.center.x + t * w.direction.x, w.center.y + t * w.direction.y};
                };
                w.start = project(world(run_at(first).a));
                w.end = project(world(run_at(last - 1).b));
                walls.push_back(w);
            }
            for (std::size_t q = last; q < hi; ++q) {
                walls.push_back(exact(q));
            }
        };
        for (const auto& [k, end] : groups) {
            if (end <= k) {
                continue;  // emptied: its run went to a neighbouring staircase
            }
            if (end - k < 3) {
                for (std::size_t q = k; q < end; ++q) {
                    walls.push_back(exact(q));
                }
                continue;
            }
            // Runs alternate between two directions; the riser direction is the one with
            // more one-face runs. A longer run in that direction is a jog or a corner
            // side: it separates staircases and stays exact.
            const GridCoord d0 = run_at(k).direction;
            const GridCoord d1 = run_at(k + 1).direction;
            std::size_t ones0 = 0, ones1 = 0;
            for (std::size_t q = k; q < end; ++q) {
                if (faces_of(q) == 1) {
                    (run_at(q).direction == d0 ? ones0 : ones1) += 1;
                }
            }
            const GridCoord riser = ones0 >= ones1 ? d0 : d1;
            std::size_t piece = k;
            for (std::size_t q = k; q <= end; ++q) {
                if (q == end || (run_at(q).direction == riser && faces_of(q) > 1)) {
                    if (q > piece) {
                        staircase(piece, q, riser);
                    }
                    if (q < end) {
                        walls.push_back(exact(q));
                    }
                    piece = q + 1;
                }
            }
        }
        for (Wall& w : walls) {
            if (!w.staircase) {
                w.center = w.start;
                const double len = std::hypot(w.end.x - w.start.x, w.end.y - w.start.y);
                w.direction = {(w.end.x - w.start.x) / len, (w.end.y - w.start.y) / len};
            }
        }
        // 4. Corners: where a staircase meets a neighbour, intersect their lines when
        //    they cross near the shared lattice vertex; otherwise join with a short
        //    connector. Between exact runs the corner is the exact vertex.
        const std::size_t count = walls.size();
        std::vector<WorldPoint> starts(count), ends(count);
        for (std::size_t k = 0; k < count; ++k) {
            starts[k] = walls[k].start;
            ends[k] = walls[k].end;
        }
        std::vector<bool> connector(count, false);  // between wall k and k + 1
        for (std::size_t k = 0; k < count; ++k) {
            Wall& a = walls[k];
            Wall& b = walls[(k + 1) % count];
            if (!a.staircase && !b.staircase) {
                continue;
            }
            const WorldPoint shared = world(run_at(a.first + a.count - 1).b);
            const double denominator = cross(a.direction, b.direction);
            bool joined = false;
            if (std::fabs(denominator) > 0.1) {
                const WorldPoint delta{b.center.x - a.center.x, b.center.y - a.center.y};
                const double t = cross(delta, b.direction) / denominator;
                const WorldPoint x{a.center.x + t * a.direction.x, a.center.y + t * a.direction.y};
                const bool forward_a = (x.x - starts[k].x) * a.direction.x + (x.y - starts[k].y) * a.direction.y > 0.0;
                const std::size_t kb = (k + 1) % count;
                const bool forward_b = (ends[kb].x - x.x) * b.direction.x + (ends[kb].y - x.y) * b.direction.y > 0.0;
                if (std::hypot(x.x - shared.x, x.y - shared.y) < 1.5 * r && forward_a && forward_b) {
                    ends[k] = x;
                    starts[kb] = x;
                    joined = true;
                }
            }
            connector[k] = !joined;
        }
        for (std::size_t k = 0; k < count; ++k) {
            add(starts[k], ends[k]);
            if (connector[k]) {
                add(ends[k], starts[(k + 1) % count]);
            }
        }
    }
    return segments;
}

LineSurface::LineSurface(const GridMap& map) : map_(map), segments_(fitWallSegments(map)) {}

SurfaceHit LineSurface::cast(WorldPoint origin, double angle, double max_range) const {
    const WorldPoint d{std::cos(angle), std::sin(angle)};
    SurfaceHit hit;
    hit.range = max_range;
    hit.point = {origin.x + max_range * d.x, origin.y + max_range * d.y};
    if (!map_.isFree(map_.worldToGrid(origin))) {
        hit.hit = true;  // the sensor is inside an obstacle (or off the map)
        hit.range = 0.0;
        hit.point = origin;
        return hit;
    }
    for (std::size_t i = 0; i < segments_.size(); ++i) {
        const WallSegment& s = segments_[i];
        if (s.normal.x * d.x + s.normal.y * d.y >= 0.0) {
            continue;  // only faces towards the beam
        }
        const WorldPoint e{s.b.x - s.a.x, s.b.y - s.a.y};
        const double denominator = cross(d, e);
        if (denominator == 0.0) {
            continue;
        }
        const WorldPoint w{s.a.x - origin.x, s.a.y - origin.y};
        const double t = cross(w, e) / denominator;
        const double u = cross(w, d) / denominator;
        if (t > 0.0 && t < hit.range && u >= -1e-9 && u <= 1.0 + 1e-9) {
            hit.hit = true;
            hit.range = t;
            hit.point = {origin.x + t * d.x, origin.y + t * d.y};
            hit.normal = s.normal;
            hit.smooth = true;
            hit.feature = static_cast<int>(i);
            const double length = std::hypot(e.x, e.y);
            hit.edge_distance = std::min(u, 1.0 - u) * length;
        }
    }
    return hit;
}

std::unique_ptr<SurfaceModel> makeSurface(const GridMap& map, SurfaceKind kind) {
    if (kind == SurfaceKind::Lines) {
        return std::make_unique<LineSurface>(map);
    }
    return std::make_unique<GridSurface>(map);
}

}  // namespace amr
