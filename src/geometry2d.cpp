#include "geometry2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace amr {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

double cross(WorldPoint o, WorldPoint a, WorldPoint b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

/// Projection interval of the points onto the axis (ax, ay).
void project(const std::vector<WorldPoint>& points, double ax, double ay, double& lo, double& hi) {
    lo = kInfinity;
    hi = -kInfinity;
    for (const WorldPoint& p : points) {
        const double v = p.x * ax + p.y * ay;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
}

std::vector<WorldPoint> boxCorners(const Box& b) {
    return {{b.x0, b.y0}, {b.x1, b.y0}, {b.x1, b.y1}, {b.x0, b.y1}};
}

}  // namespace

Polygon makeRectangle(WorldPoint center, double width, double height, double angle) {
    Polygon p;
    const double hw = width / 2.0;
    const double hh = height / 2.0;
    for (const WorldPoint corner : {WorldPoint{-hw, -hh}, WorldPoint{hw, -hh}, WorldPoint{hw, hh},
                                    WorldPoint{-hw, hh}}) {
        p.vertices.push_back(transformPoint(corner, angle, center));
    }
    return p;
}

WorldPoint transformPoint(WorldPoint p, double angle, WorldPoint translation) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c * p.x - s * p.y + translation.x, s * p.x + c * p.y + translation.y};
}

Polygon transformPolygon(const Polygon& polygon, double angle, WorldPoint translation) {
    Polygon out;
    for (const WorldPoint& v : polygon.vertices) {
        out.vertices.push_back(transformPoint(v, angle, translation));
    }
    return out;
}

double distanceToSegment(WorldPoint p, WorldPoint a, WorldPoint b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    const double t =
        len2 > 0.0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

bool polygonContains(const Polygon& polygon, WorldPoint p) {
    const auto& v = polygon.vertices;
    bool positive = false;
    bool negative = false;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const double c = cross(v[i], v[(i + 1) % v.size()], p);
        positive = positive || c > 0.0;
        negative = negative || c < 0.0;
    }
    return !(positive && negative);
}

double distanceToPolygon(const Polygon& polygon, WorldPoint p) {
    if (polygonContains(polygon, p)) {
        return 0.0;
    }
    const auto& v = polygon.vertices;
    double d = kInfinity;
    for (std::size_t i = 0; i < v.size(); ++i) {
        d = std::min(d, distanceToSegment(p, v[i], v[(i + 1) % v.size()]));
    }
    return d;
}

bool polygonOverlapsBox(const Polygon& polygon, const Box& box) {
    // Separating axis theorem; an overlap of zero length (touching) separates.
    constexpr double kTouch = 1e-12;
    const std::vector<WorldPoint> corners = boxCorners(box);
    std::vector<std::pair<double, double>> axes = {{1.0, 0.0}, {0.0, 1.0}};
    const auto& v = polygon.vertices;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const WorldPoint& a = v[i];
        const WorldPoint& b = v[(i + 1) % v.size()];
        const double nx = -(b.y - a.y);
        const double ny = b.x - a.x;
        const double len = std::hypot(nx, ny);
        if (len > 0.0) {
            axes.emplace_back(nx / len, ny / len);
        }
    }
    for (const auto& [ax, ay] : axes) {
        double plo, phi, blo, bhi;
        project(v, ax, ay, plo, phi);
        project(corners, ax, ay, blo, bhi);
        if (std::min(phi, bhi) - std::max(plo, blo) <= kTouch) {
            return false;
        }
    }
    return true;
}

double polygonArea(const Polygon& polygon) {
    const auto& v = polygon.vertices;
    double area = 0.0;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const WorldPoint& a = v[i];
        const WorldPoint& b = v[(i + 1) % v.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return std::fabs(area) / 2.0;
}

double intersectionArea(const Polygon& polygon, const Box& box) {
    // Sutherland-Hodgman: clip the polygon by the four half-planes of the box.
    std::vector<WorldPoint> poly = polygon.vertices;
    const auto clip = [&](auto inside, auto intersect) {
        std::vector<WorldPoint> out;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const WorldPoint& cur = poly[i];
            const WorldPoint& prev = poly[(i + poly.size() - 1) % poly.size()];
            const bool cin = inside(cur);
            const bool pin = inside(prev);
            if (cin) {
                if (!pin) {
                    out.push_back(intersect(prev, cur));
                }
                out.push_back(cur);
            } else if (pin) {
                out.push_back(intersect(prev, cur));
            }
        }
        poly = std::move(out);
    };
    const auto at_x = [](double x) {
        return [x](WorldPoint a, WorldPoint b) {
            const double t = (x - a.x) / (b.x - a.x);
            return WorldPoint{x, a.y + t * (b.y - a.y)};
        };
    };
    const auto at_y = [](double y) {
        return [y](WorldPoint a, WorldPoint b) {
            const double t = (y - a.y) / (b.y - a.y);
            return WorldPoint{a.x + t * (b.x - a.x), y};
        };
    };
    clip([&](WorldPoint p) { return p.x >= box.x0; }, at_x(box.x0));
    if (!poly.empty()) clip([&](WorldPoint p) { return p.x <= box.x1; }, at_x(box.x1));
    if (!poly.empty()) clip([&](WorldPoint p) { return p.y >= box.y0; }, at_y(box.y0));
    if (!poly.empty()) clip([&](WorldPoint p) { return p.y <= box.y1; }, at_y(box.y1));
    return poly.size() < 3 ? 0.0 : polygonArea({poly});
}

double rayToPolygon(const Polygon& polygon, WorldPoint origin, double angle) {
    if (polygonContains(polygon, origin)) {
        return 0.0;
    }
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    double best = kInfinity;
    const auto& v = polygon.vertices;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const WorldPoint& a = v[i];
        const WorldPoint& b = v[(i + 1) % v.size()];
        const double ex = b.x - a.x;
        const double ey = b.y - a.y;
        const double denom = dx * ey - dy * ex;
        if (std::fabs(denom) < 1e-15) {
            continue;  // parallel (a grazing ray along an edge meets its end vertices)
        }
        const double t = ((a.x - origin.x) * ey - (a.y - origin.y) * ex) / denom;
        const double u = ((a.x - origin.x) * dy - (a.y - origin.y) * dx) / denom;
        if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
            best = std::min(best, t);
        }
    }
    return best;
}

double rayToCircle(WorldPoint center, double radius, WorldPoint origin, double angle) {
    const double ox = origin.x - center.x;
    const double oy = origin.y - center.y;
    const double c = ox * ox + oy * oy - radius * radius;
    if (c <= 0.0) {
        return 0.0;
    }
    const double b = ox * std::cos(angle) + oy * std::sin(angle);
    const double disc = b * b - c;
    if (b >= 0.0 || disc < 0.0) {
        return kInfinity;
    }
    return -b - std::sqrt(disc);
}

}  // namespace amr
