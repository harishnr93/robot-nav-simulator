// Tests for grid alignment: continuous geometry, rasterization rules, the exact
// LiDAR in continuous worlds, point-level evidence, the alignment-robust planner,
// ground truth, and the artifact-vs-ghost recovery.
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "alignment.hpp"
#include "footprint.hpp"
#include "geometry2d.hpp"
#include "ghost_perception.hpp"
#include "test_framework.hpp"

using amr::Box;
using amr::ContinuousWorld;
using amr::GridMap;
using amr::Polygon;
using amr::RasterRule;
using amr::WorldPoint;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

amr::SensorModel exactSensor() {
    amr::SensorModel sensor;
    sensor.beams = 720;
    sensor.miss_probability = 0.0;
    sensor.spurious_probability = 0.0;
    return sensor;
}

struct PlacedDoor {
    ContinuousWorld world;
    WorldPoint start;
    WorldPoint goal;
    std::vector<WorldPoint> viewpoints;
};

PlacedDoor placeDoor(double width, double angle, WorldPoint subcell, double res = 0.1) {
    const amr::DoorwayScene scene = amr::makeDoorwayScene(width);
    const amr::GridPlacement g = amr::placeGrid({2.0, 2.0}, 4.4, res, angle, subcell);
    std::vector<Polygon> polygons;
    for (const Polygon& p : scene.obstacles) {
        polygons.push_back(amr::polygonToGridFrame(g.pose, p));
    }
    std::vector<WorldPoint> views;
    for (const WorldPoint& v : scene.viewpoints) {
        views.push_back(amr::worldToGridFrame(g.pose, v));
    }
    return {ContinuousWorld(polygons, {}, g.cells, g.cells, res),
            amr::worldToGridFrame(g.pose, scene.start), amr::worldToGridFrame(g.pose, scene.goal),
            views};
}

bool gridPassable(const GridMap& map, const PlacedDoor& d, double radius) {
    const double res = map.resolution();
    return amr::planWithFootprint(map, amr::inflateObstacles(map, radius),
                                  {static_cast<int>(d.start.x / res), static_cast<int>(d.start.y / res)},
                                  {static_cast<int>(d.goal.x / res), static_cast<int>(d.goal.y / res)})
        .success();
}

void testGeometry() {
    const Polygon square = amr::makeRectangle({1.0, 1.0}, 2.0, 2.0);  // [0, 2]^2
    CHECK(amr::polygonContains(square, {1.0, 1.0}));
    CHECK(amr::polygonContains(square, {2.0, 1.0}));   // boundary counts
    CHECK(!amr::polygonContains(square, {2.1, 1.0}));
    CHECK_NEAR(amr::distanceToPolygon(square, {3.0, 1.0}), 1.0, kTolerance);
    CHECK_NEAR(amr::distanceToPolygon(square, {3.0, 3.0}), std::sqrt(2.0), kTolerance);
    CHECK_NEAR(amr::distanceToPolygon(square, {1.0, 1.0}), 0.0, kTolerance);
    // Overlap needs a positive area: touching along an edge does not count.
    CHECK(amr::polygonOverlapsBox(square, Box{1.9, 0.0, 2.5, 1.0}));
    CHECK(!amr::polygonOverlapsBox(square, Box{2.0, 0.0, 2.5, 1.0}));
    CHECK(!amr::polygonOverlapsBox(square, Box{2.0, 2.0, 3.0, 3.0}));  // corner touch
    CHECK_NEAR(amr::intersectionArea(square, Box{1.5, 1.5, 2.5, 2.5}), 0.25, kTolerance);
    CHECK_NEAR(amr::intersectionArea(square, Box{3.0, 3.0, 4.0, 4.0}), 0.0, kTolerance);
    // A rotated square: overlap and area agree with the analytic diamond.
    const Polygon diamond = amr::makeRectangle({0.0, 0.0}, std::sqrt(2.0), std::sqrt(2.0), kPi / 4);
    CHECK_NEAR(amr::polygonArea(diamond), 2.0, 1e-12);
    CHECK_NEAR(amr::intersectionArea(diamond, Box{0.0, 0.0, 1.0, 1.0}), 0.5, 1e-12);
    // Rays.
    CHECK_NEAR(amr::rayToPolygon(square, {-1.0, 1.0}, 0.0), 1.0, kTolerance);
    CHECK_NEAR(amr::rayToPolygon(square, {1.0, 1.0}, 0.0), 0.0, kTolerance);  // inside
    CHECK(std::isinf(amr::rayToPolygon(square, {-1.0, 1.0}, kPi)));
    CHECK_NEAR(amr::rayToPolygon(square, {-1.0, -1.0}, kPi / 4), std::sqrt(2.0), 1e-12);
    CHECK_NEAR(amr::rayToCircle({3.0, 0.0}, 1.0, {0.0, 0.0}, 0.0), 2.0, kTolerance);
    CHECK(std::isinf(amr::rayToCircle({3.0, 0.0}, 1.0, {0.0, 0.0}, kPi / 2)));
    // Segment to polygon.
    CHECK_NEAR(amr::segmentToPolygon({3.0, -1.0}, {3.0, 3.0}, square), 1.0, kTolerance);
    CHECK_NEAR(amr::segmentToPolygon({-1.0, 1.0}, {3.0, 1.0}, square), 0.0, kTolerance);  // crosses
}

void testRasterRules() {
    // A rectangle aligned with the cells: all rules agree.
    const std::vector<Polygon> aligned = {amr::makeRectangle({0.5, 0.5}, 0.4, 0.2)};  // [0.3,0.7]x[0.4,0.6]
    const GridMap a = amr::rasterize(aligned, 10, 10, 0.1, RasterRule::AnyOverlap);
    const GridMap c = amr::rasterize(aligned, 10, 10, 0.1, RasterRule::CenterInside);
    const GridMap m = amr::rasterize(aligned, 10, 10, 0.1, RasterRule::Majority);
    for (std::size_t i = 0; i < a.cellCount(); ++i) {
        CHECK_EQ(a.data()[i], c.data()[i]);
        CHECK_EQ(a.data()[i], m.data()[i]);
    }
    CHECK(a.isOccupied({3, 4}));
    CHECK(!a.isOccupied({7, 4}));  // touches at x = 0.7 only
    // The center rule samples the cell center; majority needs half the area.
    const std::vector<Polygon> dot = {amr::makeRectangle({0.45, 0.45}, 0.04, 0.04)};  // cell (4, 4)
    CHECK(amr::rasterize(dot, 10, 10, 0.1, RasterRule::CenterInside).isOccupied({4, 4}));
    CHECK(!amr::rasterize(dot, 10, 10, 0.1, RasterRule::CenterInside).isOccupied({5, 5}));
    CHECK(!amr::rasterize(dot, 10, 10, 0.1, RasterRule::Majority).isOccupied({4, 4}));
    const std::vector<Polygon> third = {amr::makeRectangle({0.415, 0.45}, 0.03, 0.1)};   // 30 % of (4, 4)
    const std::vector<Polygon> sixty = {amr::makeRectangle({0.43, 0.45}, 0.06, 0.1)};    // 60 % of (4, 4)
    CHECK(!amr::rasterize(third, 10, 10, 0.1, RasterRule::Majority).isOccupied({4, 4}));
    CHECK(amr::rasterize(sixty, 10, 10, 0.1, RasterRule::Majority).isOccupied({4, 4}));
    // Random rotated rectangles: any-overlap contains the other two rules.
    std::mt19937 rng(4);
    std::uniform_real_distribution<double> pos(0.3, 0.7), size(0.05, 0.4), turn(0.0, kPi);
    for (int trial = 0; trial < 200; ++trial) {
        const std::vector<Polygon> p = {amr::makeRectangle({pos(rng), pos(rng)}, size(rng), size(rng), turn(rng))};
        const GridMap any = amr::rasterize(p, 10, 10, 0.1, RasterRule::AnyOverlap);
        const GridMap center = amr::rasterize(p, 10, 10, 0.1, RasterRule::CenterInside);
        const GridMap majority = amr::rasterize(p, 10, 10, 0.1, RasterRule::Majority);
        for (std::size_t i = 0; i < any.cellCount(); ++i) {
            CHECK(any.data()[i] >= center.data()[i]);
            CHECK(any.data()[i] >= majority.data()[i]);
        }
    }
}

void testPlacement() {
    // The grid holds the whole scene for any rotation and sub-cell offset, and the
    // transform is a rigid motion.
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> angle(0.0, 2 * kPi), sub(0.0, 0.1);
    const amr::DoorwayScene scene = amr::makeDoorwayScene(0.8);
    for (int trial = 0; trial < 100; ++trial) {
        const amr::GridPlacement g = amr::placeGrid({2.0, 2.0}, 4.4, 0.1, angle(rng), {sub(rng), sub(rng)});
        const double side = g.cells * 0.1;
        for (const Polygon& p : scene.obstacles) {
            for (const WorldPoint& v : amr::polygonToGridFrame(g.pose, p).vertices) {
                CHECK(v.x > 0.0 && v.y > 0.0 && v.x < side && v.y < side);
            }
        }
        const WorldPoint a = amr::worldToGridFrame(g.pose, scene.start);
        const WorldPoint b = amr::worldToGridFrame(g.pose, scene.goal);
        CHECK_NEAR(std::hypot(a.x - b.x, a.y - b.y), 2.0, 1e-12);
    }
}

void testContinuousLidar() {
    // A wall 1 m to the right of the sensor, a ghost disc in front of it.
    const std::vector<Polygon> walls = {amr::makeRectangle({2.6, 1.0}, 0.2, 2.0)};  // face at x = 2.5
    ContinuousWorld world(walls, {{{2.0, 1.0}, 0.05, 0.0, 0.5, 0.6}}, 30, 20, 0.1);
    std::mt19937 rng(6);
    const WorldPoint origin{1.5, 1.0};
    int ghost_hits = 0, wall_hits = 0;
    for (int i = 0; i < 4000; ++i) {
        const amr::BeamTrace t = world.trace(origin, 0.0, 5.0, exactSensor(), rng);
        REQUIRE(t.hit >= 0);
        if (std::fabs(t.hit_point.x - 1.95) < 1e-9) {
            ++ghost_hits;  // the disc's near side
        } else {
            CHECK_NEAR(t.hit_point.x, 2.5, 1e-9);
            CHECK_NEAR(t.free_length, 1.0, 1e-9);
            ++wall_hits;
        }
        CHECK_EQ(t.passed.front(), 15 + 10 * 30);  // the starting cell counts as observed
    }
    CHECK_NEAR(static_cast<double>(ghost_hits) / 4000, 0.6, 0.03);
    // From the side the ghost is out of sector: every beam passes through it.
    for (int i = 0; i < 50; ++i) {
        CHECK_EQ(world.trace({2.0, 0.2}, kPi / 2, 1.5, exactSensor(), rng).hit, -1);
    }
    CHECK(world.isReal(25 + 10 * 30));   // a cell inside the wall
    CHECK(!world.isReal(20 + 10 * 30));  // the ghost's cell is free in reality
}

void testPointEvidence() {
    amr::PointMap points(30, 20, 0.1);
    // A hit at (2.0, 1.0) from the west.
    amr::BeamTrace hit;
    hit.origin = {1.0, 1.0};
    hit.angle = 0.0;
    hit.free_length = 1.0;
    hit.hit = 10 * 30 + 20;
    hit.hit_point = {2.0, 1.0};
    points.integrate(hit, 0, 0);
    hit.hit_point = {2.0, 1.004};  // a second, supporting hit nearby
    points.integrate(hit, 0, 1);
    REQUIRE(points.points().size() == 2u);
    CHECK(points.usable(0));
    // A beam passing 3 mm away and continuing 0.5 m contradicts it; two do.
    amr::BeamTrace pass;
    pass.origin = {2.0, 0.5};
    pass.angle = kPi / 2;
    pass.free_length = 1.0;
    pass.passed = {10 * 30 + 20};
    for (int i = 0; i < 2; ++i) {
        points.integrate(pass, 1, i);
    }
    CHECK(!points.usable(0));
    // A beam that stops just past a point (a grazing hit on the same surface) does not.
    amr::PointMap corner(30, 20, 0.1);
    corner.integrate(hit, 0, 0);
    hit.hit_point = {2.0, 1.03};
    corner.integrate(hit, 0, 1);
    pass.free_length = 0.55;  // ends 5 cm past the point: not "through" it
    corner.integrate(pass, 1, 0);
    corner.integrate(pass, 1, 1);
    CHECK(corner.usable(0));
    // An isolated point (a spurious return) is not an obstacle.
    amr::PointMap lone(30, 20, 0.1);
    hit.hit_point = {1.0, 0.5};
    hit.hit = 5 * 30 + 10;
    lone.integrate(hit, 0, 0);
    CHECK(!lone.usable(0));
    CHECK(lone.segments(0.6, 2 * kPi / 720).empty());
}

void testSegmentsAndPlanner() {
    // Two walls with a gap: dense hits along their faces.
    amr::PointMap points(40, 40, 0.1);
    const auto add_wall = [&](double x0, double x1, int scan) {
        int beam = 0;
        for (double x = x0; x <= x1 + 1e-9; x += 0.01) {
            amr::BeamTrace t;
            t.origin = {x, 1.0};
            t.angle = kPi / 2;
            t.free_length = 1.0;
            t.hit_point = {x, 2.0};
            t.hit = 20 * 40 + static_cast<int>(x / 0.1);
            points.integrate(t, scan, beam++);
        }
    };
    add_wall(0.0, 1.66, 0);
    add_wall(2.34, 4.0, 1);  // gap 0.68 m between 1.66 and 2.34
    const auto segments = points.segments(0.6, 2 * kPi / 720);
    CHECK(segments.size() > 300u);
    const auto through = [&](double radius) {
        return amr::planOnClearance(segments, 40, 40, 0.1, {2.0, 1.0}, {2.0, 3.0}, radius, 0.025).found;
    };
    CHECK(through(0.30));   // 0.68 >= 0.6 + sqrt(2) * 0.025
    CHECK(!through(0.35));  // 0.68 < 0.70
    // Chain ends carry a margin of range x beam spacing; points inside a chain none.
    const double step = 2 * kPi / 720;
    int ends = 0;
    for (const auto& sg : segments) {
        if (sg.margin > 0.0) {
            CHECK_NEAR(sg.margin, 1.0 * step, 1e-6);  // these hits were 1 m away
            ++ends;
        }
    }
    CHECK_EQ(ends, 4);  // both ends of both walls
    // With a 3 m range the margins close a 0.655 m gap that would otherwise pass:
    // the true corners may lie up to one beam beyond the last points seen.
    amr::PointMap far(40, 40, 0.1);
    const auto add_far = [&](double x0, double x1, int scan) {
        int beam = 0;
        for (double x = x0; x <= x1 + 1e-9; x += 0.01) {
            amr::BeamTrace t;
            t.origin = {x, -1.0};
            t.angle = kPi / 2;
            t.free_length = 3.0;
            t.hit_point = {x, 2.0};
            t.hit = 20 * 40 + static_cast<int>(x / 0.1);
            far.integrate(t, scan, beam++);
        }
    };
    add_far(0.0, 1.6725, 0);
    add_far(2.3275, 4.0, 1);  // gap 0.655
    const auto far_segments = far.segments(0.6, step);
    CHECK(!amr::planOnClearance(far_segments, 40, 40, 0.1, {2.0, 1.0}, {2.0, 3.0}, 0.3, 0.025).found);
    auto no_margin = far_segments;
    for (auto& sg : no_margin) {
        sg.margin = 0.0;
    }
    CHECK(amr::planOnClearance(no_margin, 40, 40, 0.1, {2.0, 1.0}, {2.0, 3.0}, 0.3, 0.025).found);
    // With an observation mask that hides the gap, it is not used.
    std::vector<bool> observed(1600, true);
    for (int x = 15; x <= 25; ++x) {
        observed[static_cast<std::size_t>(20 * 40 + x)] = false;
    }
    CHECK(!amr::planOnClearance(segments, 40, 40, 0.1, {2.0, 1.0}, {2.0, 3.0}, 0.3, 0.025, &observed).found);
}

/// The point planner gives the same answer for every alignment outside its
/// tolerance band [2 r, 2 r + 2 R dtheta + sqrt(2) h] (corner margins of up to range x
/// beam spacing on each side, plus the lattice), while the grid's answer varies.
void testAlignmentInvariance() {
    std::mt19937 rng(8);
    std::uniform_real_distribution<double> angle(0.0, kPi / 2), sub(0.0, 0.1);
    for (const double width : {0.55, 0.70, 0.75, 0.95}) {
        int grid_passable = 0, points_passable = 0;
        const int trials = 16;
        for (int t = 0; t < trials; ++t) {
            const PlacedDoor d = placeDoor(width, angle(rng), {sub(rng), sub(rng)});
            amr::OccupancyBelief belief(d.world.width(), d.world.height(), 0.1);
            amr::PointMap points(d.world.width(), d.world.height(), 0.1);
            for (const WorldPoint& v : d.viewpoints) {
                amr::scanInto(belief, points, d.world, v, exactSensor(), rng);
            }
            const std::vector<bool> observed = amr::observedCells(belief);
            points_passable += amr::planOnClearance(points.segments(0.6, 2 * kPi / 720), d.world.width(),
                                                    d.world.height(), 0.1, d.start, d.goal, 0.3,
                                                    0.025, &observed)
                                   .found;
            grid_passable += gridPassable(
                amr::rasterize(d.world.obstacles(), d.world.width(), d.world.height(), 0.1,
                               RasterRule::AnyOverlap),
                d, 0.3);
        }
        const bool truth = width >= 0.6;
        CHECK_EQ(points_passable, truth ? trials : 0);
        if (width == 0.75) {
            CHECK(grid_passable > 0 && grid_passable < trials);  // alignment decides
        }
        if (!truth) {
            CHECK_EQ(grid_passable, 0);  // any-overlap is never unsafe
        }
    }
}

void testTruePassability() {
    for (const auto& [width, expected] :
         std::vector<std::pair<double, amr::Passability>>{{0.62, amr::Passability::Passable},
                                                          {0.58, amr::Passability::Blocked},
                                                          {0.60, amr::Passability::Borderline}}) {
        const PlacedDoor d = placeDoor(width, 0.3, {0.02, 0.07});
        CHECK(amr::truePassability(d.world, d.start, d.goal, 0.3, 0.005) == expected);
    }
}

/// End to end: a grid NO_PATH that is a discretization artifact is recovered with
/// no sensing; a door too narrow is never opened.
void testArtifactAndRealBlock() {
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> angle(0.0, kPi / 2), sub(0.0, 0.1);
    int artifacts = 0;
    for (int t = 0; t < 40 && artifacts < 3; ++t) {
        const PlacedDoor d = placeDoor(0.72, angle(rng), {sub(rng), sub(rng)});
        amr::OccupancyBelief belief(d.world.width(), d.world.height(), 0.1);
        amr::PointMap points(d.world.width(), d.world.height(), 0.1);
        for (const WorldPoint& v : d.viewpoints) {
            amr::scanInto(belief, points, d.world, v, exactSensor(), rng);
        }
        if (gridPassable(belief.map(), d, 0.3)) {
            continue;
        }
        ++artifacts;
        const auto r = amr::recoverWithPoints(belief, points, d.world, d.start, d.goal, exactSensor(),
                                              amr::PointRecoveryConfig{}, rng);
        CHECK(r.artifact);
        CHECK(r.outcome == amr::RecoveryOutcome::Recovered);
        CHECK_EQ(r.beams, 0);
    }
    CHECK(artifacts >= 1);
    // A reflection in a wide door, seen from inside its sector while mapping: the points
    // block, re-sensing from the side removes it. Not an artifact: beams were needed.
    int resolved = 0;
    for (int t = 0; t < 20 && resolved < 2; ++t) {
        const amr::DoorwayScene scene = amr::makeDoorwayScene(1.0);
        const amr::GridPlacement g = amr::placeGrid({2.0, 2.0}, 4.4, 0.1, angle(rng), {sub(rng), sub(rng)});
        std::vector<Polygon> polygons;
        for (const Polygon& p : scene.obstacles) {
            polygons.push_back(amr::polygonToGridFrame(g.pose, p));
        }
        // Reflects beams heading "down" the room (world +y) within +-70 deg.
        const amr::GhostDisc ghost{amr::worldToGridFrame(g.pose, scene.door_center), 0.08,
                                   kPi / 2 - g.pose.angle, 1.2, 1.0};
        const ContinuousWorld world(polygons, {ghost}, g.cells, g.cells, 0.1);
        const WorldPoint start = amr::worldToGridFrame(g.pose, scene.start);
        const WorldPoint goal = amr::worldToGridFrame(g.pose, scene.goal);
        amr::OccupancyBelief belief(g.cells, g.cells, 0.1);
        amr::PointMap points(g.cells, g.cells, 0.1);
        for (const WorldPoint& v : scene.viewpoints) {
            amr::scanInto(belief, points, world, amr::worldToGridFrame(g.pose, v), exactSensor(), rng);
        }
        if (amr::planOnClearance(points.segments(0.6, 2 * kPi / 720), g.cells, g.cells, 0.1, start,
                                 goal, 0.3, 0.025).found) {
            continue;  // the ghost did not block this time
        }
        const auto r = amr::recoverWithPoints(belief, points, world, start, goal, exactSensor(),
                                              amr::PointRecoveryConfig{}, rng);
        if (r.outcome == amr::RecoveryOutcome::Recovered) {
            CHECK(!r.artifact);
            CHECK(r.beams > 0);
            ++resolved;
        }
        CHECK(r.outcome != amr::RecoveryOutcome::Unsafe);
    }
    CHECK(resolved >= 1);
    for (int t = 0; t < 3; ++t) {
        const PlacedDoor d = placeDoor(0.55, angle(rng), {sub(rng), sub(rng)});
        amr::OccupancyBelief belief(d.world.width(), d.world.height(), 0.1);
        amr::PointMap points(d.world.width(), d.world.height(), 0.1);
        for (const WorldPoint& v : d.viewpoints) {
            amr::scanInto(belief, points, d.world, v, exactSensor(), rng);
        }
        amr::PointRecoveryConfig config;
        config.max_rounds = 3;
        const auto r = amr::recoverWithPoints(belief, points, d.world, d.start, d.goal, exactSensor(),
                                              config, rng);
        CHECK(r.outcome == amr::RecoveryOutcome::GaveUp);
    }
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Geometry: containment, distance, overlap, area, rays", testGeometry},
        {"Rasterization rules", testRasterRules},
        {"Grid placement covers the scene at any rotation", testPlacement},
        {"Exact LiDAR in a continuous world, ghost discs", testContinuousLidar},
        {"Point evidence: contradiction, grazing, outliers", testPointEvidence},
        {"Point segments and the clearance planner", testSegmentsAndPlanner},
        {"Point planner is alignment-invariant; the grid is not", testAlignmentInvariance},
        {"Ground truth on a fine lattice", testTruePassability},
        {"Artifact recovered without sensing; real block never opened", testArtifactAndRealBlock},
    });
}
