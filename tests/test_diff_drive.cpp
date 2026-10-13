// Tests for the differential-drive robot: unicycle kinematics, the P heading
// controller, and the turn-then-drive waypoint follower in the simulator.
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::Connectivity;
using amr::GridCoord;
using amr::GridMap;
using amr::MotionConfig;
using amr::NavigationStatus;
using amr::Pose;
using amr::SimulationResult;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

MotionConfig diffDrive() {
    MotionConfig config;
    config.model = amr::RobotModel::DiffDrive;
    return config;
}

GridMap makeRandomMap(std::mt19937& rng, int width, int height, double density,
                      double resolution) {
    std::bernoulli_distribution occupied(density);
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(width) *
                                    static_cast<std::size_t>(height));
    for (auto& cell : cells) {
        cell = occupied(rng) ? GridMap::kOccupied : GridMap::kFree;
    }
    return GridMap(width, height, resolution, std::move(cells));
}

bool isTurnSample(const amr::TrajectorySample& s) {
    return s.linear_velocity == 0.0 && s.angular_velocity != 0.0;
}

void testWrapAngle() {
    CHECK_NEAR(amr::wrapAngle(0.0), 0.0, kTolerance);
    CHECK_NEAR(amr::wrapAngle(kPi), kPi, kTolerance);
    CHECK_NEAR(amr::wrapAngle(-kPi), kPi, kTolerance);           // (-pi, pi]
    CHECK_NEAR(amr::wrapAngle(3 * kPi), kPi, kTolerance);
    CHECK_NEAR(amr::wrapAngle(2 * kPi), 0.0, kTolerance);
    CHECK_NEAR(amr::wrapAngle(-1.5 * kPi), 0.5 * kPi, kTolerance);
    CHECK_NEAR(amr::wrapAngle(0.25 + 20 * kPi), 0.25, 1e-9);
}

void testUnicycleIntegration() {
    // Straight line and pure rotation.
    const Pose straight = amr::integrateUnicycle({1.0, 2.0, 0.0}, 0.5, 0.0, 2.0);
    CHECK_NEAR(straight.x, 2.0, kTolerance);
    CHECK_NEAR(straight.y, 2.0, kTolerance);
    const Pose spin = amr::integrateUnicycle({1.0, 2.0, 0.3}, 0.0, 1.5, 0.1);
    CHECK_NEAR(spin.x, 1.0, kTolerance);
    CHECK_NEAR(spin.y, 2.0, kTolerance);
    CHECK_NEAR(spin.theta, 0.45, kTolerance);

    // Quarter circle of radius 1: (0, 0, 0) -> (1, 1, pi/2) with +y "down".
    const Pose quarter = amr::integrateUnicycle({0.0, 0.0, 0.0}, 1.0, 1.0, kPi / 2);
    CHECK_NEAR(quarter.x, 1.0, kTolerance);
    CHECK_NEAR(quarter.y, 1.0, kTolerance);
    CHECK_NEAR(quarter.theta, kPi / 2, kTolerance);

    // The closed form is exact: two half steps equal one full step...
    const Pose start{0.3, -0.7, 2.9};
    const Pose once = amr::integrateUnicycle(start, 0.8, -1.3, 0.6);
    const Pose twice = amr::integrateUnicycle(amr::integrateUnicycle(start, 0.8, -1.3, 0.3),
                                              0.8, -1.3, 0.3);
    CHECK_NEAR(once.x, twice.x, 1e-12);
    CHECK_NEAR(once.y, twice.y, 1e-12);
    CHECK_NEAR(once.theta, twice.theta, 1e-12);
    // ...and agrees with a fine Euler integration of dx = v cos, dy = v sin, dtheta = w.
    Pose euler = start;
    const int substeps = 200000;
    for (int i = 0; i < substeps; ++i) {
        const double h = 0.6 / substeps;
        euler = {euler.x + 0.8 * std::cos(euler.theta) * h,
                 euler.y + 0.8 * std::sin(euler.theta) * h, euler.theta - 1.3 * h};
    }
    CHECK_NEAR(once.x, euler.x, 1e-5);
    CHECK_NEAR(once.y, euler.y, 1e-5);
    CHECK_NEAR(once.theta, amr::wrapAngle(euler.theta), 1e-9);
}

void testHeadingController() {
    // gain 5 1/s, limit 1.5 rad/s, tolerance 0.01 rad, dt 0.1 s.
    CHECK_NEAR(amr::headingControl(1.0, 5.0, 1.5, 0.01, 0.1), 1.5, kTolerance);   // saturated
    CHECK_NEAR(amr::headingControl(-1.0, 5.0, 1.5, 0.01, 0.1), -1.5, kTolerance);
    CHECK_NEAR(amr::headingControl(0.2, 5.0, 1.5, 0.01, 0.1), 1.0, kTolerance);   // P region
    CHECK_NEAR(amr::headingControl(0.005, 5.0, 1.5, 0.01, 0.1), 0.05, kTolerance); // settle

    // No overshoot, monotone convergence, exact landing (gain * dt <= 1).
    std::mt19937 rng(4);
    std::uniform_real_distribution<double> error_dist(-kPi, kPi);
    for (int trial = 0; trial < 1000; ++trial) {
        double error = error_dist(rng);
        const double gain = 1.0 + trial % 10;  // gain * dt from 0.1 to 1.0
        int steps = 0;
        while (error != 0.0 && steps < 1000) {
            const double omega = amr::headingControl(error, gain, 1.5, 0.01, 0.1);
            CHECK(std::fabs(omega) <= 1.5 + 1e-12);
            double next = error - omega * 0.1;
            if (std::fabs(next) < 1e-12) {
                next = 0.0;  // exact landing up to floating-point rounding
            }
            CHECK(next * error >= 0.0);              // never crosses the target
            CHECK(std::fabs(next) < std::fabs(error));
            error = next;
            ++steps;
        }
        CHECK(steps < 1000);
    }
}

void testStraightPathNeedsNoTurn() {
    const GridMap map(10, 1, 0.5);
    const auto plan = amr::planAStar(map, {0, 0}, {9, 0});
    REQUIRE(plan.success());
    const SimulationResult point = amr::simulatePath(map, plan.path, MotionConfig{});
    const SimulationResult dd = amr::simulatePath(map, plan.path, diffDrive());
    CHECK(dd.status == NavigationStatus::GoalReached);
    CHECK_EQ(dd.turns, static_cast<std::size_t>(0));
    CHECK_NEAR(dd.turn_time, 0.0, kTolerance);
    REQUIRE(dd.trajectory.size() == point.trajectory.size());
    for (std::size_t i = 0; i < dd.trajectory.size(); ++i) {
        CHECK_EQ(dd.trajectory[i].state.x, point.trajectory[i].state.x);  // bit-identical
        CHECK_EQ(dd.trajectory[i].state.time, point.trajectory[i].state.time);
        CHECK_NEAR(dd.trajectory[i].state.theta, 0.0, kTolerance);
    }
    // Collinear waypoints are passed without stopping: v is at its limit throughout.
    for (std::size_t i = 1; i < dd.trajectory.size(); ++i) {
        CHECK_NEAR(dd.trajectory[i].linear_velocity, 0.5, 1e-12);
    }
}

void testTurnTimes() {
    // Facing backwards (180 deg): 19 saturated steps (1.5 rad/s) bring the error to
    // 0.2916 rad, then the P region halves it 5 times (gain * dt = 0.5) to 0.0091,
    // below the 0.01 tolerance, and 1 settling step lands it: 25 steps = 2.5 s.
    const GridMap map(10, 1, 0.5);
    const auto plan = amr::planAStar(map, {0, 0}, {9, 0});
    MotionConfig config = diffDrive();
    config.initial_heading = kPi;
    const SimulationResult back = amr::simulatePath(map, plan.path, config);
    CHECK(back.status == NavigationStatus::GoalReached);
    CHECK_EQ(back.turns, static_cast<std::size_t>(1));
    CHECK_NEAR(back.turn_time, 2.5, 1e-9);
    CHECK_NEAR(back.total_rotation, kPi, 1e-9);
    CHECK_NEAR(back.trajectory.back().state.theta, 0.0, 1e-9);
    CHECK_NEAR(back.travelTime(), 9.0 + 2.5, 1e-9);  // 4.5 m at 0.5 m/s (9 s) + turning

    // One 90 deg corner: 9 saturated + 5 P + 1 settling step = 1.5 s.
    const GridMap open(4, 4, 0.5);
    const std::vector<GridCoord> corner{{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
    const SimulationResult turn = amr::simulatePath(open, corner, diffDrive());
    CHECK(turn.status == NavigationStatus::GoalReached);
    CHECK_EQ(turn.turns, static_cast<std::size_t>(1));
    CHECK_NEAR(turn.turn_time, 1.5, 1e-9);
    CHECK_NEAR(turn.total_rotation, kPi / 2, 1e-9);
    CHECK_NEAR(turn.trajectory.back().state.theta, kPi / 2, 1e-9);  // facing +y (down)
}

// The turn-then-drive follower must reproduce the point robot's motion exactly, with
// turn-in-place pauses inserted, and respect every kinematic limit.
void testFollowsPlannedPathExactly() {
    std::mt19937 rng(77);
    std::uniform_int_distribution<int> size_dist(3, 40);
    std::uniform_real_distribution<double> heading_dist(-kPi, kPi);
    int checked = 0;
    for (int trial = 0; trial < 200; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        GridMap map = makeRandomMap(rng, width, height, 0.25, 0.5);
        std::uniform_int_distribution<int> x_dist(0, width - 1);
        std::uniform_int_distribution<int> y_dist(0, height - 1);
        const GridCoord start{x_dist(rng), y_dist(rng)};
        const GridCoord goal{x_dist(rng), y_dist(rng)};
        map.setOccupied(start, false);
        map.setOccupied(goal, false);
        const Connectivity connectivity = trial % 2 ? Connectivity::Eight : Connectivity::Four;
        const auto plan = amr::planAStar(map, start, goal, connectivity);
        if (!plan.success()) {
            continue;
        }
        MotionConfig config = diffDrive();
        config.initial_heading = heading_dist(rng);
        config.max_time = 5000.0;
        const SimulationResult point =
            amr::simulatePath(map, plan.path, MotionConfig{}, connectivity);
        const SimulationResult dd = amr::simulatePath(map, plan.path, config, connectivity);
        REQUIRE(dd.status == NavigationStatus::GoalReached);
        CHECK_EQ(dd.collision_count, static_cast<std::size_t>(0));
        ++checked;

        // Dropping the turn-in-place samples leaves exactly the point robot's positions.
        std::vector<amr::TrajectorySample> moving;
        for (const auto& s : dd.trajectory) {
            if (!isTurnSample(s)) {
                moving.push_back(s);
            }
        }
        REQUIRE(moving.size() == point.trajectory.size());
        for (std::size_t i = 0; i < moving.size(); ++i) {
            CHECK_EQ(moving[i].state.x, point.trajectory[i].state.x);
            CHECK_EQ(moving[i].state.y, point.trajectory[i].state.y);
        }
        CHECK_NEAR(dd.travelTime(), point.travelTime() + dd.turn_time, 1e-9);
        CHECK_NEAR(dd.distance_traveled, point.distance_traveled, 1e-12);

        for (std::size_t i = 1; i < dd.trajectory.size(); ++i) {
            const auto& prev = dd.trajectory[i - 1];
            const auto& s = dd.trajectory[i];
            CHECK(s.linear_velocity <= config.speed + 1e-12);
            CHECK(std::fabs(s.angular_velocity) <= config.max_angular_speed + 1e-12);
            CHECK(s.linear_velocity == 0.0 || s.angular_velocity == 0.0);  // turn OR drive
            CHECK(std::fabs(amr::wrapAngle(s.state.theta - prev.state.theta)) <=
                  config.max_angular_speed * config.dt + 1e-12);
            if (s.linear_velocity > 0.0) {
                // While driving, the heading is the direction of motion.
                const double motion = std::atan2(s.state.y - prev.state.y, s.state.x - prev.state.x);
                CHECK(std::fabs(amr::wrapAngle(motion - s.state.theta)) < 1e-6);
            }
        }
    }
    CHECK(checked > 100);
}

void testFootprintAndDiagonals() {
    std::mt19937 rng(5);
    std::uniform_int_distribution<int> size_dist(4, 35);
    std::uniform_real_distribution<double> radius_cells(0.0, 1.2);
    int successes = 0;
    for (int trial = 0; trial < 200; ++trial) {
        const int width = size_dist(rng);
        const int height = size_dist(rng);
        const GridMap map = makeRandomMap(rng, width, height, 0.2, 0.25);
        const double radius = radius_cells(rng) * 0.25;
        const GridMap inflated = amr::inflateObstacles(map, radius);
        std::vector<GridCoord> fits;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (inflated.isFree({x, y})) {
                    fits.push_back({x, y});
                }
            }
        }
        if (fits.empty()) {
            continue;
        }
        std::uniform_int_distribution<std::size_t> pick(0, fits.size() - 1);
        const Connectivity connectivity = trial % 2 ? Connectivity::Eight : Connectivity::Four;
        const auto plan = amr::planWithFootprint(map, inflated, fits[pick(rng)], fits[pick(rng)],
                                                 connectivity);
        if (!plan.success()) {
            continue;
        }
        MotionConfig config = diffDrive();
        config.robot_radius = radius;
        config.max_time = 5000.0;
        const SimulationResult sim = amr::simulatePath(map, plan.path, config, connectivity);
        CHECK(sim.status == NavigationStatus::GoalReached);
        CHECK_EQ(sim.collision_count, static_cast<std::size_t>(0));
        ++successes;
    }
    CHECK(successes > 100);
}

void testTimeoutWhileTurning() {
    const GridMap map(10, 1, 0.5);
    const auto plan = amr::planAStar(map, {0, 0}, {9, 0});
    MotionConfig config = diffDrive();
    config.initial_heading = kPi;
    config.max_time = 1.0;  // the 180 deg turn alone needs 2.5 s
    const SimulationResult sim = amr::simulatePath(map, plan.path, config);
    CHECK(sim.status == NavigationStatus::Timeout);
    CHECK_EQ(sim.steps, static_cast<std::size_t>(10));
    CHECK_NEAR(sim.distance_traveled, 0.0, kTolerance);  // still turning in place
}

void testValidation() {
    MotionConfig config = diffDrive();
    CHECK(amr::validateMotionConfig(config).empty());
    config.heading_gain = 11.0;  // gain * dt = 1.1 > 1
    CHECK(!amr::validateMotionConfig(config).empty());
    config = diffDrive();
    config.max_angular_speed = 0.0;
    CHECK(!amr::validateMotionConfig(config).empty());
    config = diffDrive();
    config.heading_tolerance = 0.5;  // > max_angular_speed * dt = 0.15
    CHECK(!amr::validateMotionConfig(config).empty());
    config = diffDrive();
    config.initial_heading = std::nan("");
    CHECK(!amr::validateMotionConfig(config).empty());
    // The point robot ignores (and does not validate) the differential-drive settings.
    MotionConfig point;
    point.heading_gain = 1000.0;
    point.dt = 1.0;
    CHECK(amr::validateMotionConfig(point).empty());
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Angle wrapping", testWrapAngle},
        {"Exact unicycle integration (line, spin, arc, Euler check)", testUnicycleIntegration},
        {"P heading controller: saturation, settle, no overshoot", testHeadingController},
        {"Straight path: no turn, identical to the point robot", testStraightPathNeedsNoTurn},
        {"Turn-in-place times (180 deg, 90 deg)", testTurnTimes},
        {"Follows the planned path exactly, within limits (200 maps)", testFollowsPlannedPathExactly},
        {"Footprint and diagonals: no collisions (200 maps)", testFootprintAndDiagonals},
        {"Timeout while turning", testTimeoutWhileTurning},
        {"Validation of differential-drive settings", testValidation},
    });
}
