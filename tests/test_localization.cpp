// Tests for global localization and its consequences: the particle filter (global
// start, tracking, kidnapping, matching places), localizability and the
// localization-aware planner, and steering by the estimate.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "closed_loop.hpp"
#include "diff_drive.hpp"
#include "ekf.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "localization_planning.hpp"
#include "particle_filter.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::GridMap;

namespace {

constexpr double kPi = 3.14159265358979323846;

GridMap load(const std::string& name, double resolution = 0.5) {
    const auto loaded = amr::loadGridMapCsv(std::string(AMR_SOURCE_DIR) + "/maps/" + name, resolution);
    CHECK(loaded.ok);
    return loaded.map;
}

std::vector<amr::TrajectorySample> drive(const GridMap& map, amr::GridCoord start, amr::GridCoord goal,
                                         double heading = 0.3) {
    const GridMap inflated = amr::inflateObstacles(map, 0.25);
    const auto plan = amr::planWithFootprint(map, inflated, start, goal);
    amr::MotionConfig motion;
    motion.model = amr::RobotModel::DiffDrive;
    motion.robot_radius = 0.25;
    motion.initial_heading = heading;
    return amr::simulatePath(map, plan.path, motion).trajectory;
}

amr::LidarConfig lidar72(std::uint32_t seed = 1) {
    amr::LidarConfig lidar;
    lidar.beams = 72;
    lidar.noise_stddev = 0.02;
    lidar.seed = seed;
    return lidar;
}

double error(const amr::Pose& a, const amr::Pose& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// ---------------------------------------------------------------------------------
// Particle filter

void testGlobalLocalization() {
    const GridMap map = load("obstacles.csv");
    const auto trajectory = drive(map, {1, 1}, {18, 18});
    const auto lidar = lidar72();
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    amr::OdometryNoise odometry;
    odometry.seed = 4;
    const auto samples = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, {}, true);
    REQUIRE(samples.size() == trajectory.size());
    // Dense global start (50 000 particles), already resampled down by the first scan.
    CHECK(samples.front().particles >= 5000u);
    // Found within a second, kept to the end.
    double sq = 0.0;
    for (std::size_t k = 10; k < samples.size(); ++k) {
        const double e = error(samples[k].truth, samples[k].estimate.pose);
        CHECK(e < 0.15);
        sq += e * e;
    }
    CHECK(std::sqrt(sq / static_cast<double>(samples.size() - 10)) < 0.03);
    CHECK(samples.back().estimate.share > 0.9);
    CHECK(samples.back().particles <= 2000u);  // KLD-sampling shrank the set
    CHECK_EQ(samples.back().relocalizations, 0u);
    // Deterministic for a seed.
    const auto again = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, {}, true);
    CHECK_EQ(again.back().estimate.pose.x, samples.back().estimate.pose.x);
}

/// Carried 3+ m with the odometry reporting nothing: the particle filter notices the
/// scans stopped fitting and finds the robot again; without recovery, and the EKF,
/// stay lost.
void testKidnappedRobot() {
    const GridMap map = load("obstacles.csv");
    auto trajectory = drive(map, {1, 1}, {9, 2});
    const auto second = drive(map, {16, 17}, {3, 18});
    REQUIRE(!trajectory.empty() && !second.empty());
    const std::size_t kidnap = trajectory.size();
    const double offset = trajectory.back().state.time + 0.1;
    std::vector<bool> kidnapped(trajectory.size(), false);
    for (std::size_t k = 0; k < second.size(); ++k) {
        amr::TrajectorySample s = second[k];
        s.state.time += offset;
        if (k == 0) {
            s.linear_velocity = s.angular_velocity = 0.0;
        }
        trajectory.push_back(s);
        kidnapped.push_back(k == 0);
    }
    const auto lidar = lidar72(2);
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    amr::OdometryNoise odometry;
    odometry.seed = 5;
    amr::ParticleFilterConfig config;
    const auto recovering = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, config, false, kidnapped);
    config.recover = false;
    const auto stubborn = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, config, false, kidnapped);
    CHECK(error(recovering[kidnap - 1].truth, recovering[kidnap - 1].estimate.pose) < 0.05);
    // Recovered within 3 s and stays so.
    for (std::size_t k = kidnap + 30; k < recovering.size(); ++k) {
        CHECK(error(recovering[k].truth, recovering[k].estimate.pose) < 0.15);
    }
    CHECK(error(stubborn.back().truth, stubborn.back().estimate.pose) > 1.0);
    amr::EkfConfig ekf;
    ekf.enabled = true;
    ekf.range_sigma = 0.02;
    ekf.odometry = odometry;
    const auto ekf_run = amr::runLocalization(map, trajectory, scans, lidar, ekf, kidnapped);
    CHECK(error(ekf_run.back().truth, ekf_run.back().estimate) > 1.0);
}

/// A room that looks the same after a half turn: the filter keeps both places, and
/// never drops the true one.
void testSymmetricRoomKeepsBothPlaces() {
    const GridMap map = load("point_symmetric.csv");
    const double w = map.width() * map.resolution(), h = map.height() * map.resolution();
    double final_truth = 0.0;
    const int runs = 4;
    for (int run = 0; run < runs; ++run) {
        const auto trajectory = drive(map, {3, 7}, {19, 8}, 0.5 * run);
        REQUIRE(!trajectory.empty());
        const auto lidar = lidar72(static_cast<std::uint32_t>(10 + run));
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        amr::ParticleFilterConfig config;
        config.seed = static_cast<std::uint32_t>(run);
        amr::ParticleFilter filter(map, config);
        filter.initializeGlobal();
        const auto offsets = amr::beamOffsets(lidar);
        bool dropped = false;
        double near_truth = 0.0;
        for (std::size_t k = 0; k < trajectory.size(); ++k) {
            if (k > 0) {
                const auto inc = amr::trueIncrement(trajectory[k - 1].state, trajectory[k].state);
                filter.predict(inc, {1e-6, 1e-6});
            }
            filter.update(offsets, scans[k].ranges, scans[k].hits, lidar.max_range);
            const auto& s = trajectory[k].state;
            const amr::Pose truth{s.x, s.y, s.theta};
            const amr::Pose image{w - s.x, h - s.y, amr::wrapAngle(s.theta + kPi)};
            near_truth = 0.0;
            double near_image = 0.0;
            for (const amr::Particle& p : filter.particles()) {
                if (error(p.pose, truth) < 0.5 && std::fabs(amr::wrapAngle(p.pose.theta - truth.theta)) < 0.35) {
                    near_truth += p.weight;
                } else if (error(p.pose, image) < 0.5 &&
                           std::fabs(amr::wrapAngle(p.pose.theta - image.theta)) < 0.35) {
                    near_image += p.weight;
                }
            }
            if (k >= 20) {
                dropped = dropped || near_truth < 1e-9;
                CHECK(near_truth + near_image > 0.98);  // all weight on the two places
            }
        }
        CHECK(!dropped);
        final_truth += near_truth;
    }
    std::cout << "    mean final weight on the true place: " << final_truth / runs << '\n';
    CHECK(final_truth / runs > 0.2 && final_truth / runs < 0.8);
}

/// Twin rooms told apart by one block: the filter ends in the right room.
void testOneDifferenceDecides() {
    const GridMap map = load("twin_rooms_marked.csv");
    for (int run = 0; run < 3; ++run) {
        const auto trajectory = drive(map, {3, 6}, {12, 10}, 1.0 * run);
        REQUIRE(!trajectory.empty());
        const auto lidar = lidar72(static_cast<std::uint32_t>(20 + run));
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        amr::OdometryNoise odometry;
        odometry.seed = static_cast<std::uint32_t>(run);
        amr::ParticleFilterConfig config;
        config.seed = static_cast<std::uint32_t>(run);
        const auto samples = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, config, true);
        CHECK(error(samples.back().truth, samples.back().estimate.pose) < 0.2);
    }
}

// ---------------------------------------------------------------------------------
// Localizability and planning

void testLocalizability() {
    const GridMap map = load("hall.csv");
    amr::LocalizabilityConfig config;
    config.beams = 72;
    config.max_range = 3.0;
    config.range_sigma = 0.02;
    const auto u = amr::computeLocalizability(map, config);
    const auto at = [&](int x, int y) { return u[static_cast<std::size_t>(y * map.width() + x)]; };
    CHECK(at(2, 2) < 0.05);           // a corner: pinned
    CHECK(at(22, 15) > 5.0);          // the middle: nothing in range
    CHECK(std::isinf(at(0, 0)));      // a wall
    // Monotone in the evidence: more range means better localizability everywhere.
    config.max_range = 6.0;
    const auto wide = amr::computeLocalizability(map, config);
    int worse = 0;
    for (std::size_t i = 0; i < u.size(); ++i) {
        worse += std::isfinite(u[i]) && wide[i] > u[i] * (1.0 + 1e-9) ? 1 : 0;
    }
    CHECK_EQ(worse, 0);
}

void testLocalizationAwarePlanning() {
    const GridMap map = load("hall.csv");
    const GridMap inflated = amr::inflateObstacles(map, 0.25);
    amr::LocalizabilityConfig lc;
    lc.max_range = 3.0;
    const auto u = amr::computeLocalizability(map, lc);
    const amr::GridCoord start{6, 8}, goal{38, 22};
    amr::LocalizationPlanConfig shortest;
    shortest.weight = 0.0;
    const auto a = amr::planForLocalization(inflated, start, goal, u, shortest);
    const auto reference = amr::planAStar(inflated, start, goal, amr::Connectivity::Eight);
    REQUIRE(a.success() && reference.success());
    CHECK(amr::isValidGridPath(inflated, a.path, amr::Connectivity::Eight));
    CHECK_NEAR(amr::gridPathCost(a.path).cells(), reference.cost().cells(), 1e-9);  // weight 0: shortest
    amr::LocalizationPlanConfig aware;
    aware.weight = 1.0;
    const auto b = amr::planForLocalization(inflated, start, goal, u, aware);
    REQUIRE(b.success());
    CHECK(amr::isValidGridPath(inflated, b.path, amr::Connectivity::Eight));
    const auto blind = [&](const std::vector<amr::GridCoord>& path) {
        double length = 0.0;
        for (std::size_t k = 1; k < path.size(); ++k) {
            if (u[static_cast<std::size_t>(path[k].y * map.width() + path[k].x)] > 0.5) {
                length += amr::pathLengthMeters(map, {path[k - 1], path[k]});
            }
        }
        return length;
    };
    std::cout << "    hall: shortest " << amr::pathLengthMeters(map, a.path) << " m (" << blind(a.path)
              << " m blind), aware " << amr::pathLengthMeters(map, b.path) << " m (" << blind(b.path) << " m blind)\n";
    CHECK(blind(b.path) < 0.25 * blind(a.path));
    CHECK(amr::pathLengthMeters(map, b.path) > amr::pathLengthMeters(map, a.path));
    CHECK(amr::localizationPenalty(0.05, aware) == 0.0);
    CHECK_NEAR(amr::localizationPenalty(0.2, aware), 2.0, 1e-12);
    CHECK_EQ(amr::localizationPenalty(1e9, aware), aware.max_penalty);
}

// ---------------------------------------------------------------------------------
// Steering by the estimate

amr::ClosedLoopConfig closedLoop(int beams, double odometry_scale, std::uint32_t seed) {
    amr::ClosedLoopConfig c;
    c.motion.model = amr::RobotModel::DiffDrive;
    c.motion.robot_radius = 0.2;
    c.lidar.beams = beams;
    c.lidar.noise_stddev = 0.02;
    c.lidar.seed = seed;
    c.ekf.enabled = true;
    c.ekf.range_sigma = 0.02;
    c.ekf.odometry.seed = seed;
    c.ekf.odometry.distance *= odometry_scale;
    c.ekf.odometry.turn *= odometry_scale;
    c.ekf.odometry.drift *= odometry_scale;
    if (odometry_scale == 0.0) {
        c.ekf.odometry.distance = c.ekf.odometry.turn = c.ekf.odometry.drift = 1e-12;
        c.ekf.initial_position_sigma = c.ekf.initial_heading_sigma = 1e-9;
    }
    return c;
}

void testSteeringByTheEstimate() {
    const GridMap map = load("obstacles.csv");
    const GridMap inflated = amr::inflateObstacles(map, 0.2);
    const auto plan = amr::planWithFootprint(map, inflated, {1, 1}, {18, 18});
    REQUIRE(plan.success());
    // Exact odometry: the estimate is the truth, the robot follows the plan.
    const auto exact = amr::simulateWithEstimate(map, plan.path, closedLoop(0, 0.0, 1));
    CHECK(exact.status == amr::NavigationStatus::GoalReached);
    CHECK(exact.final_goal_error < 0.06);
    CHECK(exact.max_error < 0.01);
    // With the LiDAR the EKF keeps the true robot on track.
    const auto ekf = amr::simulateWithEstimate(map, plan.path, closedLoop(72, 1.0, 2));
    CHECK(ekf.status == amr::NavigationStatus::GoalReached);
    CHECK(ekf.final_goal_error < 0.08);
    CHECK(ekf.max_error < 0.05);
    CHECK_EQ(ekf.localization.size(), ekf.trajectory.size());
    // Dead reckoning: the true robot ends far from where it believes it is.
    const auto blind = amr::simulateWithEstimate(map, plan.path, closedLoop(0, 1.0, 3));
    CHECK(blind.max_error > 0.2);
    CHECK(blind.status == amr::NavigationStatus::Collision || blind.final_goal_error > 0.2);
    // A collision ends the run where the true disc touched.
    const auto worst = amr::simulateWithEstimate(map, plan.path, closedLoop(0, 6.0, 4));
    if (worst.status == amr::NavigationStatus::Collision) {
        const auto& last = worst.trajectory.back().state;
        CHECK(amr::obstacleDistance(map, {last.x, last.y}, 2.0) < 0.2 + 1e-6);
    }
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Particle filter: global localization", testGlobalLocalization},
        {"Particle filter: the kidnapped robot", testKidnappedRobot},
        {"Particle filter: a symmetric room keeps both places", testSymmetricRoomKeepsBothPlaces},
        {"Particle filter: one difference decides", testOneDifferenceDecides},
        {"Localizability", testLocalizability},
        {"Localization-aware planning", testLocalizationAwarePlanning},
        {"Steering by the estimate", testSteeringByTheEstimate},
    });
}
