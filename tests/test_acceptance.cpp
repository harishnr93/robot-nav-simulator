// Acceptance tests with confidence bounds: the claims of the README and of
// docs/algorithms.md, checked on independent runs and stated as what the runs show at
// 95 % confidence (statistics.hpp), not as point estimates.
//
//   A1  moving obstacles, default robot: success rate (Clopper-Pearson lower bound)
//   A2  the defaults against the previous settings, on the same scenarios (McNemar)
//   A3  several robots: rate of runs with robot-robot contact (upper bound)
//   A4  EKF consistency: the covariance scale over independent runs, within 35 %
//   A5  particle filter: global localization success rate (lower bound)
//
// The scenario seeds are fixed, so the tests are deterministic; the bounds say what
// the fixed sample supports. Runs are spread over the machine's cores.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "astar.hpp"
#include "ekf.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "multi_robot.hpp"
#include "particle_filter.hpp"
#include "scenario_search.hpp"
#include "simulator.hpp"
#include "statistics.hpp"
#include "test_framework.hpp"

using amr::GridMap;

namespace {

constexpr double kPi = 3.14159265358979323846;

GridMap load(const std::string& name) {
    const auto loaded = amr::loadGridMapCsv(std::string(AMR_SOURCE_DIR) + "/maps/" + name, 0.5);
    return loaded.ok ? loaded.map : GridMap(1, 1, 0.5, {GridMap::kFree});
}

/// Runs job(i) for i in [0, n) on all cores.
void parallelFor(std::size_t n, const std::function<void(std::size_t)>& job) {
    std::atomic<std::size_t> next{0};
    std::vector<std::thread> threads;
    const unsigned count = std::max(1u, std::thread::hardware_concurrency());
    for (unsigned t = 0; t < count; ++t) {
        threads.emplace_back([&]() {
            for (std::size_t i = next++; i < n; i = next++) {
                job(i);
            }
        });
    }
    for (std::thread& t : threads) {
        t.join();
    }
}

void report(const std::string& what, std::size_t k, std::size_t n, double bound, const char* kind) {
    std::cout << "    " << what << ": " << k << "/" << n << ", " << kind << " bound at 95 %: " << bound << "\n";
}

// ---------------------------------------------------------------------------------
// A1, A2: moving obstacles

struct BenchmarkRuns {
    std::vector<bool> reached;
};

BenchmarkRuns runBenchmark(const GridMap& map, amr::RobotModel model, const amr::ReplanningConfig& replanning,
                           std::size_t runs, bool turning_hypotheses = true) {
    const amr::GridCoord start{1, 1}, goal{18, 18};
    const auto path = amr::planAStar(map, start, goal).path;
    amr::ScenarioSearchConfig generator;
    generator.start = start;
    generator.goal = goal;
    std::mt19937 rng(2024);
    std::vector<amr::SearchScenario> scenarios;
    for (std::size_t i = 0; i < runs; ++i) {
        scenarios.push_back(amr::randomScenario(map, generator, rng));
    }
    BenchmarkRuns out;
    out.reached.assign(runs, false);
    std::vector<char> reached(runs, 0);
    parallelFor(runs, [&](std::size_t i) {
        amr::DynamicScenario scenario;
        scenario.planning_map = &map;
        scenario.replanning = replanning;
        scenario.perception.turning_hypotheses = turning_hypotheses;
        scenario.obstacles = amr::toObstacles(map, scenarios[i], start, goal);
        scenario.perception.lidar.seed = static_cast<std::uint32_t>(i);
        amr::MotionConfig motion;
        motion.model = model;
        motion.max_time = 600.0;
        reached[i] = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario).success() ? 1 : 0;
    });
    for (std::size_t i = 0; i < runs; ++i) {
        out.reached[i] = reached[i] != 0;
    }
    return out;
}

std::size_t successes(const BenchmarkRuns& r) {
    return static_cast<std::size_t>(std::count(r.reached.begin(), r.reached.end(), true));
}

/// A1: with the defaults, the robot reaches its goal among three moving obstacles in
/// at least 98 % (point) and 96 % (differential drive) of scenarios, at 95 %
/// confidence (300 scenarios). A2: on the same scenarios, the previous settings
/// (margin 0.25 m, look-ahead 2 s, no livelock switch, no turning hypotheses) fail
/// significantly more often:
/// McNemar's exact test on the runs where only one of them failed, p < 0.05. The
/// difference is about 1 % of runs, so this needs 1000 paired scenarios (with 300 it
/// is not significant).
void testMovingObstacles() {
    const GridMap map = load("obstacles.csv");
    constexpr std::size_t kRuns = 1000, kRateRuns = 300;
    amr::ReplanningConfig previous;
    previous.margin = 0.25;
    previous.prediction_horizon = 2.0;
    previous.livelock_window = 0.0;
    for (const auto model : {amr::RobotModel::Point, amr::RobotModel::DiffDrive}) {
        const std::string name = model == amr::RobotModel::Point ? "point" : "diff-drive";
        const BenchmarkRuns now = runBenchmark(map, model, amr::ReplanningConfig{}, kRuns);
        const BenchmarkRuns before = runBenchmark(map, model, previous, kRuns, false);
        BenchmarkRuns first = now;
        first.reached.resize(kRateRuns);
        const std::size_t k = successes(first);
        const double lower = amr::rateLowerBound(k, kRateRuns);
        report(name + ", defaults, success", k, kRateRuns, lower, "lower");
        CHECK(lower >= (model == amr::RobotModel::Point ? 0.98 : 0.96));
        report(name + ", defaults, success", successes(now), kRuns, amr::rateLowerBound(successes(now), kRuns),
               "lower");
        std::size_t only_before_failed = 0, only_now_failed = 0;
        for (std::size_t i = 0; i < kRuns; ++i) {
            only_before_failed += !before.reached[i] && now.reached[i] ? 1 : 0;
            only_now_failed += before.reached[i] && !now.reached[i] ? 1 : 0;
        }
        const double p = amr::mcnemarExact(only_before_failed, only_now_failed);
        std::cout << "    " << name << ", previous settings: " << successes(before) << "/" << kRuns
                  << "; fixed " << only_before_failed << ", broken " << only_now_failed << ", McNemar p = " << p
                  << "\n";
        CHECK(only_before_failed > only_now_failed);
        CHECK(p < 0.05);
    }
}

// ---------------------------------------------------------------------------------
// A3: several robots

std::vector<amr::RobotTask> randomTasks(const GridMap& planning, int count, std::mt19937& rng) {
    std::uniform_int_distribution<int> xs(0, planning.width() - 1), ys(0, planning.height() - 1);
    const auto far = [](amr::GridCoord a, amr::GridCoord b, double cells) {
        return std::hypot(a.x - b.x, a.y - b.y) >= cells;
    };
    for (;;) {
        std::vector<amr::RobotTask> tasks;
        int attempts = 0;
        while (static_cast<int>(tasks.size()) < count && attempts++ < 1000) {
            amr::RobotTask t{{xs(rng), ys(rng)}, {xs(rng), ys(rng)}, 0.0};
            if (!planning.isFree(t.start) || !planning.isFree(t.goal) || !far(t.start, t.goal, 8.0)) {
                continue;
            }
            bool ok = true;
            for (const amr::RobotTask& o : tasks) {
                ok = ok && far(t.start, o.start, 2.0) && far(t.goal, o.goal, 2.0) && far(t.start, o.goal, 2.0) &&
                     far(t.goal, o.start, 2.0);
            }
            if (ok && amr::planAStar(planning, t.start, t.goal).success()) {
                tasks.push_back(t);
            }
        }
        if (static_cast<int>(tasks.size()) == count) {
            return tasks;
        }
    }
}

/// A3: four robots that all replan never touch: the rate of runs with contact is
/// at most 5 % at 95 % confidence (60 runs per model without contact give 4.9 %).
void testRobotsDoNotTouch() {
    const GridMap map = load("obstacles.csv");
    const GridMap planning = amr::inflateObstacles(map, 0.2);
    constexpr std::size_t kRuns = 60;
    std::mt19937 rng(11);
    std::vector<std::vector<amr::RobotTask>> tasks;
    for (std::size_t i = 0; i < kRuns; ++i) {
        tasks.push_back(randomTasks(planning, 4, rng));
    }
    for (const auto model : {amr::RobotModel::Point, amr::RobotModel::DiffDrive}) {
        std::vector<char> contact(kRuns, 0);
        parallelFor(kRuns, [&](std::size_t i) {
            amr::MultiRobotConfig config;
            config.motion.model = model;
            config.motion.robot_radius = 0.2;
            config.motion.max_time = 300.0;
            config.scenario.planning_map = &planning;
            config.scenario.perception.lidar.seed = static_cast<std::uint32_t>(i);
            contact[i] = amr::simulateRobots(map, tasks[i], config).robot_collisions > 0 ? 1 : 0;
        });
        const auto k = static_cast<std::size_t>(std::count(contact.begin(), contact.end(), 1));
        const double upper = amr::rateUpperBound(k, kRuns);
        report(std::string(model == amr::RobotModel::Point ? "point" : "diff-drive") + ", runs with contact", k,
               kRuns, upper, "upper");
        CHECK(upper <= 0.05);
    }
}

// ---------------------------------------------------------------------------------
// A4: EKF consistency

/// A4: the EKF's covariance is honest to within 35 %. If its errors have s times the
/// covariance it claims, the NEES of a run is s chi-square(3), and the sum over 100
/// independent runs (own odometry and LiDAR noise each) at one time step is
/// s chi-square(300): that gives a confidence interval for the scale s at each time
/// step. Checked every second along the route, prediction only and with a 5 cm-noise
/// LiDAR whose noise the filter knows, with Bonferroni-corrected intervals (95 % for
/// all checks together). The claim "s within [1/1.35, 1.35]" is rejected if any
/// interval lies entirely outside it. Strict consistency (s = 1) is rejected at some
/// steps: the EKF's linearization makes it a little overconfident once the heading is
/// uncertain.
void testEkfConsistency() {
    const GridMap map = load("obstacles.csv");
    const auto plan = amr::planAStar(map, {1, 1}, {18, 18}, amr::Connectivity::Eight);
    amr::MotionConfig motion;
    motion.model = amr::RobotModel::DiffDrive;
    motion.initial_heading = kPi / 4;
    const auto trajectory = amr::simulatePath(map, plan.path, motion, amr::Connectivity::Eight).trajectory;
    constexpr std::size_t kRuns = 100;
    constexpr double kTolerance = 1.35;
    for (const bool lidar_on : {false, true}) {
        std::vector<std::vector<double>> nees(kRuns);
        parallelFor(kRuns, [&](std::size_t run) {
            amr::LidarConfig lidar;
            lidar.beams = lidar_on ? 36 : 0;
            lidar.noise_stddev = 0.05;
            lidar.seed = static_cast<std::uint32_t>(run);
            const auto scans = amr::scanTrajectory(map, trajectory, lidar);
            amr::EkfConfig config;
            config.enabled = true;
            config.range_sigma = 0.05;
            config.odometry.seed = static_cast<std::uint32_t>(5000 + run);
            for (const auto& s : amr::runLocalization(map, trajectory, scans, lidar, config)) {
                nees[run].push_back(s.nees);
            }
        });
        std::vector<double> sums;
        for (std::size_t k = 10; k < nees.front().size(); k += 10) {
            double sum = 0.0;
            for (const auto& run : nees) {
                sum += run[k];
            }
            sums.push_back(sum);
        }
        const double alpha = 0.05 / static_cast<double>(sums.size());  // Bonferroni
        const double dof = 3.0 * static_cast<double>(kRuns);
        const double chi_lo = amr::chiSquareQuantile(alpha / 2.0, dof);
        const double chi_hi = amr::chiSquareQuantile(1.0 - alpha / 2.0, dof);
        std::size_t rejected_strict = 0, rejected_claim = 0;
        double scale_low = 1e9, scale_high = 0.0, lowest_upper = 1e9, highest_lower = 0.0;
        for (const double sum : sums) {
            const double lower = sum / chi_hi, upper = sum / chi_lo;  // interval for s
            scale_low = std::min(scale_low, sum / dof);
            scale_high = std::max(scale_high, sum / dof);
            highest_lower = std::max(highest_lower, lower);
            lowest_upper = std::min(lowest_upper, upper);
            rejected_strict += lower > 1.0 || upper < 1.0 ? 1 : 0;
            rejected_claim += lower > kTolerance || upper < 1.0 / kTolerance ? 1 : 0;
        }
        std::cout << "    " << (lidar_on ? "with LiDAR" : "prediction only") << ": covariance scale " << scale_low
                  << "-" << scale_high << " over " << sums.size() << " checks (" << kRuns
                  << " runs each); highest lower bound " << highest_lower << ", lowest upper bound " << lowest_upper
                  << "; s = 1 rejected at " << rejected_strict << ", s within 1.35x rejected at " << rejected_claim
                  << "\n";
        CHECK_EQ(rejected_claim, 0u);
    }
}

// ---------------------------------------------------------------------------------
// A5: particle filter

/// A5: the particle filter finds the robot from no prior knowledge: within 15 cm
/// after 1 s and to the end, on 20 routes with their own odometry noise. 20 of 20
/// show a success rate of at least 86 % at 95 % confidence.
void testGlobalLocalization() {
    const GridMap map = load("obstacles.csv");
    const std::vector<std::pair<amr::GridCoord, amr::GridCoord>> routes{
        {{1, 1}, {18, 18}}, {{18, 1}, {1, 18}}, {{1, 18}, {18, 9}}, {{9, 1}, {9, 18}}, {{18, 18}, {1, 9}}};
    constexpr std::size_t kRuns = 20;
    std::vector<char> found(kRuns, 0);
    parallelFor(kRuns, [&](std::size_t i) {
        const auto& [from, to] = routes[i % routes.size()];
        const auto plan = amr::planAStar(map, from, to);
        if (!plan.success()) {
            return;
        }
        amr::MotionConfig motion;
        motion.model = amr::RobotModel::DiffDrive;
        const auto trajectory = amr::simulatePath(map, plan.path, motion).trajectory;
        amr::LidarConfig lidar;
        lidar.beams = 72;
        lidar.seed = static_cast<std::uint32_t>(i);
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        amr::OdometryNoise odometry;
        odometry.seed = static_cast<std::uint32_t>(100 + i);
        const auto samples = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, {}, true);
        bool ok = samples.size() > 10;
        for (std::size_t k = 10; k < samples.size() && ok; ++k) {
            ok = std::hypot(samples[k].truth.x - samples[k].estimate.pose.x,
                            samples[k].truth.y - samples[k].estimate.pose.y) < 0.15;
        }
        found[i] = ok ? 1 : 0;
    });
    const auto k = static_cast<std::size_t>(std::count(found.begin(), found.end(), 1));
    const double lower = amr::rateLowerBound(k, kRuns);
    report("global localization", k, kRuns, lower, "lower");
    CHECK(lower >= 0.86);
}

}  // namespace

int main() {
    return testfw::runAll({
        {"A1/A2 moving obstacles: success rate and the gain over the previous settings", testMovingObstacles},
        {"A3 several robots: no contact", testRobotsDoNotTouch},
        {"A4 EKF consistency: covariance honest to within 35 %", testEkfConsistency},
        {"A5 particle filter: global localization", testGlobalLocalization},
    });
}
