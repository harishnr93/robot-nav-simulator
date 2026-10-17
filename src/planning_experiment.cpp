// planning_experiment: planning with moving obstacles.
//
//   planning_experiment turns [options]      turn-aware vs. shortest paths for the
//                                            differential drive (static maps)
//   planning_experiment spacetime [options]  reactive vs. space-time planning (and
//                                            the livelock switch) among moving obstacles
//   planning_experiment margins [options]    margin x prediction horizon sweep
//   planning_experiment robots [options]     2-6 robots that all replan (--map)
//
// Options: --runs N, --seed N, --map FILE, --connectivity 4|8, --speeds MIN,MAX
//          (obstacle speeds, m/s), --livelock S (margins: livelock window of every
//          variant), --output DIR (default results/planning).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "dynamic_obstacles.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "multi_robot.hpp"
#include "simulator.hpp"
#include "spacetime_planner.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Options {
    std::string mode;
    int runs = 200;
    unsigned seed = 1;
    std::string map = "maps/obstacles.csv";
    int connectivity = 4;
    double min_speed = 0.2, max_speed = 0.5;
    double livelock = 15.0;  ///< margins: livelock window of every variant, s (0: off)
    std::filesystem::path output = "results/planning";
};

int countTurns(const std::vector<amr::GridCoord>& path) {
    int turns = 0;
    for (std::size_t k = 2; k < path.size(); ++k) {
        const int ax = path[k - 1].x - path[k - 2].x, ay = path[k - 1].y - path[k - 2].y;
        const int bx = path[k].x - path[k - 1].x, by = path[k].y - path[k - 1].y;
        turns += (ax != bx || ay != by) ? 1 : 0;
    }
    return turns;
}

double median(std::vector<double> v) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

// ---------------------------------------------------------------------------------
// Turn-aware planning

int modeTurns(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(o.map, 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << "\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    const auto connectivity = o.connectivity == 8 ? amr::Connectivity::Eight : amr::Connectivity::Four;
    const double radius = 0.0;
    const amr::GridMap inflated = amr::inflateObstacles(map, radius);
    std::mt19937 rng(o.seed);
    std::uniform_int_distribution<int> xs(0, map.width() - 1), ys(0, map.height() - 1);
    struct Row {
        double length = 0, turns = 0, time = 0, planned = 0, rotation = 0;
        int reached = 0;
        std::vector<double> gains;
    };
    Row shortest, aware;
    int pairs = 0;
    for (int attempt = 0; pairs < o.runs && attempt < 100 * o.runs; ++attempt) {
        const amr::GridCoord start{xs(rng), ys(rng)}, goal{xs(rng), ys(rng)};
        if (!inflated.isFree(start) || !inflated.isFree(goal) || start == goal) {
            continue;
        }
        const double heading = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
        const amr::PlanResult a = amr::planWithFootprint(map, inflated, start, goal, connectivity);
        amr::SpaceTimeConfig sc;
        sc.connectivity = connectivity;
        sc.turn_aware = true;
        amr::MotionConfig motion;
        motion.model = amr::RobotModel::DiffDrive;
        motion.robot_radius = radius;
        motion.initial_heading = heading;
        sc.speed = motion.speed;
        sc.dt = motion.dt;
        sc.max_angular_speed = motion.max_angular_speed;
        const amr::SpaceTimePlan b = amr::planSpaceTime(inflated, start, heading, goal, {}, sc);
        if (!a.success() || !b.success) {
            continue;
        }
        ++pairs;
        const auto path_b = amr::planCells(b);
        const auto sim_a = amr::simulatePath(map, a.path, motion, connectivity);
        const auto sim_b = amr::simulatePath(map, path_b, motion, connectivity);
        for (auto* row : {&shortest, &aware}) {
            const bool is_a = row == &shortest;
            const auto& path = is_a ? a.path : path_b;
            const auto& sim = is_a ? sim_a : sim_b;
            row->length += amr::pathLengthMeters(map, path);
            row->turns += countTurns(path);
            row->time += sim.travelTime();
            row->rotation += sim.total_rotation;
            row->reached += sim.success() ? 1 : 0;
        }
        aware.planned += b.arrival;
        aware.gains.push_back(sim_a.travelTime() - sim_b.travelTime());
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "planning_turns.csv");
    csv << "planner,pairs,mean_length_m,mean_turns,mean_travel_s,mean_rotation_rad,reached\n";
    std::cout << "Turn-aware planning (" << o.map << ", " << o.connectivity << "-connected, differential drive, "
              << pairs << " random routes)\n"
              << std::left << std::setw(16) << "planner" << std::right << std::setw(11) << "length m"
              << std::setw(9) << "turns" << std::setw(12) << "rotation" << std::setw(12) << "travel s"
              << std::setw(10) << "reached" << "\n";
    for (const auto* row : {&shortest, &aware}) {
        const double n = std::max(1, pairs);
        const std::string name = row == &shortest ? "shortest" : "turn-aware";
        std::cout << std::left << std::setw(16) << name << std::right << std::fixed << std::setprecision(2)
                  << std::setw(11) << row->length / n << std::setw(9) << std::setprecision(1) << row->turns / n
                  << std::setw(10) << std::setprecision(2) << row->rotation / n << " rad" << std::setw(10)
                  << row->time / n << " s" << std::setw(9) << std::setprecision(1) << 100.0 * row->reached / n << "%\n";
        csv << name << ',' << pairs << ',' << row->length / n << ',' << row->turns / n << ',' << row->time / n << ','
            << row->rotation / n << ',' << row->reached << "\n";
    }
    std::vector<double> gains = aware.gains;
    std::sort(gains.begin(), gains.end());
    const double n = std::max(1, pairs);
    std::cout << "time saved per route: median " << std::setprecision(2) << median(gains) << " s, best "
              << (gains.empty() ? 0.0 : gains.back()) << " s, worst " << (gains.empty() ? 0.0 : gains.front())
              << " s; planned arrival vs simulated: " << aware.planned / n << " s vs " << aware.time / n << " s\n";
    return 0;
}

// ---------------------------------------------------------------------------------
// Space-time planning against the moving-obstacle benchmark

/// The benchmark's scenarios (perception_experiment dynamic): 3 obstacles patrolling
/// random straight lanes of 2-8 cells, radius 0.25-0.35 m, clear of start and goal.
std::vector<amr::DynamicObstacle> randomObstacles(const amr::GridMap& map, amr::GridCoord start,
                                                  amr::GridCoord goal, std::mt19937& rng, double min_speed,
                                                  double max_speed) {
    std::vector<amr::DynamicObstacle> obstacles;
    std::uniform_int_distribution<int> xs(0, map.width() - 1);
    std::uniform_int_distribution<int> ys(0, map.height() - 1);
    std::uniform_int_distribution<int> length(2, 8);
    std::uniform_real_distribution<double> speed(min_speed, max_speed);
    std::uniform_real_distribution<double> radius(0.25, 0.35);
    while (obstacles.size() < 3) {
        const amr::GridCoord a{xs(rng), ys(rng)};
        const bool horizontal = rng() % 2 == 0;
        int cells = length(rng);
        amr::GridCoord b = a;
        while (cells-- > 0) {
            const amr::GridCoord next{b.x + (horizontal ? 1 : 0), b.y + (horizontal ? 0 : 1)};
            if (!map.isFree(next) || next == goal || next == start) {
                break;
            }
            b = next;
        }
        amr::DynamicObstacle o;
        o.route = {map.gridToWorld(a), map.gridToWorld(b)};
        o.speed = speed(rng);
        o.radius = radius(rng);
        if (b == a || !map.isFree(a) || a == start || a == goal || !amr::validateDynamicObstacle(map, o).empty()) {
            continue;
        }
        obstacles.push_back(o);
    }
    return obstacles;
}

struct Variant {
    std::string name;
    amr::DynamicScenario scenario;
};

struct Score {
    int runs = 0, reached = 0, collisions = 0, timeouts = 0, rescued = 0, lost = 0, switches = 0;
    int walls = 0;  ///< collisions with a wall (none expected)
    double travel = 0.0, waits = 0.0, plan_ms = 0.0, plans = 0.0;
    double min_clearance = std::numeric_limits<double>::infinity();
};

/// One benchmark scenario: the obstacles and the LiDAR noise seed.
struct Scenario {
    std::vector<amr::DynamicObstacle> obstacles;
    std::uint32_t noise_seed = 0;
};

std::vector<Scenario> makeScenarios(const amr::GridMap& map, amr::GridCoord start, amr::GridCoord goal,
                                    const Options& o) {
    std::mt19937 rng(o.seed);
    std::vector<Scenario> scenarios;
    for (int run = 0; run < o.runs; ++run) {
        Scenario s;
        s.obstacles = randomObstacles(map, start, goal, rng, o.min_speed, o.max_speed);
        s.noise_seed = static_cast<std::uint32_t>(rng());
        scenarios.push_back(std::move(s));
    }
    return scenarios;
}

/// Runs every variant on every scenario with both robot models (in parallel; the
/// results do not depend on the thread count). scores[model][variant]; a variant
/// "rescues" a run the first variant failed and "loses" one it reached.
std::vector<std::vector<Score>> evaluate(const amr::GridMap& map, const std::vector<amr::GridCoord>& path,
                                         const std::vector<Variant>& variants,
                                         const std::vector<Scenario>& scenarios) {
    const std::size_t nv = variants.size();
    const std::size_t jobs = scenarios.size() * 2 * nv;
    std::vector<amr::SimulationResult> results(jobs);
    std::atomic<std::size_t> next{0};
    const auto worker = [&]() {
        for (std::size_t job = next++; job < jobs; job = next++) {
            const std::size_t v = job % nv, model = (job / nv) % 2, run = job / nv / 2;
            amr::MotionConfig motion;
            motion.model = model == 0 ? amr::RobotModel::Point : amr::RobotModel::DiffDrive;
            motion.max_time = 600.0;
            amr::DynamicScenario scenario = variants[v].scenario;
            scenario.planning_map = &map;
            scenario.obstacles = scenarios[run].obstacles;
            scenario.perception.lidar.seed = scenarios[run].noise_seed;
            amr::SimulationResult sim = amr::simulatePath(map, path, motion, amr::Connectivity::Four, &scenario);
            sim.trajectory.clear();  // keep memory flat; the scores need the summary only
            sim.trajectory.shrink_to_fit();
            results[job] = std::move(sim);
        }
    };
    std::vector<std::thread> threads;
    const unsigned count = std::max(1u, std::thread::hardware_concurrency());
    for (unsigned t = 0; t < count; ++t) {
        threads.emplace_back(worker);
    }
    for (std::thread& t : threads) {
        t.join();
    }
    std::vector<std::vector<Score>> scores(2, std::vector<Score>(nv));
    for (std::size_t run = 0; run < scenarios.size(); ++run) {
        for (std::size_t model = 0; model < 2; ++model) {
            const bool first_ok = results[(run * 2 + model) * nv].success();
            for (std::size_t v = 0; v < nv; ++v) {
                const amr::SimulationResult& sim = results[(run * 2 + model) * nv + v];
                Score& s = scores[model][v];
                const bool ok = sim.success();
                ++s.runs;
                s.reached += ok ? 1 : 0;
                s.collisions += sim.status == amr::NavigationStatus::Collision ? 1 : 0;
                s.walls += sim.status == amr::NavigationStatus::Collision && sim.collided_obstacle < 0 ? 1 : 0;
                s.timeouts += sim.status == amr::NavigationStatus::Timeout ? 1 : 0;
                s.travel += ok ? static_cast<double>(sim.steps) * 0.1 : 0.0;
                s.waits += static_cast<double>(sim.wait_steps) * 0.1;
                s.rescued += (!first_ok && ok) ? 1 : 0;
                s.lost += (first_ok && !ok) ? 1 : 0;
                s.switches += sim.strategy_switches;
                if (sim.status != amr::NavigationStatus::Collision) {
                    s.min_clearance = std::min(s.min_clearance, sim.min_dynamic_clearance);
                }
                for (const auto& e : sim.replans) {
                    s.plan_ms += e.planning_time_ms;
                    s.plans += 1.0;
                }
            }
        }
    }
    return scores;
}

std::vector<Variant> plannerVariants() {
    std::vector<Variant> v;
    // The comparison base: the reactive robot as it was (LiDAR perception, learned
    // reversals, margin 0.25 m, look-ahead 2 s, no livelock switch).
    amr::DynamicScenario previous;
    previous.replanning.margin = 0.25;
    previous.replanning.prediction_horizon = 2.0;
    previous.replanning.livelock_window = 0.0;
    v.push_back({"reactive, previous settings", previous});
    amr::DynamicScenario reactive;  // the defaults: margin 0.15 m, 1 s, switch after 15 s
    reactive.replanning.livelock_window = 0.0;
    v.push_back({"reactive, no switch", reactive});
    reactive.replanning.livelock_window = amr::ReplanningConfig{}.livelock_window;
    v.push_back({"reactive + livelock switch (default)", reactive});
    amr::DynamicScenario spacetime;
    spacetime.replanning.planner = amr::DynamicPlanner::SpaceTime;
    spacetime.replanning.livelock_window = 0.0;
    v.push_back({"space-time, no switch", spacetime});
    amr::DynamicScenario perfect = spacetime;
    perfect.replanning.perfect_prediction = true;
    spacetime.replanning.livelock_window = amr::ReplanningConfig{}.livelock_window;
    v.push_back({"space-time + livelock switch", spacetime});
    v.push_back({"space-time, perfect prediction", perfect});
    amr::DynamicScenario oracle;
    oracle.perception.mode = amr::ObstaclePerception::Oracle;
    v.push_back({"reactive + switch, oracle sightings", oracle});
    return v;
}

void printScores(const std::vector<Variant>& variants, const std::vector<std::vector<Score>>& scores,
                 std::ostream& csv) {
    csv << "robot,variant,runs,reached,collisions,timeouts,mean_travel_s,mean_wait_s,"
           "base_failures_solved,base_successes_failed,strategy_switches,min_clearance_m,ms_per_plan\n";
    for (int model = 0; model < 2; ++model) {
        const std::string robot = model == 0 ? "point" : "diff-drive";
        std::cout << "\n" << robot << " robot\n"
                  << std::left << std::setw(38) << "variant" << std::right << std::setw(9) << "reached"
                  << std::setw(11) << "collisions" << std::setw(10) << "timeouts" << std::setw(11) << "travel s"
                  << std::setw(9) << "wait s" << std::setw(18) << "fixes / breaks" << std::setw(10) << "switches"
                  << std::setw(12) << "ms/plan" << "\n";
        for (std::size_t v = 0; v < variants.size(); ++v) {
            const Score& s = scores[static_cast<std::size_t>(model)][v];
            const double n = std::max(1, s.runs);
            std::cout << std::left << std::setw(38) << variants[v].name << std::right << std::fixed
                      << std::setprecision(1) << std::setw(8) << 100.0 * s.reached / n << "%" << std::setw(11)
                      << s.collisions << std::setw(10) << s.timeouts << std::setw(11)
                      << (s.reached > 0 ? s.travel / s.reached : 0.0) << std::setw(9) << s.waits / n
                      << std::setw(10) << s.rescued << " / " << std::setw(4) << s.lost << std::setw(10)
                      << s.switches << std::setw(12) << std::setprecision(2)
                      << (s.plans > 0 ? s.plan_ms / s.plans : 0.0) << "\n";
            if (s.walls > 0) {
                std::cout << "    (" << s.walls << " of the collisions with a wall)\n";
            }
            csv << robot << ",\"" << variants[v].name << "\"," << s.runs << ',' << s.reached << ',' << s.collisions
                << ',' << s.timeouts << ',' << (s.reached > 0 ? s.travel / s.reached : 0.0) << ',' << s.waits / n
                << ',' << s.rescued << ',' << s.lost << ',' << s.switches << ',' << s.min_clearance << ','
                << (s.plans > 0 ? s.plan_ms / s.plans : 0.0) << "\n";
        }
    }
}

const amr::GridCoord kStart{1, 1}, kGoal{18, 18};

int modeSpaceTime(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv("maps/obstacles.csv", 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << " (run from the repository root)\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    const std::vector<Variant> variants = plannerVariants();
    const amr::PlanResult plan = amr::planAStar(map, kStart, kGoal);
    const auto scores = evaluate(map, plan.path, variants, makeScenarios(map, kStart, kGoal, o));
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "planning_spacetime.csv");
    std::cout << "Space-time planning (" << o.runs << " scenarios, 3 obstacles each at " << o.min_speed << "-"
              << o.max_speed << " m/s; fixes / breaks = failures of the previous settings solved / their "
              << "successes lost; switches = livelocks detected)\n";
    printScores(variants, scores, csv);
    return 0;
}

/// Item 16: the smallest safe margin and prediction horizon, per planner.
int modeMargins(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv("maps/obstacles.csv", 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << " (run from the repository root)\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    const amr::PlanResult plan = amr::planAStar(map, kStart, kGoal);
    const auto scenarios = makeScenarios(map, kStart, kGoal, o);
    std::vector<Variant> variants;
    const std::vector<double> margins{0.0, 0.05, 0.1, 0.15, 0.25, 0.35};
    for (const double margin : margins) {
        for (const double horizon : {0.5, 1.0, 2.0, 3.0}) {
            amr::DynamicScenario s;
            s.replanning.margin = margin;
            s.replanning.prediction_horizon = horizon;
            s.replanning.livelock_window = o.livelock;
            std::ostringstream name;
            name << "reactive m " << margin << " h " << horizon;
            variants.push_back({name.str(), s});
        }
        for (const double horizon : {2.0, 5.0, 10.0, 20.0}) {
            amr::DynamicScenario s;
            s.replanning.planner = amr::DynamicPlanner::SpaceTime;
            s.replanning.margin = margin;
            s.replanning.spacetime_horizon = horizon;
            s.replanning.livelock_window = o.livelock;
            std::ostringstream name;
            name << "space-time m " << margin << " H " << horizon;
            variants.push_back({name.str(), s});
        }
    }
    const auto scores = evaluate(map, plan.path, variants, scenarios);
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "planning_margins.csv");
    std::cout << "Margin x prediction horizon (" << o.runs << " scenarios at " << o.min_speed << "-" << o.max_speed
              << " m/s; h = look-ahead gate horizon, H = space-time planning horizon; min gap = closest "
              << "approach to an obstacle edge over all runs without collision)\n";
    printScores(variants, scores, csv);
    return 0;
}

/// Item 17: robots that all replan, with each other as the only moving obstacles.
std::vector<amr::RobotTask> randomTasks(const amr::GridMap& planning, int count, std::mt19937& rng) {
    std::uniform_int_distribution<int> xs(0, planning.width() - 1), ys(0, planning.height() - 1);
    std::uniform_real_distribution<double> heading(-kPi, kPi);
    const auto far = [&](amr::GridCoord a, amr::GridCoord b, double cells) {
        return std::hypot(a.x - b.x, a.y - b.y) >= cells;
    };
    for (;;) {
        std::vector<amr::RobotTask> tasks;
        int attempts = 0;
        while (static_cast<int>(tasks.size()) < count && attempts++ < 1000) {
            amr::RobotTask t{{xs(rng), ys(rng)}, {xs(rng), ys(rng)}, heading(rng)};
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

int modeRobots(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(o.map, 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << " (run from the repository root)\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    constexpr double kRadius = 0.2;
    const amr::GridMap planning = amr::inflateObstacles(map, kRadius);
    struct RobotVariant {
        std::string name;
        amr::DynamicPlanner planner;
        double livelock_window;
        bool stagger;
    };
    const std::vector<RobotVariant> variants{
        {"reactive, no switch", amr::DynamicPlanner::Reactive, 0.0, true},
        {"reactive + switch, not staggered", amr::DynamicPlanner::Reactive, 15.0, false},
        {"reactive + switch (default)", amr::DynamicPlanner::Reactive, 15.0, true},
        {"space-time, no switch", amr::DynamicPlanner::SpaceTime, 0.0, true},
        {"space-time + switch", amr::DynamicPlanner::SpaceTime, 15.0, true},
    };
    const std::vector<int> counts{2, 3, 4, 6};
    std::mt19937 rng(o.seed);
    std::vector<std::vector<std::vector<amr::RobotTask>>> tasks(counts.size());
    for (std::size_t c = 0; c < counts.size(); ++c) {
        for (int run = 0; run < o.runs; ++run) {
            tasks[c].push_back(randomTasks(planning, counts[c], rng));
        }
    }
    const std::size_t nv = variants.size(), runs = static_cast<std::size_t>(o.runs);
    const std::size_t jobs = counts.size() * runs * 2 * nv;
    std::vector<amr::MultiRobotResult> results(jobs);
    std::atomic<std::size_t> next{0};
    const auto worker = [&]() {
        for (std::size_t job = next++; job < jobs; job = next++) {
            const std::size_t v = job % nv, model = (job / nv) % 2, run = (job / nv / 2) % runs,
                              c = job / nv / 2 / runs;
            amr::MultiRobotConfig config;
            config.motion.model = model == 0 ? amr::RobotModel::Point : amr::RobotModel::DiffDrive;
            config.motion.robot_radius = kRadius;
            config.motion.max_time = 300.0;
            config.scenario.planning_map = &planning;
            config.scenario.replanning.planner = variants[v].planner;
            config.scenario.replanning.livelock_window = variants[v].livelock_window;
            config.scenario.perception.lidar.seed = static_cast<unsigned>(run * 101 + 7);
            config.stagger_livelock = variants[v].stagger;
            amr::MultiRobotResult r = amr::simulateRobots(map, tasks[c][run], config);
            for (amr::SimulationResult& robot : r.robots) {
                robot.trajectory.clear();
                robot.trajectory.shrink_to_fit();
                robot.replans.clear();
            }
            results[job] = std::move(r);
        }
    };
    std::vector<std::thread> threads;
    for (unsigned t = 0; t < std::max(1u, std::thread::hardware_concurrency()); ++t) {
        threads.emplace_back(worker);
    }
    for (std::thread& t : threads) {
        t.join();
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "planning_robots.csv");
    csv << "robot,robots,variant,runs,all_reached,robots_reached,robot_collisions,wall_collisions,timeouts,"
           "mean_makespan_s,strategy_switches\n";
    std::cout << "Robots that all replan (" << o.runs << " random task sets per count on " << o.map
              << ", radius " << kRadius << " m, LiDAR perception, 300 s limit; all = every robot reached its "
              << "goal; collisions = robots ending in robot-robot contact; makespan over runs where all reached)\n";
    for (int model = 0; model < 2; ++model) {
        std::cout << "\n" << (model == 0 ? "point" : "diff-drive") << " robots\n"
                  << std::left << std::setw(7) << "robots" << std::setw(33) << "variant" << std::right
                  << std::setw(9) << "all" << std::setw(10) << "reached" << std::setw(11) << "collisions"
                  << std::setw(10) << "timeouts" << std::setw(11) << "makespan" << std::setw(10) << "switches"
                  << "\n";
        for (std::size_t c = 0; c < counts.size(); ++c) {
            for (std::size_t v = 0; v < nv; ++v) {
                int all = 0, reached = 0, robot_hits = 0, wall_hits = 0, timeouts = 0, switches = 0;
                double makespan = 0.0;
                for (std::size_t run = 0; run < runs; ++run) {
                    const amr::MultiRobotResult& r = results[((c * runs + run) * 2 + static_cast<std::size_t>(model)) * nv + v];
                    const bool everyone = r.reached == counts[c];
                    all += everyone ? 1 : 0;
                    reached += r.reached;
                    robot_hits += r.robot_collisions;
                    wall_hits += r.obstacle_collisions;
                    timeouts += r.timeouts;
                    switches += r.strategy_switches;
                    makespan += everyone ? r.makespan : 0.0;
                }
                const double n = static_cast<double>(runs);
                std::cout << std::left << std::setw(7) << counts[c] << std::setw(33) << variants[v].name
                          << std::right << std::fixed << std::setprecision(1) << std::setw(8) << 100.0 * all / n
                          << "%" << std::setw(9) << 100.0 * reached / (n * counts[c]) << "%" << std::setw(11)
                          << robot_hits << std::setw(10) << timeouts << std::setw(11)
                          << (all > 0 ? makespan / all : 0.0) << std::setw(10) << switches << "\n";
                csv << (model == 0 ? "point" : "diff-drive") << ',' << counts[c] << ",\"" << variants[v].name
                    << "\"," << runs << ',' << all << ',' << reached << ',' << robot_hits << ',' << wall_hits << ','
                    << timeouts << ',' << (all > 0 ? makespan / all : 0.0) << ',' << switches << "\n";
            }
        }
    }
    return 0;
}

bool parse(int argc, char** argv, Options& o) {
    if (argc < 2) {
        return false;
    }
    o.mode = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (i + 1 >= argc) {
            return false;
        }
        const std::string value = argv[++i];
        if (arg == "--runs") {
            o.runs = std::atoi(value.c_str());
        } else if (arg == "--seed") {
            o.seed = static_cast<unsigned>(std::strtoul(value.c_str(), nullptr, 10));
        } else if (arg == "--map") {
            o.map = value;
        } else if (arg == "--connectivity") {
            o.connectivity = std::atoi(value.c_str());
        } else if (arg == "--speeds") {
            const auto comma = value.find(',');
            o.min_speed = std::atof(value.substr(0, comma).c_str());
            o.max_speed = comma == std::string::npos ? o.min_speed : std::atof(value.substr(comma + 1).c_str());
        } else if (arg == "--output") {
            o.output = value;
        } else if (arg == "--livelock") {
            o.livelock = std::atof(value.c_str());
        } else {
            return false;
        }
    }
    return o.connectivity == 4 || o.connectivity == 8;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::cerr << "usage: planning_experiment turns|spacetime|margins|robots [--runs N] [--seed N] [--map FILE] "
                     "[--connectivity 4|8] [--speeds MIN,MAX] [--output DIR]\n";
        return 2;
    }
    if (o.mode == "turns") {
        return modeTurns(o);
    }
    if (o.mode == "spacetime") {
        return modeSpaceTime(o);
    }
    if (o.mode == "margins") {
        return modeMargins(o);
    }
    if (o.mode == "robots") {
        return modeRobots(o);
    }
    std::cerr << "unknown mode '" << o.mode << "'\n";
    return 2;
}
