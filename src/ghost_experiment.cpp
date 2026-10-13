// ghost_experiment: "why NO_PATH?" experiments.
//
//   ghost_experiment recovery [options]   perception loop (--label NAME names the CSV;
//                                         --cover-radius, --ghosts, --density, --sector,
//                                         --max-rounds set the scenario): random worlds with ghost
//                                         sources, a mapped belief, and the recovery
//                                         strategies compared (ghost_perception.hpp)
//   ghost_experiment scaling [options]    exact solver: explanation size, labels and
//                                         time vs. obstacle density, robot radius
//                                         and map size; brute force for comparison
//   ghost_experiment example [--seed N] [--density D]
//                                         one NO_PATH map with every optimal
//                                         explanation, for the figure
//
// Options: --trials N, --seed N, --output DIR (default results/ghost).
// Writes ghost_recovery_<label>.csv / ghost_scaling.csv and prints a summary table.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "astar.hpp"
#include "footprint.hpp"
#include "ghost_cells.hpp"
#include "ghost_perception.hpp"
#include "grid_map.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Options {
    std::string mode;
    int trials = 200;
    unsigned seed = 1;
    std::filesystem::path output = "results/ghost";
    // recovery mode
    double density = 0.04;       ///< real obstacle density
    int ghosts = 3;              ///< ghost sources next to the true route
    double cover_radius = 2.0;   ///< m: mapping viewpoints within this of every reachable cell
    double sector_deg = 104.0;   ///< full width of a ghost's active sector, deg
    int max_rounds = 10;
    std::string label = "thorough";  ///< recovery output: ghost_recovery_<label>.csv
};

/// Random map with forced-free 5 x 5 blocks at the start and goal corners (as in
/// the original NO_PATH experiment).
amr::GridMap randomMap(std::mt19937& rng, int n, double res, double density) {
    std::bernoulli_distribution occupied(density);
    std::vector<std::uint8_t> cells(static_cast<std::size_t>(n * n), 0);
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const bool near_start = x <= 4 && y <= 4;
            const bool near_goal = x >= n - 5 && y >= n - 5;
            cells[static_cast<std::size_t>(y * n + x)] =
                !near_start && !near_goal && occupied(rng) ? amr::GridMap::kOccupied
                                                           : amr::GridMap::kFree;
        }
    }
    return amr::GridMap(n, n, res, std::move(cells));
}

/// Cells reachable by the disc robot from `start` (on the inflated true map).
std::vector<amr::GridCoord> reachableCells(const amr::GridMap& inflated, amr::GridCoord start) {
    std::vector<amr::GridCoord> out;
    std::vector<bool> seen(inflated.cellCount(), false);
    if (!inflated.isFree(start)) {
        return out;
    }
    std::vector<amr::GridCoord> stack{start};
    seen[inflated.index(start)] = true;
    while (!stack.empty()) {
        const amr::GridCoord c = stack.back();
        stack.pop_back();
        out.push_back(c);
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; ++d) {
            const amr::GridCoord q{c.x + dx[d], c.y + dy[d]};
            if (inflated.isFree(q) && !seen[inflated.index(q)]) {
                seen[inflated.index(q)] = true;
                stack.push_back(q);
            }
        }
    }
    return out;
}

int runRecovery(const Options& options) {
    constexpr int kN = 40;
    constexpr double kRes = 0.25;
    const int kGhosts = options.ghosts;
    const double kCoverRadius = options.cover_radius;
    const double density = options.density;
    const amr::GridCoord start{2, 2};
    const amr::GridCoord goal{kN - 3, kN - 3};
    amr::SensorModel sensor;
    amr::RecoveryConfig config;
    config.max_rounds = options.max_rounds;
    const std::vector<amr::RecoveryStrategy> strategies = {
        amr::RecoveryStrategy::MostLikely, amr::RecoveryStrategy::LogOdds,
        amr::RecoveryStrategy::FewestCells,
        amr::RecoveryStrategy::ClearAll, amr::RecoveryStrategy::BlindDelete,
        amr::RecoveryStrategy::ResenseAll};

    std::filesystem::create_directories(options.output);
    const std::filesystem::path csv_path = options.output / ("ghost_recovery_" + options.label + ".csv");
    std::ofstream csv(csv_path);
    csv << "trial,strategy,outcome,rounds,beams,cells_resensed,real_cells_cleared,travel_m,"
           "solver_ms,ghost_cells_blocking,believed_occupied\n";
    std::mt19937 rng(options.seed);
    int worlds = 0;
    int belief_ok = 0;
    int belief_unsafe = 0;
    int no_path = 0;
    std::map<std::string, std::map<std::string, int>> outcomes;
    std::map<std::string, std::vector<double>> beams_used;
    std::map<std::string, std::vector<double>> rounds_used;
    std::map<std::string, int> reasons;
    std::map<std::string, long> real_cleared;
    std::map<std::string, std::vector<double>> travel_used;
    while (no_path < options.trials) {
        ++worlds;
        amr::GhostWorld world{randomMap(rng, kN, kRes, density), {}};
        const amr::GridMap true_inflated = amr::inflateObstacles(world.truth, config.robot_radius);
        const amr::PlanResult true_plan = amr::planAStar(true_inflated, start, goal);
        if (!true_plan.success()) {
            continue;  // only worlds where the goal really is reachable
        }
        // Ghost sources next to the true route (where false obstacles hurt).
        std::vector<amr::GridCoord> candidates;
        for (const amr::GridCoord& p : true_plan.path) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const amr::GridCoord c{p.x + dx, p.y + dy};
                    const bool near_ends = std::max(std::abs(c.x - start.x), std::abs(c.y - start.y)) <= 3 ||
                                           std::max(std::abs(c.x - goal.x), std::abs(c.y - goal.y)) <= 3;
                    if (world.truth.isFree(c) && !near_ends &&
                        std::find(candidates.begin(), candidates.end(), c) == candidates.end()) {
                        candidates.push_back(c);
                    }
                }
            }
        }
        std::shuffle(candidates.begin(), candidates.end(), rng);
        std::uniform_real_distribution<double> heading(-kPi, kPi);
        for (int g = 0; g < kGhosts && g < static_cast<int>(candidates.size()); ++g) {
            world.ghosts.push_back({candidates[static_cast<std::size_t>(g)], heading(rng),
                                    options.sector_deg * kPi / 360.0});
        }
        // Map from viewpoints spread over the reachable area: greedily add a random
        // reachable cell that is farther than kCoverRadius from every viewpoint so far,
        // so the whole area is scanned from close by.
        std::vector<amr::GridCoord> reachable = reachableCells(true_inflated, start);
        std::shuffle(reachable.begin(), reachable.end(), rng);
        std::vector<amr::WorldPoint> viewpoints{world.truth.gridToWorld(start)};
        for (const amr::GridCoord& c : reachable) {
            const amr::WorldPoint p = world.truth.gridToWorld(c);
            const bool covered = std::any_of(viewpoints.begin(), viewpoints.end(), [&](const auto& v) {
                return std::hypot(p.x - v.x, p.y - v.y) <= kCoverRadius;
            });
            if (!covered) {
                viewpoints.push_back(p);
            }
        }
        amr::OccupancyBelief belief(kN, kN, kRes);
        for (const amr::WorldPoint& v : viewpoints) {
            amr::scanInto(belief, world, v, sensor, rng);
        }
        const amr::GridMap seen = belief.map();
        const amr::PlanResult belief_plan =
            amr::planAStar(amr::inflateObstacles(seen, config.robot_radius), start, goal);
        if (belief_plan.success()) {
            ++belief_ok;
            if (!amr::isValidGridPath(true_inflated, belief_plan.path)) {
                ++belief_unsafe;
            }
            continue;
        }
        ++no_path;
        int ghosts_blocking = 0;
        for (const amr::GhostSource& g : world.ghosts) {
            ghosts_blocking += seen.isOccupied(g.cell) ? 1 : 0;
        }
        long believed = 0;
        for (std::size_t i = 0; i < seen.cellCount(); ++i) {
            believed += seen.data()[i] != amr::GridMap::kFree ? 1 : 0;
        }
        const unsigned trial_seed = static_cast<unsigned>(rng());
        for (const amr::RecoveryStrategy strategy : strategies) {
            std::mt19937 trial_rng(trial_seed);  // same sensing noise for every strategy
            const amr::RecoveryResult r =
                amr::recover(strategy, belief, world, viewpoints, start, goal, sensor, config, trial_rng);
            const std::string name = amr::toString(strategy);
            outcomes[name][amr::toString(r.outcome)]++;
            real_cleared[name] += r.real_cells_cleared;
            if (!r.reason.empty()) {
                reasons[name + " " + r.reason]++;
            }
            if (r.outcome == amr::RecoveryOutcome::Recovered) {
                beams_used[name].push_back(static_cast<double>(r.beams));
                rounds_used[name].push_back(r.rounds);
                travel_used[name].push_back(r.travel_m);
            }
            csv << no_path << ',' << name << ',' << amr::toString(r.outcome) << ',' << r.rounds
                << ',' << r.beams << ',' << r.cells_resensed << ',' << r.real_cells_cleared << ','
                << std::fixed << std::setprecision(3) << r.travel_m << ',' << std::setprecision(4) << r.solver_ms << ',' << ghosts_blocking << ','
                << believed << '\n';
        }
    }
    std::cout << worlds << " reachable-or-not worlds generated; of the reachable ones, the mapped "
              << "belief had a path in " << belief_ok << " (" << belief_unsafe
              << " of them unsafe: missed obstacles) and NO_PATH in " << no_path << ".\n\n";
    std::cout << "Scenario: density " << density << ", " << kGhosts << " ghost sources ("
              << options.sector_deg << " deg sectors), mapping viewpoints every "
              << kCoverRadius << " m, " << config.max_rounds << " rounds max\n\n";
    std::cout << std::left << std::setw(14) << "strategy" << std::setw(11) << "recovered"
              << std::setw(9) << "unsafe" << std::setw(9) << "gave up" << std::setw(14)
              << "real cleared" << std::setw(14) << "median beams" << std::setw(15)
              << "median rounds" << "median drive m\n";
    const auto median = [](std::vector<double> v) {
        if (v.empty()) {
            return 0.0;
        }
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    for (const amr::RecoveryStrategy strategy : strategies) {
        const std::string name = amr::toString(strategy);
        std::cout << std::left << std::setw(14) << name << std::setw(11)
                  << outcomes[name]["RECOVERED"] << std::setw(9) << outcomes[name]["UNSAFE"]
                  << std::setw(9) << outcomes[name]["GAVE_UP"] << std::setw(14)
                  << real_cleared[name] << std::setw(14) << median(beams_used[name])
                  << std::setw(15) << median(rounds_used[name]) << median(travel_used[name])
                  << '\n';
    }
    for (const auto& [key, count] : reasons) {
        std::cout << "  gave up: " << key << " " << count << '\n';
    }
    std::cout << "\nWrote " << csv_path.string() << '\n';
    return 0;
}

int runExample(const Options& options) {
    constexpr int kN = 40;
    constexpr double kRes = 0.25;
    constexpr double kRadius = 0.3;
    std::mt19937 rng(options.seed);
    const amr::GridMap map = randomMap(rng, kN, kRes, options.density);
    amr::GhostQuery query;
    query.robot_radius = kRadius;
    query.max_solutions = 1000;
    const amr::GhostResult result = amr::findGhostCells(map, {2, 2}, {kN - 3, kN - 3}, query);
    std::cout << "status " << amr::toString(result.status) << ", " << result.cost << " cell(s), "
              << result.solutions.size() << " optimal explanation(s), " << result.time_ms << " ms\n";
    std::filesystem::create_directories(options.output);
    const amr::GridMap inflated = amr::inflateObstacles(map, kRadius);
    std::ofstream grid(options.output / "example_map.csv");
    grid << "x,y,occupied,inflated\n";
    for (int y = 0; y < kN; ++y) {
        for (int x = 0; x < kN; ++x) {
            grid << x << ',' << y << ',' << (map.isOccupied({x, y}) ? 1 : 0) << ','
                 << (inflated.isOccupied({x, y}) ? 1 : 0) << '\n';
        }
    }
    std::ofstream solutions(options.output / "example_solutions.csv");
    solutions << "solution,x,y\n";
    for (std::size_t i = 0; i < result.solutions.size(); ++i) {
        for (const amr::GridCoord& c : result.solutions[i]) {
            solutions << i << ',' << c.x << ',' << c.y << '\n';
        }
    }
    std::ofstream path(options.output / "example_path.csv");
    path << "x,y\n";
    for (const amr::GridCoord& c : result.path) {
        path << c.x << ',' << c.y << '\n';
    }
    std::ofstream meta(options.output / "example_meta.csv");
    meta << "resolution,robot_radius,start_x,start_y,goal_x,goal_y,status,cost,solutions,time_ms\n"
         << kRes << ',' << kRadius << ",2,2," << kN - 3 << ',' << kN - 3 << ','
         << amr::toString(result.status) << ',' << result.cost << ',' << result.solutions.size()
         << ',' << result.time_ms << '\n';
    return result.status == amr::GhostStatus::Found ? 0 : 1;
}

int runScaling(const Options& options) {
    std::filesystem::create_directories(options.output);
    std::ofstream csv(options.output / "ghost_scaling.csv");
    csv << "map_cells,density,radius_m,connectivity,seed,status,ghost_cells,labels,solver_ms,"
           "bruteforce_ms\n";
    struct Setting {
        int n;
        double density;
        double radius;
        amr::Connectivity connectivity;
        bool brute_force;
    };
    std::vector<Setting> settings;
    for (const double density : {0.04, 0.06, 0.08, 0.10, 0.12, 0.15}) {
        settings.push_back({40, density, 0.3, amr::Connectivity::Four, density <= 0.06});
    }
    for (const double radius : {0.2, 0.4, 0.6}) {
        settings.push_back({40, 0.06, radius, amr::Connectivity::Four, false});
    }
    settings.push_back({40, 0.08, 0.3, amr::Connectivity::Eight, false});
    for (const int n : {20, 60, 80, 120}) {
        settings.push_back({n, 0.06, 0.3, amr::Connectivity::Four, false});
    }
    std::cout << std::left << std::setw(7) << "cells" << std::setw(9) << "density" << std::setw(8)
              << "radius" << std::setw(6) << "conn" << std::setw(11) << "NO_PATH" << std::setw(22)
              << "ghost cells (median/max)" << std::setw(16) << "labels (median)"
              << std::setw(22) << "solver ms (median/max)" << "brute force ms (median)\n";
    for (const Setting& s : settings) {
        const amr::GridCoord start{2, 2};
        const amr::GridCoord goal{s.n - 3, s.n - 3};
        std::vector<double> sizes, labels, times, brute;
        int failures = 0;
        int limits = 0;
        for (int seed = 0; seed < options.trials; ++seed) {
            std::mt19937 rng(options.seed * 100003u + static_cast<unsigned>(seed));
            const amr::GridMap map = randomMap(rng, s.n, 0.25, s.density);
            amr::GhostQuery query;
            query.robot_radius = s.radius;
            query.connectivity = s.connectivity;
            query.max_labels = 2'000'000;  // bounds memory (labels hold sets)
            const amr::GhostResult r = amr::findGhostCells(map, start, goal, query);
            if (r.status == amr::GhostStatus::PathExists || r.status == amr::GhostStatus::Impossible) {
                continue;
            }
            ++failures;
            double brute_ms = -1.0;
            if (s.brute_force && r.status == amr::GhostStatus::Found && r.cost <= 2.5) {
                const auto t0 = std::chrono::steady_clock::now();
                const int k = amr::referenceGhostCount(map, start, goal, s.radius, s.connectivity, 2);
                brute_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t0).count();
                if (k != static_cast<int>(std::lround(r.cost))) {
                    std::cerr << "MISMATCH with brute force (seed " << seed << ")\n";
                    return 1;
                }
                brute.push_back(brute_ms);
            }
            if (r.status == amr::GhostStatus::LimitReached) {
                ++limits;
            } else {
                sizes.push_back(r.cost);
            }
            labels.push_back(static_cast<double>(r.labels));
            times.push_back(r.time_ms);
            csv << s.n * s.n << ',' << s.density << ',' << s.radius << ','
                << static_cast<int>(s.connectivity) << ',' << seed << ',' << amr::toString(r.status)
                << ',' << (r.status == amr::GhostStatus::Found ? std::to_string(std::lround(r.cost)) : "")
                << ',' << r.labels << ',' << std::fixed << std::setprecision(4) << r.time_ms << ','
                << (brute_ms >= 0 ? std::to_string(brute_ms) : "") << '\n';
        }
        const auto median = [](std::vector<double> v) {
            if (v.empty()) {
                return 0.0;
            }
            std::sort(v.begin(), v.end());
            return v[v.size() / 2];
        };
        const auto max = [](const std::vector<double>& v) {
            return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
        };
        std::ostringstream size_text, time_text;
        size_text << median(sizes) << " / " << max(sizes) << (limits ? " (+" + std::to_string(limits) + " limit)" : "");
        time_text << std::fixed << std::setprecision(2) << median(times) << " / " << max(times);
        std::cout << std::left << std::setw(7) << s.n * s.n << std::setw(9) << s.density
                  << std::setw(8) << s.radius << std::setw(6) << static_cast<int>(s.connectivity)
                  << std::setw(11) << (std::to_string(failures) + "/" + std::to_string(options.trials))
                  << std::setw(22) << size_text.str() << std::setw(16) << median(labels)
                  << std::setw(22) << time_text.str()
                  << (brute.empty() ? std::string("-") : std::to_string(median(brute))) << '\n';
    }
    std::cout << "\nWrote " << (options.output / "ghost_scaling.csv").string() << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    const std::string mode = argc >= 2 ? argv[1] : "";
    if (mode != "recovery" && mode != "scaling" && mode != "example") {
        std::cerr << "usage: ghost_experiment recovery|scaling|example [--trials N] [--seed N] "
                     "[--output DIR] [--density D] [--ghosts N] [--cover-radius M] "
                     "[--sector DEG] [--max-rounds N] [--label NAME]\n";
        return 1;
    }
    options.mode = argv[1];
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string arg = argv[i];
        if (arg == "--trials") {
            options.trials = std::max(1, std::atoi(argv[i + 1]));
        } else if (arg == "--seed") {
            options.seed = static_cast<unsigned>(std::strtoul(argv[i + 1], nullptr, 10));
        } else if (arg == "--output") {
            options.output = argv[i + 1];
        } else if (arg == "--label") {
            options.label = argv[i + 1];
        } else if (arg == "--density") {
            options.density = std::atof(argv[i + 1]);
        } else if (arg == "--ghosts") {
            options.ghosts = std::atoi(argv[i + 1]);
        } else if (arg == "--cover-radius") {
            options.cover_radius = std::atof(argv[i + 1]);
        } else if (arg == "--sector") {
            options.sector_deg = std::atof(argv[i + 1]);
        } else if (arg == "--max-rounds") {
            options.max_rounds = std::atoi(argv[i + 1]);
        } else {
            std::cerr << "unknown option " << arg << '\n';
            return 1;
        }
    }
    if (options.mode == "example") {
        return runExample(options);
    }
    return options.mode == "recovery" ? runRecovery(options) : runScaling(options);
}
