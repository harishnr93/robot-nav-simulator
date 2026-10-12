// robot_simulator: command-line entry point.
//
// Pipeline: load and validate a map -> plan an A* path -> simulate a point robot
// following the path -> export CSV results -> print a summary.
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <system_error>

#include "astar.hpp"
#include "csv_writer.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "types.hpp"

namespace {

/// Process exit codes. Documented in --help and README.
enum ExitCode : int {
    kExitSuccess = 0,
    kExitUsageError = 1,
    kExitMapError = 2,
    kExitInvalidRequest = 3,
    kExitNoPath = 4,
    kExitOutputError = 5,
    kExitNavigationFailed = 6,
    kExitReferenceMismatch = 7,
};

constexpr double kDefaultResolution = 0.5;  // meters per cell

struct CliOptions {
    std::string map_file;
    amr::GridCoord start;
    amr::GridCoord goal;
    bool has_start = false;
    bool has_goal = false;
    double resolution = kDefaultResolution;
    amr::MotionConfig motion;
    std::filesystem::path output_dir = "results";
    bool reference_bfs = false;
    bool show_help = false;
};

/// Result of the optional optimality check against the reference BFS.
struct ReferenceCheck {
    std::optional<int> cost;  ///< nullopt: not computed; kReferenceNoPath: no path
    bool agrees = true;       ///< BFS and A* agree on reachability and cost
};

void printUsage(std::ostream& os) {
    os << "Usage: robot_simulator --map FILE --start X Y --goal X Y [options]\n"
          "\n"
          "Plans a collision-free path on an occupancy-grid map using A*, then\n"
          "simulates a point robot following it.\n"
          "\n"
          "Required:\n"
          "  --map FILE            Map CSV file (0 = free, 1 = occupied, row 0 = top)\n"
          "  --start X Y           Start cell (x = column, y = row, origin upper-left)\n"
          "  --goal X Y            Goal cell\n"
          "\n"
          "Options:\n"
          "  --resolution M        Cell size in meters (default 0.5)\n"
          "  --speed V             Robot speed in m/s (default 0.5)\n"
          "  --dt S                Simulation timestep in s (default 0.1)\n"
          "  --goal-tolerance M    Allowed final distance to the goal in m (default 0.05)\n"
          "  --max-time S          Simulation time limit in s (default 600)\n"
          "  --robot-radius M      Robot footprint radius in m; obstacles are inflated\n"
          "                        by it for planning (default 0 = point robot)\n"
          "  --output DIR          Output directory (default results/)\n"
          "  --reference-bfs       Also run the reference BFS and check A* optimality\n"
          "  -h, --help            Show this help and exit\n"
          "\n"
          "Outputs:\n"
          "  DIR/path.csv                waypoint_index,grid_x,grid_y,world_x,world_y\n"
          "  DIR/planning_metrics.csv    status, timing, expanded nodes, path length\n"
          "  DIR/trajectory.csv          time,x,y,target_waypoint_index\n"
          "  DIR/simulation_metrics.csv  navigation status, travel time, distance, collisions\n"
          "  DIR/inflated_map.csv        planning map after inflation (robot radius > 0)\n"
          "\n"
          "Exit codes:\n"
          "  0 goal reached, 1 usage error, 2 map error, 3 invalid start/goal,\n"
          "  4 no path exists, 5 output write error, 6 navigation failed,\n"
          "  7 A* disagrees with the reference BFS (only with --reference-bfs)\n";
}

bool parseInt(const std::string& text, int& value) {
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto [ptr, ec] = std::from_chars(begin, end, value);
    return ec == std::errc() && ptr == end;
}

bool parseFiniteDouble(const std::string& text, double& value) {
    if (text.empty()) {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (end != text.c_str() + text.size() || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

/// Returns std::nullopt and sets error on invalid arguments.
std::optional<CliOptions> parseArguments(int argc, char** argv, std::string& error) {
    CliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto requireValues = [&](int count) {
            if (i + count >= argc) {
                error = "option " + arg + " requires " + std::to_string(count) + " value(s)";
                return false;
            }
            return true;
        };
        auto parseCoord = [&](amr::GridCoord& coord) {
            if (!requireValues(2)) {
                return false;
            }
            const std::string x_text = argv[++i];
            const std::string y_text = argv[++i];
            if (!parseInt(x_text, coord.x) || !parseInt(y_text, coord.y)) {
                error = "option " + arg + " expects two integers, got '" + x_text + "' '" +
                        y_text + "'";
                return false;
            }
            return true;
        };
        auto parseNumber = [&](double& value) {
            if (!requireValues(1)) {
                return false;
            }
            const std::string text = argv[++i];
            if (!parseFiniteDouble(text, value)) {
                error = "option " + arg + " expects a number, got '" + text + "'";
                return false;
            }
            return true;
        };

        bool ok = true;
        if (arg == "-h" || arg == "--help") {
            options.show_help = true;
            return options;
        } else if (arg == "--map") {
            ok = requireValues(1);
            if (ok) {
                options.map_file = argv[++i];
            }
        } else if (arg == "--start") {
            ok = parseCoord(options.start);
            options.has_start = ok;
        } else if (arg == "--goal") {
            ok = parseCoord(options.goal);
            options.has_goal = ok;
        } else if (arg == "--resolution") {
            ok = parseNumber(options.resolution);
            if (ok && options.resolution <= 0.0) {
                error = "--resolution must be positive";
                ok = false;
            }
        } else if (arg == "--speed") {
            ok = parseNumber(options.motion.speed);
        } else if (arg == "--dt") {
            ok = parseNumber(options.motion.dt);
        } else if (arg == "--goal-tolerance") {
            ok = parseNumber(options.motion.goal_tolerance);
        } else if (arg == "--max-time") {
            ok = parseNumber(options.motion.max_time);
        } else if (arg == "--robot-radius") {
            ok = parseNumber(options.motion.robot_radius);
        } else if (arg == "--reference-bfs") {
            options.reference_bfs = true;
        } else if (arg == "--output") {
            ok = requireValues(1);
            if (ok) {
                options.output_dir = argv[++i];
            }
        } else {
            error = "unknown option '" + arg + "'";
            ok = false;
        }
        if (!ok) {
            return std::nullopt;
        }
    }

    if (options.map_file.empty()) {
        error = "missing required option --map";
    } else if (!options.has_start) {
        error = "missing required option --start";
    } else if (!options.has_goal) {
        error = "missing required option --goal";
    } else {
        const std::string motion_error = amr::validateMotionConfig(options.motion);
        if (!motion_error.empty()) {
            error = "invalid motion parameters: " + motion_error;
        }
    }
    if (!error.empty()) {
        return std::nullopt;
    }
    return options;
}

ReferenceCheck checkAgainstBfs(const amr::GridMap& map, const CliOptions& options,
                              const amr::PlanResult& plan) {
    ReferenceCheck check;
    if (amr::validatePlanRequest(map, options.start, options.goal) !=
        amr::PlanStatus::Success) {
        return check;  // invalid start/goal: nothing to compare
    }
    const std::optional<int> bfs = amr::bfsShortestPathCost(map, options.start, options.goal);
    check.cost = bfs ? *bfs : amr::kReferenceNoPath;
    check.agrees = plan.success() ? (bfs && *bfs == plan.path_cost) : !bfs;
    return check;
}

/// Obstacle inflation applied for the robot footprint.
struct Inflation {
    amr::GridMap planning_map;      ///< map A* plans on (== input map for a point robot)
    std::size_t inflated_cells = 0; ///< free cells blocked by the inflation
    double time_ms = 0.0;
};

Inflation inflate(const amr::GridMap& map, double robot_radius) {
    const auto t_begin = std::chrono::steady_clock::now();
    Inflation inflation{amr::inflateObstacles(map, robot_radius)};
    inflation.time_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t_begin).count();
    for (std::size_t i = 0; i < map.cellCount(); ++i) {
        inflation.inflated_cells += map.data()[i] != inflation.planning_map.data()[i];
    }
    return inflation;
}

void printSummary(const CliOptions& options, const amr::GridMap& map,
                  const Inflation& inflation, const amr::PlanResult& plan,
                  const ReferenceCheck& reference, const amr::SimulationResult& sim) {
    const amr::WorldPoint start_w = map.gridToWorld(options.start);
    const amr::WorldPoint goal_w = map.gridToWorld(options.goal);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Map:            " << options.map_file << " (" << map.width() << " x "
              << map.height() << " cells, " << map.resolution() << " m/cell)\n";
    std::cout << "Start:          cell " << options.start << ", world (" << start_w.x << ", "
              << start_w.y << ") m\n";
    std::cout << "Goal:           cell " << options.goal << ", world (" << goal_w.x << ", "
              << goal_w.y << ") m\n";
    if (options.motion.robot_radius > 0.0) {
        std::cout << "Footprint:      radius " << std::setprecision(3)
                  << options.motion.robot_radius << " m, "
                  << inflation.inflated_cells << " cells inflated (" << std::setprecision(3)
                  << inflation.time_ms << " ms)\n"
                  << std::setprecision(2);
    }
    std::cout << "Planner status: " << amr::toString(plan.status) << '\n';
    std::cout << "Expanded nodes: " << plan.expanded_nodes << '\n';
    std::cout << std::setprecision(3) << "Planning time:  " << plan.planning_time_ms
              << " ms\n";
    if (plan.success()) {
        std::cout << std::setprecision(2) << "Path:           " << plan.path.size()
                  << " waypoints, " << plan.path_cost << " steps, "
                  << amr::pathLengthMeters(map, plan.path) << " m\n";
    } else {
        std::cout << "Path:           none\n";
    }
    if (reference.cost) {
        std::cout << "Reference BFS:  ";
        if (*reference.cost == amr::kReferenceNoPath) {
            std::cout << "no path";
        } else {
            std::cout << *reference.cost << " steps";
        }
        std::cout << (reference.agrees ? " (matches A*)" : " (MISMATCH with A*)") << '\n';
    }
    std::cout << "Navigation:     " << amr::toString(sim.status) << '\n';
    if (!sim.trajectory.empty()) {
        std::cout << std::setprecision(2) << "Motion:         " << sim.steps << " steps of "
                  << options.motion.dt << " s at " << options.motion.speed << " m/s, travel time "
                  << sim.travelTime() << " s, distance " << sim.distance_traveled << " m\n";
        std::cout << std::setprecision(4) << "Goal error:     " << sim.final_goal_error
                  << " m (tolerance " << options.motion.goal_tolerance << " m), collisions "
                  << sim.collision_count << '\n';
    }
    std::cout << "Results:        " << options.output_dir.string() << '\n';
}

int exitCodeFor(amr::PlanStatus plan_status, amr::NavigationStatus nav_status) {
    switch (plan_status) {
        case amr::PlanStatus::Success:
            return nav_status == amr::NavigationStatus::GoalReached ? kExitSuccess
                                                                    : kExitNavigationFailed;
        case amr::PlanStatus::NoPath:
            return kExitNoPath;
        case amr::PlanStatus::StartOutOfBounds:
        case amr::PlanStatus::GoalOutOfBounds:
        case amr::PlanStatus::StartOccupied:
        case amr::PlanStatus::GoalOccupied:
        case amr::PlanStatus::StartTooCloseToObstacle:
        case amr::PlanStatus::GoalTooCloseToObstacle:
            return kExitInvalidRequest;
    }
    return kExitInvalidRequest;
}

bool writeResults(const CliOptions& options, const amr::GridMap& map,
                  const Inflation& inflation, const amr::PlanResult& plan,
                  const ReferenceCheck& reference, const amr::SimulationResult& sim,
                  std::string& error) {
    const std::filesystem::path& dir = options.output_dir;
    // Every file is rewritten on every run (header-only when there is no data),
    // so results from an earlier run can never be mistaken for the current one.
    // inflated_map.csv only exists for a footprint; a stale one is removed.
    const std::filesystem::path inflated_file = dir / "inflated_map.csv";
    if (options.motion.robot_radius > 0.0) {
        if (!amr::writeGridMapCsv(inflated_file, inflation.planning_map, error)) {
            return false;
        }
    } else {
        std::error_code ignored;
        std::filesystem::remove(inflated_file, ignored);
    }
    return amr::writePathCsv(dir / "path.csv", map, plan.path, error) &&
           amr::writePlanningMetricsCsv(dir / "planning_metrics.csv", map, options.start,
                                        options.goal, plan, reference.cost,
                                        options.motion.robot_radius, inflation.time_ms,
                                        error) &&
           amr::writeTrajectoryCsv(dir / "trajectory.csv", sim.trajectory, error) &&
           amr::writeSimulationMetricsCsv(dir / "simulation_metrics.csv", plan.success(),
                                          options.motion, sim, error);
}

}  // namespace

int main(int argc, char** argv) {
    std::string error;
    const std::optional<CliOptions> parsed = parseArguments(argc, argv, error);
    if (!parsed) {
        std::cerr << "Error: " << error << "\nRun with --help for usage.\n";
        return kExitUsageError;
    }
    const CliOptions& options = *parsed;
    if (options.show_help) {
        printUsage(std::cout);
        return kExitSuccess;
    }

    const amr::MapLoadResult loaded = amr::loadGridMapCsv(options.map_file, options.resolution);
    if (!loaded.ok) {
        std::cerr << "Error: " << loaded.error << '\n';
        return kExitMapError;
    }
    const amr::GridMap& map = loaded.map;

    // Plan on the inflated map (identical to `map` for a point robot). Inflation is
    // timed separately and is not part of the planning time.
    const Inflation inflation = inflate(map, options.motion.robot_radius);
    const amr::PlanResult plan =
        amr::planWithFootprint(map, inflation.planning_map, options.start, options.goal);
    // The BFS check runs after planning, on the same planning map.
    const ReferenceCheck reference = options.reference_bfs
                                         ? checkAgainstBfs(inflation.planning_map, options, plan)
                                         : ReferenceCheck{};
    amr::SimulationResult sim;  // stays NOT_RUN with no trajectory if planning failed
    if (plan.success()) {
        sim = amr::simulatePath(map, plan.path, options.motion);
    }

    std::error_code ec;
    std::filesystem::create_directories(options.output_dir, ec);
    if (ec) {
        std::cerr << "Error: cannot create output directory '" << options.output_dir.string()
                  << "': " << ec.message() << '\n';
        return kExitOutputError;
    }
    if (!writeResults(options, map, inflation, plan, reference, sim, error)) {
        std::cerr << "Error: " << error << '\n';
        return kExitOutputError;
    }

    printSummary(options, map, inflation, plan, reference, sim);
    if (!reference.agrees) {
        std::cerr << "Error: A* result disagrees with the reference BFS\n";
        return kExitReferenceMismatch;
    }
    if (!plan.success()) {
        std::cerr << "Planning failed: " << amr::toString(plan.status) << '\n';
    } else if (!sim.success()) {
        std::cerr << "Navigation failed: " << amr::toString(sim.status) << '\n';
    }
    return exitCodeFor(plan.status, sim.status);
}
