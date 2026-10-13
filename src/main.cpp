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
#include "ekf.hpp"
#include "footprint.hpp"
#include "lidar.hpp"
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
constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;

struct CliOptions {
    std::string map_file;
    amr::GridCoord start;
    amr::GridCoord goal;
    bool has_start = false;
    bool has_goal = false;
    double resolution = kDefaultResolution;
    amr::MotionConfig motion;
    amr::LidarConfig lidar;
    amr::EkfConfig ekf;
    std::string dynamic_file;           ///< --dynamic-obstacles (empty: static world)
    amr::ReplanningConfig replanning;
    std::filesystem::path output_dir = "results";
    bool reference_bfs = false;
    amr::Connectivity connectivity = amr::Connectivity::Four;
    bool show_help = false;
};

/// Result of the optional optimality check against the reference BFS.
struct ReferenceCheck {
    amr::ReferenceCost result;  ///< what the reference search found (if it ran)
    bool agrees = true;         ///< reference and A* agree on reachability and cost
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
          "  --robot-model MODEL   point (default) or diff-drive (heading, turn-then-drive)\n"
          "  --max-angular-speed W Differential drive: angular speed limit in rad/s (1.5)\n"
          "  --heading-gain K      Differential drive: P gain of the heading controller\n"
          "                        in 1/s (5.0); K * dt must be at most 1\n"
          "  --initial-heading DEG Differential drive: heading at t = 0 in degrees, 0 =\n"
          "                        facing +x, 90 = facing +y (down the map) (default 0)\n"
          "  --lidar-beams N       Simulated 2D LiDAR with N beams, one scan per\n"
          "                        trajectory sample (default 0 = off)\n"
          "  --lidar-fov DEG       LiDAR field of view in degrees, centred on the heading\n"
          "                        (default 360)\n"
          "  --lidar-range M       LiDAR maximum range in m (default 5)\n"
          "  --lidar-noise SIGMA   Gaussian range noise in m (default 0)\n"
          "  --lidar-seed N        Seed of the range noise (default 0)\n"
          "  --ekf                 EKF localization (differential drive): noisy wheel\n"
          "                        odometry, corrected by the LiDAR scans if enabled\n"
          "  --odom-noise-dist K   Odometry distance error, m per sqrt(m) driven (0.05)\n"
          "  --odom-noise-turn K   Odometry heading error, rad per sqrt(rad) turned (0.05)\n"
          "  --odom-noise-drift K  Odometry heading drift, rad per sqrt(m) driven (0.02)\n"
          "  --odom-seed N         Seed of the odometry noise (default 0)\n"
          "  --ekf-range-sigma M   LiDAR range std assumed by the EKF in m (default 0.05)\n"
          "  --ekf-gate N          Reject beams whose innovation exceeds N std (default 3)\n"
          "  --dynamic-obstacles FILE  Moving obstacles (one per line: radius_m,speed_mps,\n"
          "                        x0,y0,x1,y1,... cells; back and forth); the robot\n"
          "                        replans when one blocks its path\n"
          "  --sensing-range M     Dynamic obstacles are seen within M meters, in line\n"
          "                        of sight (default 3)\n"
          "  --replan-margin M     Extra clearance around a seen obstacle when\n"
          "                        replanning, m (default 0.25)\n"
          "  --prediction-horizon S  Block where a seen obstacle will be within S seconds\n"
          "                        at its estimated velocity (default 2; 0 = where it is)\n"
          "  --replan-patience S   Wait up to S seconds for a blocked path to clear before\n"
          "                        replanning a detour (default 6; 0 = replan at once)\n"

          "  --robot-radius M      Robot footprint radius in m; obstacles are inflated\n"
          "                        by it for planning (default 0 = point robot)\n"
          "  --output DIR          Output directory (default results/)\n"
          "  --connectivity 4|8    Grid moves: 4 = straight only (default), 8 = also\n"
          "                        diagonal (cost sqrt(2), no cutting obstacle corners)\n"
          "  --reference-bfs       Also run the reference search (BFS for 4-connected,\n"
          "                        exact Dijkstra for 8-connected) and check A* optimality\n"
          "  -h, --help            Show this help and exit\n"
          "\n"
          "Outputs:\n"
          "  DIR/path.csv                waypoint_index,grid_x,grid_y,world_x,world_y\n"
          "  DIR/planning_metrics.csv    status, timing, expanded nodes, path length\n"
          "  DIR/trajectory.csv          time,x,y,target_waypoint_index\n"
          "  DIR/simulation_metrics.csv  navigation status, travel time, distance, collisions\n"
          "  DIR/inflated_map.csv        planning map after inflation (robot radius > 0)\n"
          "  DIR/lidar_scans.csv         LiDAR scans: time,beam,angle,range,hit (LiDAR on)\n"
          "  DIR/localization.csv        true, dead-reckoning and EKF poses, covariance (--ekf)\n"
          "  DIR/dynamic_obstacles.csv   obstacle positions per sample (--dynamic-obstacles)\n"
          "  DIR/replans.csv, DIR/replanned_paths.csv  replanning attempts and new paths\n"
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
        } else if (arg == "--robot-model") {
            ok = requireValues(1);
            if (ok) {
                const std::string text = argv[++i];
                if (text == "point" || text == "diff-drive") {
                    options.motion.model = text == "point" ? amr::RobotModel::Point
                                                           : amr::RobotModel::DiffDrive;
                } else {
                    error = "--robot-model must be point or diff-drive, got '" + text + "'";
                    ok = false;
                }
            }
        } else if (arg == "--max-angular-speed") {
            ok = parseNumber(options.motion.max_angular_speed);
        } else if (arg == "--heading-gain") {
            ok = parseNumber(options.motion.heading_gain);
        } else if (arg == "--lidar-beams") {
            double beams = 0.0;
            ok = parseNumber(beams);
            if (ok && (beams != std::floor(beams) || beams < 0 || beams > amr::kMaxLidarBeams)) {
                error = "--lidar-beams must be a whole number between 0 and 3600";
                ok = false;
            }
            options.lidar.beams = static_cast<int>(beams);
        } else if (arg == "--lidar-fov") {
            double degrees = 0.0;
            ok = parseNumber(degrees);
            options.lidar.fov = degrees * kRadiansPerDegree;
        } else if (arg == "--lidar-range") {
            ok = parseNumber(options.lidar.max_range);
        } else if (arg == "--lidar-noise") {
            ok = parseNumber(options.lidar.noise_stddev);
        } else if (arg == "--lidar-seed") {
            double seed = 0.0;
            ok = parseNumber(seed);
            if (ok && (seed != std::floor(seed) || seed < 0 || seed > 4294967295.0)) {
                error = "--lidar-seed must be a whole number between 0 and 4294967295";
                ok = false;
            }
            options.lidar.seed = static_cast<std::uint32_t>(seed);
        } else if (arg == "--dynamic-obstacles") {
            ok = requireValues(1);
            if (ok) {
                options.dynamic_file = argv[++i];
            }
        } else if (arg == "--sensing-range") {
            ok = parseNumber(options.replanning.sensing_range);
        } else if (arg == "--replan-margin") {
            ok = parseNumber(options.replanning.margin);
        } else if (arg == "--replan-patience") {
            ok = parseNumber(options.replanning.patience);
        } else if (arg == "--prediction-horizon") {
            ok = parseNumber(options.replanning.prediction_horizon);
        } else if (arg == "--ekf") {
            options.ekf.enabled = true;
        } else if (arg == "--odom-noise-dist") {
            ok = parseNumber(options.ekf.odometry.distance);
        } else if (arg == "--odom-noise-turn") {
            ok = parseNumber(options.ekf.odometry.turn);
        } else if (arg == "--odom-noise-drift") {
            ok = parseNumber(options.ekf.odometry.drift);
        } else if (arg == "--odom-seed") {
            double seed = 0.0;
            ok = parseNumber(seed);
            if (ok && (seed != std::floor(seed) || seed < 0 || seed > 4294967295.0)) {
                error = "--odom-seed must be a whole number between 0 and 4294967295";
                ok = false;
            }
            options.ekf.odometry.seed = static_cast<std::uint32_t>(seed);
        } else if (arg == "--ekf-range-sigma") {
            ok = parseNumber(options.ekf.range_sigma);
        } else if (arg == "--ekf-gate") {
            ok = parseNumber(options.ekf.gate);
        } else if (arg == "--initial-heading") {
            double degrees = 0.0;
            ok = parseNumber(degrees);
            options.motion.initial_heading = degrees * kRadiansPerDegree;
        } else if (arg == "--robot-radius") {
            ok = parseNumber(options.motion.robot_radius);
        } else if (arg == "--connectivity") {
            ok = requireValues(1);
            if (ok) {
                const std::string text = argv[++i];
                int value = 0;
                if (parseInt(text, value) && (value == 4 || value == 8)) {
                    options.connectivity = static_cast<amr::Connectivity>(value);
                } else {
                    error = "--connectivity must be 4 or 8, got '" + text + "'";
                    ok = false;
                }
            }
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
        const std::string lidar_error = amr::validateLidarConfig(options.lidar);
        const std::string ekf_error = amr::validateEkfConfig(options.ekf);
        const std::string replan_error = amr::validateReplanningConfig(options.replanning);
        if (!motion_error.empty()) {
            error = "invalid motion parameters: " + motion_error;
        } else if (!lidar_error.empty()) {
            error = "invalid LiDAR parameters: " + lidar_error;
        } else if (!ekf_error.empty()) {
            error = "invalid EKF parameters: " + ekf_error;
        } else if (!replan_error.empty()) {
            error = "invalid replanning parameters: " + replan_error;
        } else if (options.ekf.enabled && options.motion.model != amr::RobotModel::DiffDrive) {
            error = "--ekf requires --robot-model diff-drive (the odometry measures turns "
                    "and distances driven)";
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
    const std::optional<amr::GridCost> reference =
        amr::referenceShortestPathCost(map, options.start, options.goal, options.connectivity);
    check.result = {true, reference};
    // Exact comparison: equal costs have equal (straight, diagonal) counts.
    check.agrees = plan.success() ? (reference && *reference == plan.cost()) : !reference;
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
                  const ReferenceCheck& reference, const amr::SimulationResult& sim,
                  const std::vector<amr::LidarScan>& scans,
                  const std::vector<amr::LocalizationSample>& localization,
                  const amr::DynamicScenario* dynamic) {
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
                  << " waypoints, " << plan.path_cost << " steps";
        if (options.connectivity == amr::Connectivity::Eight) {
            std::cout << " (" << plan.diagonal_moves << " diagonal)";
        }
        std::cout << ", " << amr::pathLengthMeters(map, plan.path) << " m\n";
    } else {
        std::cout << "Path:           none\n";
    }
    if (reference.result.computed) {
        const bool four = options.connectivity == amr::Connectivity::Four;
        std::cout << (four ? "Reference BFS:  " : "Reference:      exact Dijkstra, ");
        if (!reference.result.cost) {
            std::cout << "no path";
        } else if (four) {
            std::cout << reference.result.cost->straight << " steps";
        } else {
            std::cout << std::setprecision(6) << reference.result.cost->cells() << " cells"
                      << std::setprecision(2);
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
        if (options.motion.model == amr::RobotModel::DiffDrive) {
            std::cout << std::setprecision(2) << "Heading:        differential drive, "
                      << sim.turns << " turns in place, " << sim.turn_time
                      << " s turning, total rotation " << sim.total_rotation
                      << " rad, final heading "
                      << sim.trajectory.back().state.theta / kRadiansPerDegree << " deg\n";
        }
    }
    if (!scans.empty()) {
        // Closest return over the run: the robot's minimum clearance as the LiDAR saw it.
        double closest = options.lidar.max_range;
        double closest_time = 0.0;
        std::size_t returns = 0;
        std::size_t beams = 0;
        for (const amr::LidarScan& scan : scans) {
            for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
                ++beams;
                if (scan.hits[i]) {
                    ++returns;
                    if (scan.ranges[i] < closest) {
                        closest = scan.ranges[i];
                        closest_time = scan.time;
                    }
                }
            }
        }
        std::cout << std::setprecision(2) << "LiDAR:          " << options.lidar.beams
                  << " beams over " << options.lidar.fov / kRadiansPerDegree << " deg, range "
                  << options.lidar.max_range << " m, " << scans.size() << " scans, "
                  << std::setprecision(1) << 100.0 * static_cast<double>(returns) /
                         static_cast<double>(beams)
                  << " % returns, closest " << std::setprecision(3) << closest << " m at t = "
                  << std::setprecision(1) << closest_time << " s\n";
    }
    if (dynamic != nullptr && !sim.trajectory.empty()) {
        std::size_t failed = 0;
        double replan_ms = 0.0;
        for (const amr::ReplanEvent& event : sim.replans) {
            failed += event.status == amr::PlanStatus::Success ? 0 : 1;
            replan_ms += event.planning_time_ms;
        }
        std::cout << std::setprecision(2) << "Dynamic:        " << dynamic->obstacles.size()
                  << " moving obstacle(s), sensing range " << options.replanning.sensing_range
                  << " m; " << sim.replans.size() - failed << " replan(s)";
        if (failed > 0) {
            std::cout << " + " << failed << " attempt(s) without a path";
        }
        std::cout << " in " << std::setprecision(3) << replan_ms << " ms, "
                  << std::setprecision(1) << static_cast<double>(sim.wait_steps) * options.motion.dt
                  << " s waiting, closest gap " << std::setprecision(3)
                  << sim.min_dynamic_clearance << " m\n";
        if (sim.collided_obstacle >= 0) {
            std::cout << "                hit by moving obstacle " << sim.collided_obstacle
                      << '\n';
        }
    }
    if (!localization.empty()) {
        const amr::LocalizationSummary summary = amr::summarizeLocalization(localization);
        const auto errors = [](const amr::PoseErrorStats& stats) {
            std::cout << "position error RMS " << std::setprecision(3) << stats.position_rms
                      << " m, max " << stats.position_max << " m, final "
                      << stats.position_final << " m; heading RMS " << std::setprecision(2)
                      << stats.heading_rms / kRadiansPerDegree << " deg\n";
        };
        std::cout << "EKF estimate:   ";
        errors(summary.estimate);
        std::cout << "Dead reckoning: ";
        errors(summary.odometry);
        std::cout << "EKF updates:    ";
        if (scans.empty()) {
            std::cout << "none (no LiDAR: prediction only)";
        } else {
            std::cout << summary.beams_used << " beams fused, " << summary.beams_rejected
                      << " rejected by the " << std::setprecision(1) << options.ekf.gate
                      << "-sigma gate";
        }
        std::cout << ", mean NEES " << std::setprecision(2) << summary.mean_nees
                  << " (3 = consistent)\n";
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
                  const std::vector<amr::LidarScan>& scans,
                  const std::vector<amr::LocalizationSample>& localization,
                  const amr::DynamicScenario* dynamic, std::string& error) {
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
    // lidar_scans.csv only exists with the LiDAR enabled; a stale one is removed.
    const std::filesystem::path lidar_file = dir / "lidar_scans.csv";
    if (options.lidar.enabled()) {
        if (!amr::writeLidarCsv(lidar_file, scans, error)) {
            return false;
        }
    } else {
        std::error_code ignored;
        std::filesystem::remove(lidar_file, ignored);
    }
    // localization.csv only exists with --ekf; a stale one is removed.
    const std::filesystem::path localization_file = dir / "localization.csv";
    if (options.ekf.enabled) {
        if (!amr::writeLocalizationCsv(localization_file, localization, error)) {
            return false;
        }
    } else {
        std::error_code ignored;
        std::filesystem::remove(localization_file, ignored);
    }
    // The dynamic-obstacle files only exist with --dynamic-obstacles; stale ones are removed.
    const std::filesystem::path dynamic_files[] = {dir / "dynamic_obstacles.csv",
                                                   dir / "replans.csv",
                                                   dir / "replanned_paths.csv"};
    if (dynamic != nullptr) {
        if (!amr::writeDynamicObstaclesCsv(dynamic_files[0], map, sim.trajectory,
                                           dynamic->obstacles, options.replanning.sensing_range,
                                           error) ||
            !amr::writeReplansCsv(dynamic_files[1], dynamic_files[2], sim.replans, error)) {
            return false;
        }
    } else {
        for (const auto& file : dynamic_files) {
            std::error_code ignored;
            std::filesystem::remove(file, ignored);
        }
    }
    return amr::writePathCsv(dir / "path.csv", map, plan.path, error) &&
           amr::writePlanningMetricsCsv(dir / "planning_metrics.csv", map, options.start,
                                        options.goal, plan, reference.result,
                                        options.motion.robot_radius, inflation.time_ms,
                                        options.connectivity, error) &&
           amr::writeTrajectoryCsv(dir / "trajectory.csv", sim.trajectory,
                                   options.motion.model == amr::RobotModel::DiffDrive, error) &&
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
        amr::planWithFootprint(map, inflation.planning_map, options.start, options.goal,
                               options.connectivity);
    // The BFS check runs after planning, on the same planning map.
    const ReferenceCheck reference = options.reference_bfs
                                         ? checkAgainstBfs(inflation.planning_map, options, plan)
                                         : ReferenceCheck{};
    // Moving obstacles: the initial plan ignores them (the robot has not seen them
    // yet); the simulator replans on the inflated map when one blocks the path.
    std::optional<amr::DynamicScenario> dynamic;
    if (!options.dynamic_file.empty()) {
        amr::DynamicObstacleLoadResult obstacles =
            amr::loadDynamicObstacles(options.dynamic_file, map);
        if (!obstacles.ok) {
            std::cerr << "Error: " << obstacles.error << '\n';
            return kExitMapError;
        }
        dynamic.emplace();
        dynamic->planning_map = &inflation.planning_map;
        dynamic->obstacles = std::move(obstacles.obstacles);
        dynamic->replanning = options.replanning;
    }
    const amr::DynamicScenario* scenario = dynamic ? &*dynamic : nullptr;
    amr::SimulationResult sim;  // stays NOT_RUN with no trajectory if planning failed
    if (plan.success()) {
        sim = amr::simulatePath(map, plan.path, options.motion, options.connectivity, scenario);
    }
    // The LiDAR observes the real map from every trajectory sample; it does not
    // influence the motion.
    const std::vector<amr::LidarScan> scans =
        amr::scanTrajectory(map, sim.trajectory, options.lidar);
    // The EKF runs alongside as well: it estimates the pose from noisy odometry and
    // the scans, but the robot is controlled with its true pose.
    const std::vector<amr::LocalizationSample> localization =
        amr::runLocalization(map, sim.trajectory, scans, options.lidar, options.ekf);

    std::error_code ec;
    std::filesystem::create_directories(options.output_dir, ec);
    if (ec) {
        std::cerr << "Error: cannot create output directory '" << options.output_dir.string()
                  << "': " << ec.message() << '\n';
        return kExitOutputError;
    }
    if (!writeResults(options, map, inflation, plan, reference, sim, scans, localization, scenario,
                      error)) {
        std::cerr << "Error: " << error << '\n';
        return kExitOutputError;
    }

    printSummary(options, map, inflation, plan, reference, sim, scans, localization, scenario);
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
