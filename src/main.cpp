// robot_simulator: command-line entry point.
//
// Current scope (Phases 1-2): load and validate a map, plan an A* path,
// export path.csv and planning_metrics.csv, print a summary.
#include <charconv>
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
#include "grid_map.hpp"
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
};

constexpr double kDefaultResolution = 0.5;  // meters per cell

struct CliOptions {
    std::string map_file;
    amr::GridCoord start;
    amr::GridCoord goal;
    bool has_start = false;
    bool has_goal = false;
    double resolution = kDefaultResolution;
    std::filesystem::path output_dir = "results";
    bool show_help = false;
};

void printUsage(std::ostream& os) {
    os << "Usage: robot_simulator --map FILE --start X Y --goal X Y [options]\n"
          "\n"
          "Plans a collision-free path on an occupancy-grid map using A*.\n"
          "\n"
          "Required:\n"
          "  --map FILE          Map CSV file (0 = free, 1 = occupied, row 0 = top)\n"
          "  --start X Y         Start cell (x = column, y = row, origin upper-left)\n"
          "  --goal X Y          Goal cell\n"
          "\n"
          "Options:\n"
          "  --resolution M      Cell size in meters (default 0.5)\n"
          "  --output DIR        Output directory (default results/)\n"
          "  -h, --help          Show this help and exit\n"
          "\n"
          "Outputs:\n"
          "  DIR/path.csv              waypoint_index,grid_x,grid_y,world_x,world_y\n"
          "  DIR/planning_metrics.csv  status, timing, expanded nodes, path length\n"
          "\n"
          "Exit codes:\n"
          "  0 success, 1 usage error, 2 map error, 3 invalid start/goal,\n"
          "  4 no path exists, 5 output write error\n";
}

bool parseInt(const std::string& text, int& value) {
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto [ptr, ec] = std::from_chars(begin, end, value);
    return ec == std::errc() && ptr == end;
}

bool parsePositiveDouble(const std::string& text, double& value) {
    if (text.empty()) {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (end != text.c_str() + text.size() || !std::isfinite(parsed) || parsed <= 0.0) {
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

        if (arg == "-h" || arg == "--help") {
            options.show_help = true;
            return options;
        } else if (arg == "--map") {
            if (!requireValues(1)) {
                return std::nullopt;
            }
            options.map_file = argv[++i];
        } else if (arg == "--start") {
            if (!parseCoord(options.start)) {
                return std::nullopt;
            }
            options.has_start = true;
        } else if (arg == "--goal") {
            if (!parseCoord(options.goal)) {
                return std::nullopt;
            }
            options.has_goal = true;
        } else if (arg == "--resolution") {
            if (!requireValues(1)) {
                return std::nullopt;
            }
            const std::string text = argv[++i];
            if (!parsePositiveDouble(text, options.resolution)) {
                error = "--resolution expects a positive number, got '" + text + "'";
                return std::nullopt;
            }
        } else if (arg == "--output") {
            if (!requireValues(1)) {
                return std::nullopt;
            }
            options.output_dir = argv[++i];
        } else {
            error = "unknown option '" + arg + "'";
            return std::nullopt;
        }
    }

    if (options.map_file.empty()) {
        error = "missing required option --map";
    } else if (!options.has_start) {
        error = "missing required option --start";
    } else if (!options.has_goal) {
        error = "missing required option --goal";
    }
    if (!error.empty()) {
        return std::nullopt;
    }
    return options;
}

void printSummary(const CliOptions& options, const amr::GridMap& map,
                  const amr::PlanResult& result) {
    const amr::WorldPoint start_w = map.gridToWorld(options.start);
    const amr::WorldPoint goal_w = map.gridToWorld(options.goal);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Map:            " << options.map_file << " (" << map.width() << " x "
              << map.height() << " cells, " << map.resolution() << " m/cell)\n";
    std::cout << "Start:          cell " << options.start << ", world (" << start_w.x << ", "
              << start_w.y << ") m\n";
    std::cout << "Goal:           cell " << options.goal << ", world (" << goal_w.x << ", "
              << goal_w.y << ") m\n";
    std::cout << "Planner status: " << amr::toString(result.status) << '\n';
    std::cout << "Expanded nodes: " << result.expanded_nodes << '\n';
    std::cout << std::setprecision(3) << "Planning time:  " << result.planning_time_ms
              << " ms\n";
    if (result.success()) {
        std::cout << std::setprecision(2) << "Path:           " << result.path.size()
                  << " waypoints, " << result.path_cost << " steps, "
                  << amr::pathLengthMeters(map, result.path) << " m\n";
    } else {
        std::cout << "Path:           none\n";
    }
    std::cout << "Results:        " << options.output_dir.string() << '\n';
}

int exitCodeFor(amr::PlanStatus status) {
    switch (status) {
        case amr::PlanStatus::Success:
            return kExitSuccess;
        case amr::PlanStatus::NoPath:
            return kExitNoPath;
        case amr::PlanStatus::StartOutOfBounds:
        case amr::PlanStatus::GoalOutOfBounds:
        case amr::PlanStatus::StartOccupied:
        case amr::PlanStatus::GoalOccupied:
            return kExitInvalidRequest;
    }
    return kExitInvalidRequest;
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

    const amr::PlanResult result = amr::planAStar(map, options.start, options.goal);

    std::error_code ec;
    std::filesystem::create_directories(options.output_dir, ec);
    if (ec) {
        std::cerr << "Error: cannot create output directory '" << options.output_dir.string()
                  << "': " << ec.message() << '\n';
        return kExitOutputError;
    }
    // path.csv is always rewritten (header-only on failure) so a stale path
    // from an earlier run can never be mistaken for the current result.
    if (!amr::writePathCsv(options.output_dir / "path.csv", map, result.path, error) ||
        !amr::writePlanningMetricsCsv(options.output_dir / "planning_metrics.csv", map,
                                      options.start, options.goal, result, error)) {
        std::cerr << "Error: " << error << '\n';
        return kExitOutputError;
    }

    printSummary(options, map, result);
    if (!result.success()) {
        std::cerr << "Planning failed: " << amr::toString(result.status) << '\n';
    }
    return exitCodeFor(result.status);
}
