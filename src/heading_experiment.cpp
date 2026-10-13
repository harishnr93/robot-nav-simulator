// heading_experiment: P vs. PID heading control on a heading plant with actuator
// lag and an optional disturbance (see heading_control.hpp).
//
// Scenarios (each run with the P and with the PID controller):
//   step              90 deg step of the reference
//   step_disturbance  90 deg step with a constant disturbance turn rate
//   ramp              reference turning at a constant rate (following an arc)
//   path_profile      the heading profile of a planned path: the direction of each
//                     straight run of an 8-connected A* path, held for the time the
//                     robot needs to drive it
//
// Writes, to --output (default results/heading/):
//   heading_<scenario>_<p|pid>.csv  time,reference,theta,error,command,omega
//   heading_metrics.csv             one row per scenario and controller
// and prints a summary table. --sweep prints the PID gain grid used to choose the
// default gains instead.
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "astar.hpp"
#include "grid_map.hpp"
#include "heading_control.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kStepSize = kPi / 2;        // 90 deg
constexpr double kSettlingBandFraction = 0.02;
constexpr double kSteadyWindow = 1.0;        // s
constexpr double kPathSpeed = 0.5;           // m/s, as in the navigation simulator

struct Options {
    amr::HeadingGains p_gains{4.0, 0.0, 0.0};
    amr::HeadingGains pid_gains{4.0, 2.0, 0.4};
    double tau = 0.25;
    double disturbance = 0.3;
    double ramp_rate = 0.5;
    double max_omega = 1.5;
    double dt = 0.01;
    double duration = 10.0;
    std::string map_file = "maps/obstacles.csv";
    std::filesystem::path output_dir = "results/heading";
    bool sweep = false;
    bool help = false;
};

void printUsage() {
    std::cout <<
        "Usage: heading_experiment [options]\n"
        "\n"
        "P vs. PID heading control on a heading with actuator lag (tau) and a constant\n"
        "disturbance turn rate. Runs four scenarios and writes CSV files + metrics.\n"
        "\n"
        "Options (defaults in brackets):\n"
        "  --kp K           proportional gain of both controllers, 1/s [4]\n"
        "  --ki K           PID integral gain, 1/s^2 [2]\n"
        "  --kd K           PID derivative gain (on the measured heading) [0.4]\n"
        "  --tau S          actuator time constant, s [0.25]\n"
        "  --disturbance W  disturbance turn rate in step_disturbance, rad/s [0.3]\n"
        "  --ramp-rate W    reference turn rate in the ramp scenario, rad/s [0.5]\n"
        "  --max-angular-speed W  command limit, rad/s [1.5]\n"
        "  --dt S           control period, s [0.01]\n"
        "  --duration S     length of the step and ramp runs, s [10]\n"
        "  --map FILE       map for the path_profile scenario [maps/obstacles.csv]\n"
        "  --output DIR     output directory [results/heading]\n"
        "  --sweep          print the PID gain grid (Kp fixed) instead of running\n"
        "  -h, --help       show this help\n";
}

bool parseDouble(const std::string& text, double& value) {
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (text.empty() || end != text.c_str() + text.size() || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

std::optional<Options> parseArguments(int argc, char** argv, std::string& error) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto number = [&](double& value) {
            if (i + 1 >= argc || !parseDouble(argv[++i], value)) {
                error = "option " + arg + " expects a number";
                return false;
            }
            return true;
        };
        bool ok = true;
        if (arg == "-h" || arg == "--help") {
            o.help = true;
        } else if (arg == "--kp") {
            ok = number(o.p_gains.kp);
            o.pid_gains.kp = o.p_gains.kp;
        } else if (arg == "--ki") {
            ok = number(o.pid_gains.ki);
        } else if (arg == "--kd") {
            ok = number(o.pid_gains.kd);
        } else if (arg == "--tau") {
            ok = number(o.tau);
        } else if (arg == "--disturbance") {
            ok = number(o.disturbance);
        } else if (arg == "--ramp-rate") {
            ok = number(o.ramp_rate);
        } else if (arg == "--max-angular-speed") {
            ok = number(o.max_omega);
        } else if (arg == "--dt") {
            ok = number(o.dt);
        } else if (arg == "--duration") {
            ok = number(o.duration);
        } else if (arg == "--map" && i + 1 < argc) {
            o.map_file = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            o.output_dir = argv[++i];
        } else if (arg == "--sweep") {
            o.sweep = true;
        } else {
            error = "unknown or incomplete option '" + arg + "'";
            ok = false;
        }
        if (!ok) {
            return std::nullopt;
        }
    }
    if (o.p_gains.kp <= 0 || o.pid_gains.ki < 0 || o.pid_gains.kd < 0 || o.tau < 0 ||
        o.max_omega <= 0 || o.dt <= 0 || o.duration <= 0 || o.duration / o.dt > 1e7) {
        error = "gains must be non-negative (kp > 0), tau >= 0, and max angular speed, dt "
                "and duration positive";
        return std::nullopt;
    }
    return o;
}

/// Heading reference of a planned path: the direction of each straight run, held
/// for the time needed to drive it at kPathSpeed. Returns {times, headings}.
std::optional<std::pair<std::vector<double>, std::vector<double>>> pathProfile(
    const Options& o, std::string& error) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(o.map_file, 0.5);
    if (!loaded.ok) {
        error = loaded.error;
        return std::nullopt;
    }
    const amr::GridMap& map = loaded.map;
    const amr::GridCoord start{1, 1};
    const amr::GridCoord goal{map.width() - 2, map.height() - 2};
    const amr::PlanResult plan = amr::planAStar(map, start, goal, amr::Connectivity::Eight);
    if (!plan.success()) {
        error = "no 8-connected path from (1, 1) to the opposite corner on " + o.map_file;
        return std::nullopt;
    }
    std::vector<double> times{0.0};
    std::vector<double> headings;
    double run_length = 0.0;
    for (std::size_t i = 1; i < plan.path.size(); ++i) {
        const int dx = plan.path[i].x - plan.path[i - 1].x;
        const int dy = plan.path[i].y - plan.path[i - 1].y;
        const double heading = std::atan2(dy, dx);
        if (!headings.empty() && std::fabs(heading - headings.back()) > 1e-12) {
            times.push_back(times.back() + run_length / kPathSpeed);  // a new straight run
            run_length = 0.0;
        }
        if (headings.empty() || std::fabs(heading - headings.back()) > 1e-12) {
            headings.push_back(heading);
        }
        run_length += std::hypot(dx, dy) * map.resolution();
    }
    times.push_back(times.back() + run_length / kPathSpeed);
    return std::make_pair(times, headings);
}

struct Scenario {
    std::string name;
    std::function<double(double)> reference;
    double disturbance;
    double step_size;  // 0 for non-step references
    double duration;
};

bool writeRun(const std::filesystem::path& file, const std::vector<amr::HeadingSample>& samples) {
    std::ofstream out(file);
    out << "time,reference,theta,error,command,omega\n" << std::fixed << std::setprecision(9);
    for (const auto& s : samples) {
        out << s.time << ',' << s.reference << ',' << s.theta << ',' << s.error << ','
            << s.command << ',' << s.omega << '\n';
    }
    return static_cast<bool>(out);
}

std::string formatPercent(double value) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(2) << value << '%';
    return os.str();
}

std::string formatTime(double t) {
    if (t < 0.0) {
        return "never";
    }
    std::ostringstream os;
    os << std::fixed << std::setprecision(2) << t << " s";
    return os.str();
}

void runSweep(const Options& o) {
    std::cout << "PID gain grid, Kp = " << o.p_gains.kp << " (90 deg step, tau = " << o.tau
              << " s; 'disturbed' adds " << o.disturbance << " rad/s)\n"
              << "  Ki   Kd | nominal OS  settling | disturbed OS  settling | ramp IAE\n";
    for (double ki = 0.5; ki <= 4.0 + 1e-9; ki += 0.5) {
        for (double kd = 0.0; kd <= 1.2 + 1e-9; kd += 0.2) {
            const amr::HeadingGains gains{o.p_gains.kp, ki, kd};
            auto run = [&](double d, const std::function<double(double)>& ref, double step) {
                amr::HeadingPlant plant{o.tau, d, o.max_omega};
                amr::HeadingController c(amr::HeadingControllerKind::PID, gains, o.max_omega);
                const auto s = amr::simulateHeadingLoop(ref, plant, c, 0.0, o.dt, 15.0);
                return amr::computeHeadingMetrics(s, step, kSettlingBandFraction * std::fabs(step),
                                                  kSteadyWindow, 0.0, o.max_omega);
            };
            const auto a = run(0.0, [](double) { return kStepSize; }, kStepSize);
            const auto b = run(o.disturbance, [](double) { return kStepSize; }, kStepSize);
            const auto c = run(0.0, [&](double t) { return o.ramp_rate * t; }, 0.0);
            std::cout << std::fixed << std::setprecision(1) << std::setw(5) << ki << std::setw(5)
                      << kd << std::setprecision(2) << " | " << std::setw(7) << a.overshoot_pct
                      << " %  " << std::setw(8) << formatTime(a.settling_time) << " | "
                      << std::setw(9) << b.overshoot_pct << " %  " << std::setw(8)
                      << formatTime(b.settling_time) << " | " << std::setprecision(3) << c.iae
                      << '\n';
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string error;
    const auto parsed = parseArguments(argc, argv, error);
    if (!parsed) {
        std::cerr << "Error: " << error << "\nRun with --help for usage.\n";
        return 1;
    }
    const Options& o = *parsed;
    if (o.help) {
        printUsage();
        return 0;
    }
    if (o.sweep) {
        runSweep(o);
        return 0;
    }
    const auto profile = pathProfile(o, error);
    if (!profile) {
        std::cerr << "Error: " << error << '\n';
        return 2;
    }
    const auto& [switch_times, headings] = *profile;
    const auto path_reference = [&](double t) {
        std::size_t i = 0;
        while (i + 1 < headings.size() && t >= switch_times[i + 1]) {
            ++i;
        }
        return headings[i];
    };

    const std::vector<Scenario> scenarios{
        {"step", [](double) { return kStepSize; }, 0.0, kStepSize, o.duration},
        {"step_disturbance", [](double) { return kStepSize; }, o.disturbance, kStepSize,
         o.duration},
        {"ramp", [&](double t) { return o.ramp_rate * t; }, 0.0, 0.0, o.duration},
        {"path_profile", path_reference, 0.0, 0.0, switch_times.back() + 2.0},
    };

    std::error_code ec;
    std::filesystem::create_directories(o.output_dir, ec);
    if (ec) {
        std::cerr << "Error: cannot create '" << o.output_dir.string() << "'\n";
        return 3;
    }
    std::ofstream metrics_file(o.output_dir / "heading_metrics.csv");
    metrics_file << "scenario,controller,kp,ki,kd,tau_s,disturbance_radps,max_omega_radps,dt_s,"
                    "duration_s,overshoot_pct,rise_time_s,settling_time_s,"
                    "steady_state_error_rad,iae_rad_s,rms_error_rad,effort_abs_rad,"
                    "effort_sq_rad2_per_s,command_variation_radps,saturated_fraction\n";

    std::cout << "Heading control: tau " << o.tau << " s, |u| <= " << o.max_omega
              << " rad/s, dt " << o.dt << " s; P: Kp " << o.p_gains.kp << "; PID: Kp "
              << o.pid_gains.kp << ", Ki " << o.pid_gains.ki << ", Kd " << o.pid_gains.kd << "\n\n"
              << std::left << std::setw(18) << "scenario" << std::setw(5) << "ctrl"
              << std::right << std::setw(10) << "overshoot" << std::setw(11) << "settling"
              << std::setw(14) << "steady err" << std::setw(10) << "IAE" << std::setw(9)
              << "RMS" << std::setw(10) << "effort" << '\n';
    for (const Scenario& scenario : scenarios) {
        for (const auto kind : {amr::HeadingControllerKind::P, amr::HeadingControllerKind::PID}) {
            const amr::HeadingGains gains =
                kind == amr::HeadingControllerKind::P ? o.p_gains : o.pid_gains;
            amr::HeadingPlant plant{o.tau, scenario.disturbance, o.max_omega};
            amr::HeadingController controller(kind, gains, o.max_omega);
            const auto samples = amr::simulateHeadingLoop(scenario.reference, plant, controller,
                                                          0.0, o.dt, scenario.duration);
            const auto m = amr::computeHeadingMetrics(
                samples, scenario.step_size, kSettlingBandFraction * std::fabs(scenario.step_size),
                kSteadyWindow, 0.0, o.max_omega);
            const std::string tag = kind == amr::HeadingControllerKind::P ? "p" : "pid";
            if (!writeRun(o.output_dir / ("heading_" + scenario.name + "_" + tag + ".csv"),
                          samples)) {
                std::cerr << "Error: cannot write results\n";
                return 3;
            }
            const bool step = scenario.step_size != 0.0;
            metrics_file << scenario.name << ',' << amr::toString(kind) << ',' << gains.kp << ','
                         << gains.ki << ',' << gains.kd << ',' << o.tau << ','
                         << scenario.disturbance << ',' << o.max_omega << ',' << o.dt << ','
                         << scenario.duration << ',' << std::setprecision(6) << std::fixed
                         << (step ? std::to_string(m.overshoot_pct) : "") << ','
                         << (step && m.rise_time >= 0 ? std::to_string(m.rise_time) : "") << ','
                         << (step && m.settling_time >= 0 ? std::to_string(m.settling_time) : "")
                         << ',' << m.steady_state_error << ',' << m.iae << ',' << m.rms_error
                         << ',' << m.effort_abs << ',' << m.effort_sq << ','
                         << m.command_variation << ',' << m.saturated_fraction << '\n'
                         << std::defaultfloat;
            std::cout << std::left << std::setw(18) << scenario.name << std::setw(5)
                      << amr::toString(kind) << std::right << std::fixed << std::setprecision(2)
                      << std::setw(10) << (step ? formatPercent(m.overshoot_pct) : "-")
                      << std::setw(11) << (step ? formatTime(m.settling_time) : "-")
                      << std::setw(10) << m.steady_state_error * 180.0 / kPi << " deg"
                      << std::setprecision(3) << std::setw(10) << m.iae << std::setw(9)
                      << m.rms_error << std::setw(10) << m.effort_abs << '\n'
                      << std::defaultfloat;
        }
    }
    if (!metrics_file) {
        std::cerr << "Error: cannot write heading_metrics.csv\n";
        return 3;
    }
    std::cout << "\nResults: " << o.output_dir.string() << '\n';
    return 0;
}
