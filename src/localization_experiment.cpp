// localization_experiment: global localization, the kidnapped robot, symmetric maps,
// planning for localization and steering by the estimate.
//
//   localization_experiment global [options]    particle filter from no prior knowledge:
//                                               how often and how fast it finds the robot
//   localization_experiment kidnap [options]    the robot is carried elsewhere mid-run:
//                                               particle filter with and without recovery,
//                                               against the EKF
//
// Options: --runs N, --seed N, --beams N (LiDAR, default 72), --noise M (default 0.02),
//          --particles N (maximum, default 5000), --map FILE (default maps/obstacles.csv),
//          --output DIR (default results/localization).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "ekf.hpp"
#include "footprint.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "closed_loop.hpp"
#include "localization_planning.hpp"
#include "particle_filter.hpp"
#include "simulator.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Options {
    std::string mode;
    int runs = 50;
    unsigned seed = 1;
    int beams = 72;
    double noise = 0.02;
    double range = 5.0;
    std::size_t particles = 5000;
    double sigma_hit = -1.0;    ///< override ParticleFilterConfig::sigma_hit (< 0: default)
    double plan_weight = -1.0;  ///< steering: localization-aware planning weight (< 0: shortest path)
    double correlation = -1.0;  ///< override ParticleFilterConfig::correlation
    std::string map = "maps/obstacles.csv";
    std::filesystem::path output = "results/localization";
};

/// A random differential-drive route of at least `min_length` m between free cells of
/// the map (footprint 0.25 m), starting with a random heading. Empty if none found.
std::vector<amr::TrajectorySample> randomRoute(const amr::GridMap& map, std::mt19937& rng,
                                               double min_length = 6.0,
                                               const amr::GridCoord* forced_start = nullptr,
                                               bool (*allowed)(amr::GridCoord) = nullptr) {
    const amr::GridMap inflated = amr::inflateObstacles(map, 0.25);
    std::uniform_int_distribution<int> xs(0, map.width() - 1);
    std::uniform_int_distribution<int> ys(0, map.height() - 1);
    for (int attempt = 0; attempt < 500; ++attempt) {
        const amr::GridCoord start = forced_start ? *forced_start : amr::GridCoord{xs(rng), ys(rng)};
        const amr::GridCoord goal{xs(rng), ys(rng)};
        if (!inflated.isFree(start) || !inflated.isFree(goal) ||
            (allowed && (!allowed(start) || !allowed(goal)))) {
            continue;
        }
        const amr::WorldPoint a = map.gridToWorld(start);
        const amr::WorldPoint b = map.gridToWorld(goal);
        if (std::hypot(a.x - b.x, a.y - b.y) < min_length) {
            continue;
        }
        const amr::PlanResult plan = amr::planWithFootprint(map, inflated, start, goal);
        if (!plan.success()) {
            continue;
        }
        amr::MotionConfig motion;
        motion.model = amr::RobotModel::DiffDrive;
        motion.robot_radius = 0.25;
        motion.initial_heading = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
        const amr::SimulationResult sim = amr::simulatePath(map, plan.path, motion);
        if (sim.success()) {
            return sim.trajectory;
        }
    }
    return {};
}

double positionError(const amr::Pose& a, const amr::Pose& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double headingError(const amr::Pose& a, const amr::Pose& b) {
    return std::fabs(amr::wrapAngle(a.theta - b.theta));
}

amr::ParticleFilterConfig pfConfig(const Options& o) {
    amr::ParticleFilterConfig config;
    if (o.sigma_hit > 0.0) {
        config.sigma_hit = o.sigma_hit;
    }
    if (o.correlation > 0.0) {
        config.correlation = o.correlation;
    }
    return config;
}

/// First time from which the estimate stays within the thresholds until the end, or
/// a negative value if it is not within them at the end.
template <typename Error>
double settleTime(const std::vector<double>& times, const Error& within, std::size_t from = 0) {
    double settled = -1.0;
    for (std::size_t k = from; k < times.size(); ++k) {
        if (within(k)) {
            if (settled < 0.0) {
                settled = times[k];
            }
        } else {
            settled = -1.0;
        }
    }
    return settled;
}

constexpr double kGoodPosition = 0.3;           // m
constexpr double kGoodHeading = 10.0 * kPi / 180.0;  // rad

amr::LidarConfig lidarFor(const Options& o, std::mt19937& rng) {
    amr::LidarConfig lidar;
    lidar.beams = o.beams;
    lidar.noise_stddev = o.noise;
    lidar.max_range = o.range;
    lidar.seed = static_cast<std::uint32_t>(rng());
    return lidar;
}

double median(std::vector<double> v) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<std::size_t>(q * static_cast<double>(v.size())))];
}

// ---------------------------------------------------------------------------------
// Global localization

int modeGlobal(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(o.map, 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << "\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    std::mt19937 rng(o.seed);
    struct Row {
        std::size_t particles;
        int found = 0, runs = 0;
        std::vector<double> times, rms, counts;
        double seconds = 0.0, updates = 0.0;
    };
    std::vector<Row> rows;
    for (const std::size_t n : {500, 1000, 2000, 5000}) {
        if (n <= o.particles) {
            rows.push_back({n, 0, 0, {}, {}, {}, 0.0, 0.0});
        }
    }
    for (int run = 0; run < o.runs; ++run) {
        const auto trajectory = randomRoute(map, rng, 8.0);
        if (trajectory.empty()) {
            continue;
        }
        const amr::LidarConfig lidar = lidarFor(o, rng);
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        amr::OdometryNoise odometry;
        odometry.seed = static_cast<std::uint32_t>(rng());
        const auto pf_seed = static_cast<std::uint32_t>(rng());
        for (Row& row : rows) {
            amr::ParticleFilterConfig config = pfConfig(o);
            config.max_particles = row.particles;
            config.seed = pf_seed;
            const auto t0 = std::chrono::steady_clock::now();
            const auto samples = amr::runParticleFilter(map, trajectory, scans, lidar, odometry, config, true);
            row.seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            row.updates += static_cast<double>(samples.size());
            std::vector<double> times;
            for (const auto& s : samples) {
                times.push_back(s.time);
            }
            const double t = settleTime(times, [&](std::size_t k) {
                return positionError(samples[k].truth, samples[k].estimate.pose) < kGoodPosition &&
                       headingError(samples[k].truth, samples[k].estimate.pose) < kGoodHeading;
            });
            ++row.runs;
            if (t >= 0.0) {
                ++row.found;
                row.times.push_back(t);
                double sq = 0.0, n = 0.0, count = 0.0;
                for (const auto& s : samples) {
                    if (s.time >= t) {
                        sq += std::pow(positionError(s.truth, s.estimate.pose), 2);
                        count += static_cast<double>(s.particles);
                        n += 1.0;
                    }
                }
                row.rms.push_back(std::sqrt(sq / n));
                row.counts.push_back(count / n);
            }
        }
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "localization_global.csv");
    csv << "max_particles,runs,found,median_time_s,p90_time_s,median_rms_m,mean_particles_after,ms_per_update\n";
    std::cout << "Global localization (" << o.map << ", " << o.beams << " beams, noise " << o.noise
              << " m, routes of 8+ m)\n"
              << std::setw(10) << "particles" << std::setw(10) << "found" << std::setw(14) << "time median"
              << std::setw(10) << "p90" << std::setw(14) << "RMS after cm" << std::setw(16)
              << "particles after" << std::setw(12) << "ms/update" << "\n";
    for (const Row& r : rows) {
        double mean_count = 0.0;
        for (const double c : r.counts) {
            mean_count += c / static_cast<double>(std::max<std::size_t>(1, r.counts.size()));
        }
        const double ms = 1000.0 * r.seconds / std::max(1.0, r.updates);
        std::cout << std::setw(10) << r.particles << std::setw(9) << std::fixed << std::setprecision(1)
                  << 100.0 * r.found / std::max(1, r.runs) << "%" << std::setw(12) << median(r.times) << " s"
                  << std::setw(8) << percentile(r.times, 0.9) << " s" << std::setw(14) << std::setprecision(2)
                  << 100.0 * median(r.rms) << std::setw(16) << std::setprecision(0) << mean_count << std::setw(12)
                  << std::setprecision(2) << ms << "\n";
        csv << r.particles << ',' << r.runs << ',' << r.found << ',' << median(r.times) << ','
            << percentile(r.times, 0.9) << ',' << median(r.rms) << ',' << mean_count << ',' << ms << "\n";
    }
    return 0;
}

// ---------------------------------------------------------------------------------
// Kidnapped robot

int modeKidnap(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(o.map, 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << "\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    std::mt19937 rng(o.seed);
    struct Row {
        std::string name;
        int recovered = 0, runs = 0;
        std::vector<double> times;
        double before_rms_sq = 0.0, before_n = 0.0;
        int false_alarms = 0;  // global restarts before the kidnapping
    };
    std::vector<Row> rows = {{"particle filter, recovery", 0, 0, {}, 0, 0, 0},
                             {"particle filter, no recovery", 0, 0, {}, 0, 0, 0},
                             {"EKF", 0, 0, {}, 0, 0, 0}};
    for (int run = 0; run < o.runs; ++run) {
        auto first = randomRoute(map, rng, 6.0);
        if (first.empty()) {
            continue;
        }
        // Carried away: a new route from a random free cell at least 3 m away.
        const amr::RobotState& end = first.back().state;
        std::vector<amr::TrajectorySample> second;
        for (int attempt = 0; attempt < 50 && second.empty(); ++attempt) {
            auto candidate = randomRoute(map, rng, 6.0);
            if (!candidate.empty() &&
                std::hypot(candidate.front().state.x - end.x, candidate.front().state.y - end.y) >= 3.0) {
                second = std::move(candidate);
            }
        }
        if (second.empty()) {
            continue;
        }
        std::vector<amr::TrajectorySample> trajectory = first;
        std::vector<bool> kidnapped(first.size(), false);
        const double offset = end.time + 0.1;
        const std::size_t kidnap = trajectory.size();
        for (std::size_t k = 0; k < second.size(); ++k) {
            amr::TrajectorySample s = second[k];
            s.state.time += offset;
            if (k == 0) {
                s.linear_velocity = 0.0;
                s.angular_velocity = 0.0;
            }
            trajectory.push_back(s);
            kidnapped.push_back(k == 0);
        }
        const amr::LidarConfig lidar = lidarFor(o, rng);
        const auto scans = amr::scanTrajectory(map, trajectory, lidar);
        amr::OdometryNoise odometry;
        odometry.seed = static_cast<std::uint32_t>(rng());
        const auto pf_seed = static_cast<std::uint32_t>(rng());
        std::vector<double> times;
        std::vector<amr::Pose> truth, estimate;
        for (const auto& s : trajectory) {
            times.push_back(s.state.time);
            truth.push_back({s.state.x, s.state.y, s.state.theta});
        }
        for (std::size_t v = 0; v < rows.size(); ++v) {
            estimate.clear();
            if (v < 2) {
                amr::ParticleFilterConfig config = pfConfig(o);
                config.max_particles = o.particles;
                config.recover = v == 0;
                config.seed = pf_seed;
                const auto samples =
                    amr::runParticleFilter(map, trajectory, scans, lidar, odometry, config, false, kidnapped);
                for (const auto& s : samples) {
                    estimate.push_back(s.estimate.pose);
                }
                rows[v].false_alarms += samples[kidnap - 1].relocalizations > 0 ? 1 : 0;
            } else {
                amr::EkfConfig config;
                config.enabled = true;
                config.odometry = odometry;
                config.range_sigma = o.noise;
                for (const auto& s : amr::runLocalization(map, trajectory, scans, lidar, config, kidnapped)) {
                    estimate.push_back(s.estimate);
                }
            }
            Row& row = rows[v];
            ++row.runs;
            for (std::size_t k = 0; k < kidnap; ++k) {
                row.before_rms_sq += std::pow(positionError(truth[k], estimate[k]), 2);
                row.before_n += 1.0;
            }
            const double t = settleTime(times, [&](std::size_t k) {
                return positionError(truth[k], estimate[k]) < kGoodPosition &&
                       headingError(truth[k], estimate[k]) < kGoodHeading;
            }, kidnap);
            if (t >= 0.0) {
                ++row.recovered;
                row.times.push_back(t - offset);
            }
        }
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "localization_kidnap.csv");
    csv << "filter,runs,recovered,median_recovery_s,p90_recovery_s,rms_before_m,false_alarms\n";
    std::cout << "Kidnapped robot (" << o.map << ", carried 3+ m without odometry, " << o.particles
              << " particles max)\n"
              << std::left << std::setw(32) << "filter" << std::right << std::setw(11) << "recovered"
              << std::setw(16) << "time median" << std::setw(10) << "p90" << std::setw(18)
              << "RMS before cm" << std::setw(14) << "false alarms" << "\n";
    for (const Row& r : rows) {
        const double before = std::sqrt(r.before_rms_sq / std::max(1.0, r.before_n));
        std::cout << std::left << std::setw(32) << r.name << std::right << std::fixed << std::setprecision(1)
                  << std::setw(10) << 100.0 * r.recovered / std::max(1, r.runs) << "%" << std::setw(14)
                  << median(r.times) << " s" << std::setw(8) << percentile(r.times, 0.9) << " s"
                  << std::setw(18) << std::setprecision(2) << 100.0 * before << std::setw(14) << r.false_alarms
                  << "\n";
        csv << '"' << r.name << "\"," << r.runs << ',' << r.recovered << ',' << median(r.times) << ','
            << percentile(r.times, 0.9) << ',' << before << ',' << r.false_alarms << "\n";
    }
    return 0;
}

// ---------------------------------------------------------------------------------
// Symmetric maps

/// West room of the twin-rooms maps (cells x 1-15, y 1-12).
bool inWestRoom(amr::GridCoord c) {
    return c.x >= 1 && c.x <= 15 && c.y >= 1 && c.y <= 12;
}

constexpr double kTwinShift = 8.5;  // m, east room = west room + 17 cells

/// Middle of the pillar corridors (cells x 10-33), away from the distinguishable ends.
bool inCorridorMiddle(amr::GridCoord c) {
    return c.x >= 10 && c.x <= 33 && c.y >= 1 && c.y <= 6;
}

enum class Outcome { Correct, Twin, Lost };

Outcome classify(const amr::Pose& truth, const amr::Pose& estimate, const amr::Pose& twin) {
    if (positionError(truth, estimate) < kGoodPosition && headingError(truth, estimate) < kGoodHeading) {
        return Outcome::Correct;
    }
    if (positionError(twin, estimate) < kGoodPosition && headingError(twin, estimate) < kGoodHeading) {
        return Outcome::Twin;
    }
    return Outcome::Lost;
}

int modeSymmetric(const Options& o) {
    std::mt19937 rng(o.seed);
    // Part 1: the EKF from an uncertain prior: matching places 8.5 m apart (twin rooms)
    // and 2 m apart (a corridor with pillars every 2 m), against maps without them.
    struct Case {
        std::string map;
        double shift;                   // m between matching places (along x)
        bool (*allowed)(amr::GridCoord);
    };
    const std::vector<Case> cases = {{"maps/twin_rooms.csv", kTwinShift, inWestRoom},
                                     {"maps/twin_rooms_different.csv", kTwinShift, inWestRoom},
                                     {"maps/pillar_corridor.csv", 2.0, inCorridorMiddle},
                                     {"maps/pillar_corridor_irregular.csv", 2.0, inCorridorMiddle}};
    std::cout << "EKF from an uncertain start (" << o.beams << " beams, prior heading std 0.1 rad)\n"
              << std::left << std::setw(32) << "map" << std::right << std::setw(12) << "prior std" << std::setw(10)
              << "correct" << std::setw(20) << "matching place" << std::setw(8) << "lost" << std::setw(22)
              << "std claimed when wrong" << "\n";
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "localization_symmetric_ekf.csv");
    csv << "map,prior_std_m,runs,correct,matching_place,lost,median_claimed_std_when_wrong_m\n";
    for (const Case& test : cases) {
        const amr::MapLoadResult loaded = amr::loadGridMapCsv(test.map, 0.5);
        if (!loaded.ok) {
            std::cerr << loaded.error << "\n";
            return 1;
        }
        const amr::GridMap& map = loaded.map;
        for (const double sigma : {0.25, 0.5, 1.0, 1.5, 2.0, 4.0}) {
            int counts[3] = {0, 0, 0};
            std::vector<double> claimed;
            int runs = 0;
            std::mt19937 local(o.seed + static_cast<unsigned>(sigma * 100));  // same routes for paired maps
            for (int run = 0; run < o.runs; ++run) {
                const auto trajectory = randomRoute(map, local, 4.0, nullptr, test.allowed);
                if (trajectory.empty()) {
                    continue;
                }
                const amr::LidarConfig lidar = lidarFor(o, local);
                const auto scans = amr::scanTrajectory(map, trajectory, lidar);
                amr::EkfConfig config;
                config.enabled = true;
                config.range_sigma = o.noise;
                config.odometry.seed = static_cast<std::uint32_t>(local());
                config.initial_position_sigma = sigma;
                config.initial_heading_sigma = 0.1;
                const auto samples = amr::runLocalization(map, trajectory, scans, lidar, config);
                const amr::Pose truth = samples.back().truth;
                const amr::Pose& estimate = samples.back().estimate;
                Outcome outcome = Outcome::Lost;
                if (positionError(truth, estimate) < kGoodPosition && headingError(truth, estimate) < kGoodHeading) {
                    outcome = Outcome::Correct;
                } else {
                    for (int k = -4; k <= 4; ++k) {
                        const amr::Pose image{truth.x + k * test.shift, truth.y, truth.theta};
                        if (k != 0 && positionError(image, estimate) < kGoodPosition &&
                            headingError(image, estimate) < kGoodHeading) {
                            outcome = Outcome::Twin;
                        }
                    }
                }
                ++counts[static_cast<int>(outcome)];
                if (outcome != Outcome::Correct) {
                    claimed.push_back(std::sqrt(std::max(samples.back().covariance[0], samples.back().covariance[4])));
                }
                ++runs;
            }
            const double n = std::max(1, runs);
            std::cout << std::left << std::setw(32) << test.map.substr(5) << std::right << std::fixed
                      << std::setprecision(2) << std::setw(10) << sigma << " m" << std::setw(9)
                      << std::setprecision(1) << 100.0 * counts[0] / n << "%" << std::setw(19) << 100.0 * counts[1] / n
                      << "%" << std::setw(7) << 100.0 * counts[2] / n << "%" << std::setw(20)
                      << std::setprecision(3) << (claimed.empty() ? 0.0 : median(claimed)) << " m\n";
            csv << test.map.substr(5) << ',' << sigma << ',' << runs << ',' << counts[0] << ',' << counts[1] << ','
                << counts[2] << ',' << (claimed.empty() ? 0.0 : median(claimed)) << "\n";
        }
    }

    // Part 2: the particle filter from no prior knowledge: does it keep both places?
    std::cout << "\nParticle filter, global start (" << o.particles << " particles max)\n"
              << std::left << std::setw(26) << "map" << std::right << std::setw(10) << "correct" << std::setw(16)
              << "twin (wrong)" << std::setw(8) << "lost" << std::setw(18) << "weight on both" << std::setw(18)
              << "true place lost" << std::setw(22) << "final weight on truth" << "\n";
    std::ofstream csv2(o.output / "localization_symmetric_pf.csv");
    csv2 << "map,runs,correct,twin,lost,mean_weight_on_truth_and_twin,runs_true_place_dropped,mean_final_weight_on_truth\n";
    for (const std::string name : {"maps/twin_rooms.csv", "maps/twin_rooms_marked.csv", "maps/point_symmetric.csv"}) {
        const amr::MapLoadResult loaded = amr::loadGridMapCsv(name, 0.5);
        const amr::GridMap& map = loaded.map;
        const bool point = name.find("point") != std::string::npos;
        const double w = map.width() * map.resolution();
        const double h = map.height() * map.resolution();
        const auto image = [&](const amr::Pose& p) {
            return point ? amr::Pose{w - p.x, h - p.y, amr::wrapAngle(p.theta + kPi)}
                         : amr::Pose{p.x + kTwinShift, p.y, p.theta};
        };
        int counts[3] = {0, 0, 0}, runs = 0, dropped = 0;
        double both = 0.0, both_n = 0.0, final_truth = 0.0;
        for (int run = 0; run < o.runs; ++run) {
            const auto trajectory = randomRoute(map, rng, 4.0, nullptr, point ? nullptr : inWestRoom);
            // (a route of 4+ m in the west room, or anywhere in the point-symmetric room)
            if (trajectory.empty()) {
                continue;
            }
            const amr::LidarConfig lidar = lidarFor(o, rng);
            const auto scans = amr::scanTrajectory(map, trajectory, lidar);
            amr::OdometryNoise odometry;
            odometry.seed = static_cast<std::uint32_t>(rng());
            amr::ParticleFilterConfig config;
            config.max_particles = o.particles;
            config.seed = static_cast<std::uint32_t>(rng());
            amr::ParticleFilter filter(map, config);
            filter.initializeGlobal();
            std::mt19937 noise(odometry.seed);
            std::normal_distribution<double> unit(0.0, 1.0);
            for (int i = 0; i < 3; ++i) {
                unit(noise);
            }
            const auto offsets = amr::beamOffsets(lidar);
            bool truth_dropped = false;
            double last_truth = 0.0;
            for (std::size_t k = 0; k < trajectory.size(); ++k) {
                if (k > 0) {
                    const auto truth = amr::trueIncrement(trajectory[k - 1].state, trajectory[k].state);
                    const double dt = trajectory[k].state.time - trajectory[k - 1].state.time;
                    const auto var = amr::odometryVariances(
                        {trajectory[k].linear_velocity * dt, trajectory[k].angular_velocity * dt}, odometry);
                    const double dn = std::sqrt(var[0]) * unit(noise);
                    const double rn = std::sqrt(var[1]) * unit(noise);
                    filter.predict({truth.distance + dn, truth.rotation + rn}, var);
                }
                filter.update(offsets, scans[k].ranges, scans[k].hits, lidar.max_range);
                const amr::RobotState& s = trajectory[k].state;
                const amr::Pose truth{s.x, s.y, s.theta};
                const amr::Pose twin = image(truth);
                double near_truth = 0.0, near_twin = 0.0;
                for (const amr::Particle& p : filter.particles()) {
                    if (positionError(p.pose, truth) < 0.5 && headingError(p.pose, truth) < 0.35) {
                        near_truth += p.weight;
                    } else if (positionError(p.pose, twin) < 0.5 && headingError(p.pose, twin) < 0.35) {
                        near_twin += p.weight;
                    }
                }
                last_truth = near_truth;
                if (k >= 20) {  // after 2 s
                    both += near_truth + near_twin;
                    both_n += 1.0;
                    truth_dropped = truth_dropped || near_truth < 1e-9;
                }
            }
            const amr::RobotState& last = trajectory.back().state;
            const amr::Pose truth{last.x, last.y, last.theta};
            ++counts[static_cast<int>(classify(truth, filter.estimate().pose, image(truth)))];
            dropped += truth_dropped ? 1 : 0;
            final_truth += last_truth;
            ++runs;
        }
        const double n = std::max(1, runs);
        std::cout << std::left << std::setw(26) << name.substr(5) << std::right << std::fixed << std::setprecision(1)
                  << std::setw(9) << 100.0 * counts[0] / n << "%" << std::setw(15) << 100.0 * counts[1] / n << "%"
                  << std::setw(7) << 100.0 * counts[2] / n << "%" << std::setw(17) << 100.0 * both / std::max(1.0, both_n)
                  << "%" << std::setw(18) << dropped << std::setw(21) << 100.0 * final_truth / n << "%\n";
        csv2 << name.substr(5) << ',' << runs << ',' << counts[0] << ',' << counts[1] << ',' << counts[2] << ','
             << both / std::max(1.0, both_n) << ',' << dropped << ',' << final_truth / n << "\n";
    }
    return 0;
}

// ---------------------------------------------------------------------------------
// Planning for localization

int modePlanning(const Options& o) {
    const std::string name = o.map == "maps/obstacles.csv" ? "maps/hall.csv" : o.map;
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(name, 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << "\n";
        return 1;
    }
    const amr::GridMap& map = loaded.map;
    const double radius = 0.25;
    const amr::GridMap inflated = amr::inflateObstacles(map, radius);
    amr::LocalizabilityConfig lc;
    lc.beams = o.beams;
    lc.max_range = o.range;
    lc.range_sigma = o.noise;
    const std::vector<double> localizability = amr::computeLocalizability(map, lc);
    const std::vector<double> weights = {0.0, 0.25, 0.5, 1.0, 2.0, 4.0};
    struct Row {
        double length = 0, rms_sq = 0, samples = 0, worst = 0, claimed = 0, blind = 0;
        int failures = 0, runs = 0;
        std::vector<double> worst_errors;
    };
    std::vector<Row> rows(weights.size());
    std::mt19937 rng(o.seed);
    std::uniform_int_distribution<int> xs(0, map.width() - 1), ys(0, map.height() - 1);
    int pairs = 0;
    for (int attempt = 0; pairs < o.runs && attempt < 100 * o.runs; ++attempt) {
        const amr::GridCoord start{xs(rng), ys(rng)}, goal{xs(rng), ys(rng)};
        if (!inflated.isFree(start) || !inflated.isFree(goal)) {
            continue;
        }
        const amr::WorldPoint a = map.gridToWorld(start), b = map.gridToWorld(goal);
        if (std::hypot(a.x - b.x, a.y - b.y) < 10.0) {
            continue;
        }
        ++pairs;
        const auto lidar_seed = static_cast<std::uint32_t>(rng());
        const auto odometry_seed = static_cast<std::uint32_t>(rng());
        const double heading = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
        for (std::size_t v = 0; v < weights.size(); ++v) {
            amr::LocalizationPlanConfig pc;
            pc.weight = weights[v];
            const amr::PlanResult plan = amr::planForLocalization(inflated, start, goal, localizability, pc);
            if (!plan.success()) {
                continue;
            }
            amr::MotionConfig motion;
            motion.model = amr::RobotModel::DiffDrive;
            motion.robot_radius = radius;
            motion.initial_heading = heading;
            motion.max_time = 1200.0;
            const auto sim = amr::simulatePath(map, plan.path, motion, amr::Connectivity::Eight);
            if (!sim.success()) {
                continue;
            }
            amr::LidarConfig lidar;
            lidar.beams = o.beams;
            lidar.max_range = o.range;
            lidar.noise_stddev = o.noise;
            lidar.seed = lidar_seed;
            const auto scans = amr::scanTrajectory(map, sim.trajectory, lidar);
            amr::EkfConfig config;
            config.enabled = true;
            config.range_sigma = o.noise;
            config.odometry.seed = odometry_seed;
            const auto samples = amr::runLocalization(map, sim.trajectory, scans, lidar, config);
            Row& row = rows[v];
            ++row.runs;
            row.length += amr::pathLengthMeters(map, plan.path);
            double worst = 0.0, claimed = 0.0;
            for (const auto& s : samples) {
                const double e = positionError(s.truth, s.estimate);
                row.rms_sq += e * e;
                row.samples += 1.0;
                worst = std::max(worst, e);
                claimed = std::max(claimed, std::sqrt(std::max(s.covariance[0], s.covariance[4])));
            }
            for (std::size_t k = 1; k < plan.path.size(); ++k) {
                const double u = localizability[static_cast<std::size_t>(plan.path[k].y * map.width() + plan.path[k].x)];
                if (u > 0.5) {
                    row.blind += amr::pathLengthMeters(map, {plan.path[k - 1], plan.path[k]});
                }
            }
            row.worst += worst;
            row.worst_errors.push_back(worst);
            row.claimed += claimed;
            row.failures += worst > 0.5 ? 1 : 0;
        }
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "localization_planning.csv");
    csv << "weight,runs,mean_length_m,blind_length_m,rms_error_m,mean_worst_error_m,p90_worst_error_m,"
           "mean_max_claimed_std_m,runs_worse_than_0.5m\n";
    std::cout << "Planning for localization (" << name << ", " << o.beams << " beams, range " << o.range
              << " m, " << pairs << " start-goal pairs of 10+ m)\n"
              << std::setw(8) << "weight" << std::setw(11) << "length m" << std::setw(10) << "blind m"
              << std::setw(12) << "RMS cm" << std::setw(14) << "worst cm" << std::setw(12) << "p90 worst"
              << std::setw(14) << "claimed std" << std::setw(12) << "> 0.5 m" << "\n";
    for (std::size_t v = 0; v < weights.size(); ++v) {
        const Row& r = rows[v];
        const double n = std::max(1, r.runs);
        std::cout << std::setw(8) << std::fixed << std::setprecision(2) << weights[v] << std::setw(11)
                  << std::setprecision(1) << r.length / n << std::setw(10) << r.blind / n << std::setw(12)
                  << std::setprecision(2) << 100.0 * std::sqrt(r.rms_sq / std::max(1.0, r.samples)) << std::setw(14)
                  << 100.0 * r.worst / n << std::setw(12) << 100.0 * percentile(r.worst_errors, 0.9) << std::setw(14)
                  << 100.0 * r.claimed / n << std::setw(11) << std::setprecision(1) << 100.0 * r.failures / n << "%\n";
        csv << weights[v] << ',' << r.runs << ',' << r.length / n << ',' << r.blind / n << ','
            << std::sqrt(r.rms_sq / std::max(1.0, r.samples)) << ',' << r.worst / n << ','
            << percentile(r.worst_errors, 0.9) << ',' << r.claimed / n << ',' << r.failures << "\n";
    }
    return 0;
}

// ---------------------------------------------------------------------------------
// Steering by the estimate

/// The same map with every cell split into k x k cells (identical geometry).
amr::GridMap subdivide(const amr::GridMap& map, int k) {
    amr::GridMap fine(map.width() * k, map.height() * k, map.resolution() / k);
    for (int y = 0; y < fine.height(); ++y) {
        for (int x = 0; x < fine.width(); ++x) {
            fine.setOccupied({x, y}, map.isOccupied({x / k, y / k}));
        }
    }
    return fine;
}

int modeSteering(const Options& o) {
    const amr::MapLoadResult loaded = amr::loadGridMapCsv(o.map, 0.5);
    if (!loaded.ok) {
        std::cerr << loaded.error << "\n";
        return 1;
    }
    // 0.1 m cells, so margins of a few centimetres are not lost to the inflation's
    // quantization at cell centers.
    const amr::GridMap map = subdivide(loaded.map, 5);
    const double radius = 0.2;
    const std::vector<double> margins = {0.0, 0.05, 0.1, 0.2, 0.3};
    struct Quality {
        std::string name;
        int beams;
        double range;
        double odometry_scale;  // multiplies the default odometry noise
        double safety;          // ClosedLoopConfig::safety_sigmas
    };
    const std::vector<Quality> qualities = {{"true pose (exact odometry)", 0, 5.0, 0.0, 0.0},
                                            {"EKF, 72 beams, 5 m", 72, 5.0, 1.0, 0.0},
                                            {"EKF, 18 beams, 2 m", 18, 2.0, 1.0, 0.0},
                                            {"EKF, 18 beams, 2 m, odometry x3", 18, 2.0, 3.0, 0.0},
                                            {"  ... + hold at 3 sigma", 18, 2.0, 3.0, 3.0},
                                            {"dead reckoning (no LiDAR)", 0, 5.0, 1.0, 0.0},
                                            {"  ... + hold at 3 sigma", 0, 5.0, 1.0, 3.0}};
    struct Cell {
        int runs = 0, reached = 0, collisions = 0, exceeded = 0, timeouts = 0;
        double clearance = 0.0, goal_error = 0.0, max_error = 0.0, travel = 0.0;
    };
    std::vector<std::vector<Cell>> table(qualities.size(), std::vector<Cell>(margins.size()));
    std::mt19937 rng(o.seed);
    std::uniform_int_distribution<int> xs(0, map.width() - 1), ys(0, map.height() - 1);
    const amr::GridMap widest = amr::inflateObstacles(map, radius + margins.back());
    std::vector<double> localizability;
    if (o.plan_weight >= 0.0) {
        amr::LocalizabilityConfig lc;
        lc.beams = 18;  // the weaker LiDAR of the table, the one where it matters
        lc.max_range = 2.0;
        lc.range_sigma = o.noise;
        localizability = amr::computeLocalizability(map, lc);
    }
    int pairs = 0;
    for (int attempt = 0; pairs < o.runs && attempt < 200 * o.runs; ++attempt) {
        const amr::GridCoord start{xs(rng), ys(rng)}, goal{xs(rng), ys(rng)};
        if (!widest.isFree(start) || !widest.isFree(goal)) {
            continue;
        }
        const amr::WorldPoint a = map.gridToWorld(start), b = map.gridToWorld(goal);
        if (std::hypot(a.x - b.x, a.y - b.y) < 6.0 || !amr::planAStar(widest, start, goal).success()) {
            continue;
        }
        ++pairs;
        const auto seed = static_cast<std::uint32_t>(rng());
        const double heading = std::uniform_real_distribution<double>(-kPi, kPi)(rng);
        for (std::size_t mi = 0; mi < margins.size(); ++mi) {
            const amr::GridMap inflated = amr::inflateObstacles(map, radius + margins[mi]);
            amr::PlanResult plan;
            if (o.plan_weight >= 0.0) {
                amr::LocalizationPlanConfig pc;
                pc.weight = o.plan_weight;
                plan = amr::planForLocalization(inflated, start, goal, localizability, pc);
            } else {
                plan = amr::planWithFootprint(map, inflated, start, goal);
            }
            if (!plan.success()) {
                continue;
            }
            for (std::size_t q = 0; q < qualities.size(); ++q) {
                amr::ClosedLoopConfig config;
                config.motion.model = amr::RobotModel::DiffDrive;
                config.motion.robot_radius = radius;
                config.motion.initial_heading = heading;
                config.motion.max_time = 300.0;
                config.lidar.beams = qualities[q].beams;
                config.lidar.max_range = qualities[q].range;
                config.lidar.noise_stddev = o.noise;
                config.lidar.seed = seed;
                config.ekf.enabled = true;
                config.ekf.range_sigma = o.noise;
                config.ekf.odometry.seed = seed + 1;
                config.safety_sigmas = qualities[q].safety;
                const double k = qualities[q].odometry_scale;
                config.ekf.odometry.distance *= k;
                config.ekf.odometry.turn *= k;
                config.ekf.odometry.drift *= k;
                if (k == 0.0) {
                    config.ekf.odometry.distance = config.ekf.odometry.turn = config.ekf.odometry.drift = 1e-12;
                    config.ekf.initial_position_sigma = config.ekf.initial_heading_sigma = 1e-9;
                }
                const auto r = amr::simulateWithEstimate(map, plan.path, config,
                                                         o.plan_weight >= 0.0 ? amr::Connectivity::Eight
                                                                              : amr::Connectivity::Four);
                Cell& cell = table[q][mi];
                ++cell.runs;
                cell.reached += r.status == amr::NavigationStatus::GoalReached ? 1 : 0;
                cell.collisions += r.status == amr::NavigationStatus::Collision ? 1 : 0;
                cell.timeouts += r.status == amr::NavigationStatus::Timeout ? 1 : 0;
                cell.travel += r.trajectory.back().state.time;
                cell.clearance += r.min_clearance;
                cell.goal_error += r.final_goal_error;
                cell.max_error += r.max_error;
                cell.exceeded += r.max_error > margins[mi] ? 1 : 0;
            }
        }
    }
    std::filesystem::create_directories(o.output);
    std::ofstream csv(o.output / "localization_steering.csv");
    csv << "localization,margin_m,runs,reached,collisions,timeouts,mean_min_clearance_m,mean_true_goal_error_m,"
           "mean_max_error_m,runs_error_above_margin,mean_time_s\n";
    std::cout << "Steering by the estimate (" << o.map << " at 0.1 m cells, robot radius " << radius << " m, " << pairs
              << " routes of 6+ m)\n";
    for (std::size_t q = 0; q < qualities.size(); ++q) {
        std::cout << "\n" << qualities[q].name << "\n"
                  << std::setw(10) << "margin" << std::setw(10) << "reached" << std::setw(12) << "collisions"
                  << std::setw(10) << "stopped" << std::setw(16) << "min clear cm" << std::setw(16)
                  << "goal error cm" << std::setw(15) << "max error cm" << std::setw(11) << "time s" << "\n";
        for (std::size_t mi = 0; mi < margins.size(); ++mi) {
            const Cell& c = table[q][mi];
            const double n = std::max(1, c.runs);
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << margins[mi] << " m" << std::setw(9)
                      << std::setprecision(1) << 100.0 * c.reached / n << "%" << std::setw(11)
                      << 100.0 * c.collisions / n << "%" << std::setw(9) << 100.0 * c.timeouts / n << "%"
                      << std::setw(16) << std::setprecision(1) << 100.0 * c.clearance / n << std::setw(16)
                      << 100.0 * c.goal_error / n << std::setw(15) << 100.0 * c.max_error / n << std::setw(11)
                      << c.travel / n << "\n";
            csv << '"' << qualities[q].name << "\"," << margins[mi] << ',' << c.runs << ',' << c.reached << ','
                << c.collisions << ',' << c.timeouts << ',' << c.clearance / n << ',' << c.goal_error / n << ','
                << c.max_error / n << ',' << c.exceeded << ',' << c.travel / n << "\n";
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
        } else if (arg == "--beams") {
            o.beams = std::atoi(value.c_str());
        } else if (arg == "--noise") {
            o.noise = std::atof(value.c_str());
        } else if (arg == "--particles") {
            o.particles = static_cast<std::size_t>(std::atol(value.c_str()));
        } else if (arg == "--plan-weight") {
            o.plan_weight = std::atof(value.c_str());
        } else if (arg == "--range") {
            o.range = std::atof(value.c_str());
        } else if (arg == "--sigma-hit") {
            o.sigma_hit = std::atof(value.c_str());
        } else if (arg == "--correlation") {
            o.correlation = std::atof(value.c_str());
        } else if (arg == "--map") {
            o.map = value;
        } else if (arg == "--output") {
            o.output = value;
        } else {
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::cerr << "usage: localization_experiment global|kidnap [--runs N] [--seed N] [--beams N] "
                     "[--noise M] [--particles N] [--map FILE] [--output DIR]\n";
        return 2;
    }
    if (o.mode == "global") {
        return modeGlobal(o);
    }
    if (o.mode == "kidnap") {
        return modeKidnap(o);
    }
    if (o.mode == "steering") {
        return modeSteering(o);
    }
    if (o.mode == "planning") {
        return modePlanning(o);
    }
    if (o.mode == "symmetric") {
        return modeSymmetric(o);
    }
    std::cerr << "unknown mode '" << o.mode << "'\n";
    return 2;
}
