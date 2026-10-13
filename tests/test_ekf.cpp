// Tests for EKF localization: odometry model, Jacobians, the Kalman update, outlier
// gating, convergence and statistical consistency (NEES).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "astar.hpp"
#include "diff_drive.hpp"
#include "ekf.hpp"
#include "grid_map.hpp"
#include "lidar.hpp"
#include "robot.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"

using amr::EkfConfig;
using amr::GridMap;
using amr::LidarConfig;
using amr::Matrix3;
using amr::OdometryIncrement;
using amr::Pose;
using amr::PoseEkf;

namespace {

constexpr double kPi = 3.14159265358979323846;

GridMap makeMap(const std::vector<std::string>& rows, double resolution) {
    std::vector<std::uint8_t> cells;
    for (const std::string& row : rows) {
        for (const char c : row) {
            cells.push_back(c == '#' ? GridMap::kOccupied : GridMap::kFree);
        }
    }
    return GridMap(static_cast<int>(rows.front().size()), static_cast<int>(rows.size()),
                   resolution, std::move(cells));
}

/// 10 m x 10 m room with a few obstacles (0.5 m cells).
GridMap room() {
    return makeMap({"....................", "....................", "...###..............",
                    "...###.........##...", "...............##...", "....................",
                    "........####........", "........####........", "....................",
                    "....................", "..##................", "..##.........#......",
                    ".............#......", ".............#......", "....................",
                    "......#####.........", "....................", "................##..",
                    "................##..", "...................."},
                   0.5);
}

/// Differential-drive trajectory across the room (8-connected path, several turns).
std::vector<amr::TrajectorySample> roomTrajectory(const GridMap& map) {
    const auto plan = amr::planAStar(map, {1, 1}, {18, 18}, amr::Connectivity::Eight);
    amr::MotionConfig config;
    config.model = amr::RobotModel::DiffDrive;
    config.initial_heading = kPi / 2;
    const auto sim = amr::simulatePath(map, plan.path, config, amr::Connectivity::Eight);
    return sim.trajectory;
}

Matrix3 diagonal(double a, double b, double c) {
    return {a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c};
}

/// Full-circle LiDAR scan from the true pose, exact or with Gaussian noise.
void scanFrom(const GridMap& map, const Pose& truth, const std::vector<double>& offsets,
              double max_range, std::vector<double>& ranges, std::vector<bool>& hits) {
    ranges.clear();
    hits.clear();
    for (const double offset : offsets) {
        const amr::RayHit ray = amr::castRay(map, {truth.x, truth.y}, truth.theta + offset,
                                             max_range);
        ranges.push_back(ray.range);
        hits.push_back(ray.hit);
    }
}

void testIncrementsReproduceTheTrajectory() {
    const GridMap map = room();
    const auto trajectory = roomTrajectory(map);
    REQUIRE(trajectory.size() > 100);
    // Turn-then-drive: every step is a turn or a straight drive, and "turn by dphi,
    // then drive ds" reproduces the next pose from the true increments.
    Pose pose{trajectory[0].state.x, trajectory[0].state.y, trajectory[0].state.theta};
    std::size_t turns = 0;
    std::size_t drives = 0;
    for (std::size_t k = 1; k < trajectory.size(); ++k) {
        const OdometryIncrement inc =
            amr::trueIncrement(trajectory[k - 1].state, trajectory[k].state);
        CHECK(inc.distance == 0.0 || std::fabs(inc.rotation) <= 1e-6);
        turns += inc.distance == 0.0 ? 1 : 0;
        drives += inc.distance > 0.0 ? 1 : 0;
        // The commanded increments equal the true ones (up to the 1e-6 rad residual
        // turn absorbed when driving starts).
        const double dt = trajectory[k].state.time - trajectory[k - 1].state.time;
        CHECK_NEAR(trajectory[k].linear_velocity * dt, inc.distance, 1e-9);
        CHECK_NEAR(trajectory[k].angular_velocity * dt, inc.rotation, 1e-6);
        pose = amr::applyIncrement(pose, inc);
        CHECK_NEAR(pose.x, trajectory[k].state.x, 1e-9);
        CHECK_NEAR(pose.y, trajectory[k].state.y, 1e-9);
        CHECK_NEAR(amr::wrapAngle(pose.theta - trajectory[k].state.theta), 0.0, 1e-9);
    }
    CHECK(turns > 10);
    CHECK(drives > 100);
}

void testOdometryVariances() {
    amr::OdometryNoise noise;
    noise.distance = 0.1;
    noise.turn = 0.2;
    noise.drift = 0.05;
    const auto v = amr::odometryVariances({0.5, -0.25}, noise);
    CHECK_NEAR(v[0], 0.01 * 0.5, 1e-15);
    CHECK_NEAR(v[1], 0.04 * 0.25 + 0.0025 * 0.5, 1e-15);
    const auto none = amr::odometryVariances({0.0, 0.0}, noise);
    CHECK_EQ(none[0], 0.0);
    CHECK_EQ(none[1], 0.0);
}

/// Prediction: P' = F P F^T + V M V^T with the Jacobians of applyIncrement, checked
/// against central finite differences.
void testPredictionJacobians() {
    const Pose pose{1.0, 2.0, 0.7};
    const OdometryIncrement inc{0.3, 0.2};
    const double h = 1e-6;
    // F by finite differences of the pose; V of the increment.
    double f[3][3];
    double v[3][2];
    for (int j = 0; j < 3; ++j) {
        Pose plus = pose;
        Pose minus = pose;
        (j == 0 ? plus.x : j == 1 ? plus.y : plus.theta) += h;
        (j == 0 ? minus.x : j == 1 ? minus.y : minus.theta) -= h;
        const Pose a = amr::applyIncrement(plus, inc);
        const Pose b = amr::applyIncrement(minus, inc);
        f[0][j] = (a.x - b.x) / (2 * h);
        f[1][j] = (a.y - b.y) / (2 * h);
        f[2][j] = amr::wrapAngle(a.theta - b.theta) / (2 * h);
    }
    for (int j = 0; j < 2; ++j) {
        const OdometryIncrement plus{inc.distance + (j == 0 ? h : 0), inc.rotation + (j == 1 ? h : 0)};
        const OdometryIncrement minus{inc.distance - (j == 0 ? h : 0),
                                      inc.rotation - (j == 1 ? h : 0)};
        const Pose a = amr::applyIncrement(pose, plus);
        const Pose b = amr::applyIncrement(pose, minus);
        v[0][j] = (a.x - b.x) / (2 * h);
        v[1][j] = (a.y - b.y) / (2 * h);
        v[2][j] = amr::wrapAngle(a.theta - b.theta) / (2 * h);
    }
    const Matrix3 p{0.04, 0.01, 0.002, 0.01, 0.09, -0.003, 0.002, -0.003, 0.01};
    const std::array<double, 2> m{0.0004, 0.0009};
    PoseEkf ekf(pose, p);
    ekf.predict(inc, m);
    const Pose expected_pose = amr::applyIncrement(pose, inc);
    CHECK_NEAR(ekf.pose().x, expected_pose.x, 1e-15);
    CHECK_NEAR(ekf.pose().theta, expected_pose.theta, 1e-15);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = v[i][0] * m[0] * v[j][0] + v[i][1] * m[1] * v[j][1];
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b) {
                    expected += f[i][a] * p[3 * a + b] * f[j][b];
                }
            }
            CHECK_NEAR(ekf.covariance()[3 * i + j], expected, 1e-9);
        }
    }
}

/// The range Jacobian against central finite differences of castRay, at random
/// poses and beams whose hit stays on the same face.
void testRangeJacobian() {
    const GridMap map = room();
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> pos(0.3, 9.7);
    std::uniform_real_distribution<double> ang(-kPi, kPi);
    const double h = 1e-7;
    int checked = 0;
    for (int trial = 0; trial < 3000 && checked < 500; ++trial) {
        const Pose pose{pos(rng), pos(rng), ang(rng)};
        const double offset = ang(rng);
        const amr::RayHit hit = amr::castRay(map, {pose.x, pose.y}, pose.theta + offset, 20.0);
        if (!hit.hit || hit.range < 0.05 ||
            (hit.face != amr::RayFace::Vertical && hit.face != amr::RayFace::Horizontal)) {
            continue;
        }
        const double incidence = hit.face == amr::RayFace::Vertical
                                     ? std::cos(pose.theta + offset)
                                     : std::sin(pose.theta + offset);
        if (std::fabs(incidence) < 0.2) {
            continue;
        }
        const auto jacobian = amr::rangeJacobian(hit.face, pose.theta + offset, hit.range);
        bool same_face = true;
        double numeric[3];
        for (int j = 0; j < 3; ++j) {
            Pose plus = pose;
            Pose minus = pose;
            (j == 0 ? plus.x : j == 1 ? plus.y : plus.theta) += h;
            (j == 0 ? minus.x : j == 1 ? minus.y : minus.theta) -= h;
            const auto a = amr::castRay(map, {plus.x, plus.y}, plus.theta + offset, 20.0);
            const auto b = amr::castRay(map, {minus.x, minus.y}, minus.theta + offset, 20.0);
            // Near a face edge the perturbed rays may hit another face: skip.
            same_face = same_face && a.face == hit.face && b.face == hit.face &&
                        std::fabs(a.range - b.range) < 1e-3;
            numeric[j] = (a.range - b.range) / (2 * h);
        }
        if (!same_face) {
            continue;
        }
        ++checked;
        for (int j = 0; j < 3; ++j) {
            CHECK_NEAR(jacobian[j], numeric[j], 1e-5 * std::max(1.0, std::fabs(numeric[j])));
        }
    }
    CHECK(checked >= 500);
}

/// One beam: the information-form update equals the textbook Kalman update
/// K = P h^T / (h P h^T + R), x+ = x + K nu, P+ = (I - K h) P.
void testSingleBeamKalmanUpdate() {
    const GridMap map = room();
    const Pose estimate{5.1, 4.2, 0.3};
    const Matrix3 p{0.01, 0.002, 0.001, 0.002, 0.02, -0.002, 0.001, -0.002, 0.005};
    const std::vector<double> offsets{0.2};
    const auto predicted = amr::castRay(map, {estimate.x, estimate.y}, estimate.theta + 0.2, 20.0);
    REQUIRE(predicted.hit);
    REQUIRE(predicted.face == amr::RayFace::Vertical || predicted.face == amr::RayFace::Horizontal);
    const double measured = predicted.range + 0.04;
    const double sigma = 0.05;
    PoseEkf ekf(estimate, p);
    const auto stats = ekf.update(map, offsets, {measured}, {true}, 20.0, sigma, 3.0);
    CHECK_EQ(stats.used, 1u);
    const auto h = amr::rangeJacobian(predicted.face, estimate.theta + 0.2, predicted.range);
    double ph[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            ph[i] += p[3 * i + j] * h[j];
        }
    }
    const double s = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] + sigma * sigma;
    const double nu = measured - predicted.range;
    CHECK_NEAR(ekf.pose().x, estimate.x + ph[0] / s * nu, 1e-12);
    CHECK_NEAR(ekf.pose().y, estimate.y + ph[1] / s * nu, 1e-12);
    CHECK_NEAR(ekf.pose().theta, estimate.theta + ph[2] / s * nu, 1e-12);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK_NEAR(ekf.covariance()[3 * i + j], p[3 * i + j] - ph[i] * ph[j] / s, 1e-12);
        }
    }
}

/// Outliers beyond the gate are rejected and do not move the estimate.
void testGate() {
    const GridMap map = room();
    const Pose estimate{5.1, 4.2, 0.3};
    const Matrix3 p = diagonal(1e-4, 1e-4, 1e-4);
    const std::vector<double> offsets{0.2};
    const auto predicted = amr::castRay(map, {estimate.x, estimate.y}, estimate.theta + 0.2, 20.0);
    PoseEkf gated(estimate, p);
    const auto rejected = gated.update(map, offsets, {predicted.range + 1.0}, {true}, 20.0, 0.05,
                                       3.0);
    CHECK_EQ(rejected.used, 0u);
    CHECK_EQ(rejected.rejected, 1u);
    CHECK_EQ(gated.pose().x, estimate.x);
    CHECK_EQ(gated.covariance()[0], p[0]);
    // Without a meaningful gate the same beam is fused.
    PoseEkf open(estimate, p);
    CHECK_EQ(open.update(map, offsets, {predicted.range + 1.0}, {true}, 20.0, 0.05, 1e6).used, 1u);
    CHECK(open.pose().x != estimate.x);
    // Beams without a measured return are ignored.
    PoseEkf none(estimate, p);
    const auto ignored = none.update(map, offsets, {20.0}, {false}, 20.0, 0.05, 3.0);
    CHECK_EQ(ignored.used + ignored.rejected, 0u);
}

/// From a wrong initial pose (0.3 m and 6 deg off, within its stated uncertainty),
/// repeated exact scans pull the estimate onto the true pose.
void testConvergence() {
    const GridMap map = room();
    LidarConfig lidar;
    lidar.beams = 72;
    lidar.max_range = 20.0;
    const auto offsets = amr::beamOffsets(lidar);
    const Pose truth{5.25, 4.6, 0.4};
    PoseEkf ekf({5.45, 4.38, 0.5}, diagonal(0.04, 0.04, 0.01));
    std::vector<double> ranges;
    std::vector<bool> hits;
    scanFrom(map, truth, offsets, lidar.max_range, ranges, hits);
    double first_error = 0.0;
    for (int i = 0; i < 10; ++i) {
        ekf.update(map, offsets, ranges, hits, lidar.max_range, 0.05, 3.0);
        if (i == 0) {
            first_error = std::hypot(ekf.pose().x - truth.x, ekf.pose().y - truth.y);
        }
    }
    CHECK(first_error < 0.1);
    CHECK_NEAR(ekf.pose().x, truth.x, 1e-3);
    CHECK_NEAR(ekf.pose().y, truth.y, 1e-3);
    CHECK_NEAR(ekf.pose().theta, truth.theta, 1e-3);
    CHECK(ekf.covariance()[0] < 1e-4);
}

EkfConfig enabledConfig(std::uint32_t seed) {
    EkfConfig config;
    config.enabled = true;
    config.odometry.seed = seed;
    return config;
}

void testRunsAlongTheTrajectory() {
    const GridMap map = room();
    const auto trajectory = roomTrajectory(map);
    LidarConfig lidar;
    lidar.beams = 72;
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    const auto with_lidar = amr::runLocalization(map, trajectory, scans, lidar, enabledConfig(3));
    const auto prediction = amr::runLocalization(map, trajectory, {}, lidar, enabledConfig(3));
    REQUIRE(with_lidar.size() == trajectory.size());
    REQUIRE(prediction.size() == trajectory.size());
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        CHECK_EQ(with_lidar[k].time, trajectory[k].state.time);
        CHECK_EQ(with_lidar[k].truth.x, trajectory[k].state.x);
        // Same seed: the same odometry with and without the LiDAR.
        CHECK_EQ(with_lidar[k].odometry.x, prediction[k].odometry.x);
        // Without measurements the estimate is dead reckoning.
        CHECK_EQ(prediction[k].estimate.x, prediction[k].odometry.x);
        CHECK_EQ(prediction[k].estimate.theta, prediction[k].odometry.theta);
        if (k > 0) {
            // Prediction only: the uncertainty never shrinks.
            const Matrix3& a = prediction[k - 1].covariance;
            const Matrix3& b = prediction[k].covariance;
            CHECK(b[0] + b[4] + b[8] >= a[0] + a[4] + a[8]);
        }
    }
    const auto summary = amr::summarizeLocalization(with_lidar);
    CHECK(summary.odometry.position_max > 0.1);  // dead reckoning drifts
    CHECK(summary.estimate.position_max < 0.03);  // the EKF does not
    CHECK(summary.estimate.position_rms < summary.odometry.position_rms / 10);
    CHECK(summary.beams_used > 100 * trajectory.size() / 4);
    // Determinism and seeds.
    const auto again = amr::runLocalization(map, trajectory, scans, lidar, enabledConfig(3));
    const auto other = amr::runLocalization(map, trajectory, scans, lidar, enabledConfig(4));
    CHECK_EQ(again.back().estimate.x, with_lidar.back().estimate.x);
    CHECK(other.back().odometry.x != with_lidar.back().odometry.x);
    // Disabled: nothing.
    CHECK(amr::runLocalization(map, trajectory, scans, lidar, EkfConfig{}).empty());
}

/// Noise-free odometry: dead reckoning follows the true trajectory.
void testNoiseFreeOdometry() {
    const GridMap map = room();
    const auto trajectory = roomTrajectory(map);
    EkfConfig config = enabledConfig(1);
    config.odometry.distance = 0.0;
    config.odometry.turn = 0.0;
    config.odometry.drift = 0.0;
    config.initial_position_sigma = 1e-12;
    config.initial_heading_sigma = 1e-12;
    const auto samples = amr::runLocalization(map, trajectory, {}, LidarConfig{}, config);
    REQUIRE(samples.size() == trajectory.size());
    for (const auto& s : samples) {
        CHECK_NEAR(s.odometry.x, s.truth.x, 1e-9);
        CHECK_NEAR(s.odometry.y, s.truth.y, 1e-9);
        CHECK_NEAR(amr::wrapAngle(s.odometry.theta - s.truth.theta), 0.0, 1e-9);
    }
}

/// Consistency: averaged over many runs, the NEES e^T P^-1 e is close to 3 (the
/// state dimension) if the filter's covariance matches its actual errors.
void testConsistency() {
    const GridMap map = room();
    const auto trajectory = roomTrajectory(map);
    const auto mean_nees = [&](bool lidar_on, int runs) {
        LidarConfig lidar;
        lidar.beams = lidar_on ? 36 : 0;
        lidar.noise_stddev = 0.05;
        double sum = 0.0;
        std::size_t count = 0;
        for (int run = 0; run < runs; ++run) {
            lidar.seed = static_cast<std::uint32_t>(run);
            const auto scans = amr::scanTrajectory(map, trajectory, lidar);
            EkfConfig config = enabledConfig(static_cast<std::uint32_t>(1000 + run));
            config.range_sigma = 0.05;
            for (const auto& s : amr::runLocalization(map, trajectory, scans, lidar, config)) {
                sum += s.nees;
                ++count;
            }
        }
        return sum / static_cast<double>(count);
    };
    const double prediction = mean_nees(false, 200);
    const double corrected = mean_nees(true, 40);
    std::cout << "    mean NEES: prediction only " << prediction << ", with LiDAR " << corrected
              << '\n';
    CHECK(prediction > 2.5 && prediction < 3.5);
    CHECK(corrected > 2.4 && corrected < 3.8);
}

/// Driving west (heading +-180 deg), the estimate's heading flips between +pi and
/// -pi; errors and the NEES must use the wrapped difference.
void testHeadingWrap() {
    const GridMap map = room();
    const auto plan = amr::planAStar(map, {18, 9}, {1, 9});
    REQUIRE(plan.success());
    amr::MotionConfig motion;
    motion.model = amr::RobotModel::DiffDrive;
    motion.initial_heading = kPi;
    const auto trajectory = amr::simulatePath(map, plan.path, motion).trajectory;
    LidarConfig lidar;
    lidar.beams = 36;
    const auto scans = amr::scanTrajectory(map, trajectory, lidar);
    bool flipped = false;
    for (std::uint32_t seed = 0; seed < 5; ++seed) {
        const auto samples = amr::runLocalization(map, trajectory, scans, lidar,
                                                  enabledConfig(seed));
        double max_nees = 0.0;
        for (const auto& s : samples) {
            max_nees = std::max(max_nees, s.nees);
            flipped = flipped || (s.estimate.theta < 0.0) != (s.truth.theta < 0.0);
        }
        CHECK(max_nees < 100.0);
        CHECK(amr::summarizeLocalization(samples).estimate.heading_rms < 0.01);
    }
    CHECK(flipped);  // the case this test is about did occur
    // Directly: 179.4 deg vs. -179.4 deg is a 1.2 deg error.
    amr::LocalizationSample sample;
    sample.truth.theta = kPi - 0.01;
    sample.estimate.theta = -kPi + 0.01;
    sample.odometry = sample.truth;
    CHECK_NEAR(amr::summarizeLocalization({sample}).estimate.heading_rms, 0.02, 1e-12);
}

/// The initial estimate is the true start pose plus an error drawn from the stated
/// initial uncertainty (so the NEES is meaningful from the first sample).
void testInitialError() {
    const GridMap map = room();
    const auto trajectory = roomTrajectory(map);
    const std::vector<amr::TrajectorySample> start(trajectory.begin(), trajectory.begin() + 1);
    double sum_x = 0.0, sum_xx = 0.0, sum_tt = 0.0;
    const int runs = 2000;
    for (int run = 0; run < runs; ++run) {
        EkfConfig config = enabledConfig(static_cast<std::uint32_t>(run));
        config.initial_position_sigma = 0.02;
        config.initial_heading_sigma = 0.05;
        const auto s = amr::runLocalization(map, start, {}, LidarConfig{}, config).front();
        const double ex = s.estimate.x - s.truth.x;
        const double et = amr::wrapAngle(s.estimate.theta - s.truth.theta);
        CHECK_EQ(s.odometry.x, s.estimate.x);  // dead reckoning starts at the same pose
        CHECK_NEAR(s.covariance[0], 0.02 * 0.02, 1e-15);
        sum_x += ex;
        sum_xx += ex * ex;
        sum_tt += et * et;
    }
    CHECK(std::fabs(sum_x / runs) < 0.002);
    CHECK_NEAR(std::sqrt(sum_xx / runs), 0.02, 0.002);
    CHECK_NEAR(std::sqrt(sum_tt / runs), 0.05, 0.005);
}

void testValidation() {
    EkfConfig config;
    CHECK(amr::validateEkfConfig(config).empty());  // disabled: not checked
    config.enabled = true;
    CHECK(amr::validateEkfConfig(config).empty());
    EkfConfig bad = config;
    bad.odometry.drift = -0.1;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.odometry.distance = 11.0;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.range_sigma = 0.0;
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.gate = std::nan("");
    CHECK(!amr::validateEkfConfig(bad).empty());
    bad = config;
    bad.initial_heading_sigma = 0.0;
    CHECK(!amr::validateEkfConfig(bad).empty());
}

}  // namespace

int main() {
    return testfw::runAll({
        {"Odometry increments reproduce the trajectory exactly", testIncrementsReproduceTheTrajectory},
        {"Odometry variances", testOdometryVariances},
        {"Prediction: Jacobians vs. finite differences", testPredictionJacobians},
        {"Range Jacobian vs. finite differences of the ray cast", testRangeJacobian},
        {"Single-beam update equals the Kalman formula", testSingleBeamKalmanUpdate},
        {"Innovation gate rejects outliers", testGate},
        {"Converges from a wrong initial pose", testConvergence},
        {"Localization along a trajectory", testRunsAlongTheTrajectory},
        {"Noise-free odometry follows the truth", testNoiseFreeOdometry},
        {"Consistency: mean NEES close to 3", testConsistency},
        {"Heading errors are wrapped at +-180 deg", testHeadingWrap},
        {"Initial error drawn from the initial uncertainty", testInitialError},
        {"Configuration validation", testValidation},
    });
}
