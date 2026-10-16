#include "ekf.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <memory>
#include <optional>

namespace amr {

namespace {

/// Grazing beams (|cos| or |sin| of the angle to the face normal below this,
/// i.e. incidence above ~78 deg) are not used: their range changes too fast with
/// the pose for the linearization to hold.
constexpr double kMinIncidenceCosine = 0.2;
constexpr double kPi = 3.14159265358979323846;
/// Model policy: beams closer than this to grazing are skipped (numerical safety only).
constexpr double kMinModelIncidenceCosine = 0.02;
/// Model policy: map error left after a wall's offset and tilt, as a fraction of the
/// offset std (0.0082 m of 0.029 m on maps rasterized at 0.1 m).
constexpr double kResidualMapFraction = 0.3;

Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
    Matrix3 c{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                c[3 * i + j] += a[3 * i + k] * b[3 * k + j];
            }
        }
    }
    return c;
}

Matrix3 transpose(const Matrix3& a) {
    return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

/// Average with the transpose: removes the rounding asymmetry of a covariance.
Matrix3 symmetrize(const Matrix3& a) {
    Matrix3 s = a;
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            const double mean = 0.5 * (a[3 * i + j] + a[3 * j + i]);
            s[3 * i + j] = mean;
            s[3 * j + i] = mean;
        }
    }
    return s;
}

/// Inverse by cofactors. The callers only invert covariance / information
/// matrices, which are positive definite.
Matrix3 inverse(const Matrix3& m) {
    const Matrix3 cofactor{
        m[4] * m[8] - m[5] * m[7], m[5] * m[6] - m[3] * m[8], m[3] * m[7] - m[4] * m[6],
        m[2] * m[7] - m[1] * m[8], m[0] * m[8] - m[2] * m[6], m[1] * m[6] - m[0] * m[7],
        m[1] * m[5] - m[2] * m[4], m[2] * m[3] - m[0] * m[5], m[0] * m[4] - m[1] * m[3]};
    const double det = m[0] * cofactor[0] + m[1] * cofactor[1] + m[2] * cofactor[2];
    Matrix3 inv = transpose(cofactor);
    for (double& v : inv) {
        v /= det;
    }
    return inv;
}

double quadratic(const std::array<double, 3>& v, const Matrix3& m) {
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            sum += v[i] * m[3 * i + j] * v[j];
        }
    }
    return sum;
}

std::array<double, 3> poseError(const Pose& truth, const Pose& estimate) {
    return {truth.x - estimate.x, truth.y - estimate.y, wrapAngle(truth.theta - estimate.theta)};
}

Pose poseOf(const RobotState& state) {
    return {state.x, state.y, state.theta};
}

bool validNoise(double value) {
    return std::isfinite(value) && value >= 0.0 && value <= kMaxOdometryNoise;
}

}  // namespace

std::string validateEkfConfig(const EkfConfig& config) {
    if (!config.enabled) {
        return {};
    }
    const OdometryNoise& noise = config.odometry;
    if (!validNoise(noise.distance) || !validNoise(noise.turn) || !validNoise(noise.drift)) {
        return "odometry noise must be between 0 and 10";
    }
    if (!std::isfinite(config.range_sigma) || config.range_sigma <= 0.0 ||
        config.range_sigma > kMaxRangeSigma) {
        return "EKF range sigma must be in (0, 100] m";
    }
    if (!std::isfinite(config.gate) || config.gate <= 0.0) {
        return "EKF gate must be a positive number of standard deviations";
    }
    const MeasurementModelConfig& m = config.model;
    if (!std::isfinite(m.max_incidence) || m.max_incidence <= 0.0 || m.max_incidence > kPi / 2) {
        return "EKF maximum incidence must be in (0, 90] degrees";
    }
    if (!std::isfinite(m.bias_sigma) || m.bias_sigma < 0.0 || m.bias_sigma > kMaxRangeSigma ||
        !std::isfinite(m.bias_time) || m.bias_time < 0.0) {
        return "EKF bias model must have sigma in [0, 100] m and a non-negative time";
    }
    if (!std::isfinite(m.map_sigma) || m.map_sigma < 0.0 || m.map_sigma > kMaxRangeSigma ||
        !std::isfinite(m.map_tilt) || m.map_tilt < 0.0 || !std::isfinite(m.map_residual) ||
        m.map_residual < 0.0 || m.map_residual > kMaxRangeSigma || m.max_map_states < 2) {
        return "EKF map-error model must have sigma in [0, 100] m and a positive length";
    }
    if (!std::isfinite(config.initial_position_sigma) || config.initial_position_sigma <= 0.0 ||
        !std::isfinite(config.initial_heading_sigma) || config.initial_heading_sigma <= 0.0) {
        return "initial EKF uncertainty must be positive";
    }
    return {};
}

OdometryIncrement trueIncrement(const RobotState& from, const RobotState& to) {
    // Negative distance when the robot backed up (a retreat from a moving obstacle).
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const bool backwards = (to.x - from.x) * std::cos(to.theta) +
                               (to.y - from.y) * std::sin(to.theta) < 0.0;
    return {backwards ? -distance : distance, wrapAngle(to.theta - from.theta)};
}

Pose applyIncrement(const Pose& pose, const OdometryIncrement& increment) {
    const double theta = pose.theta + increment.rotation;
    return {pose.x + increment.distance * std::cos(theta),
            pose.y + increment.distance * std::sin(theta), wrapAngle(theta)};
}

std::array<double, 2> odometryVariances(const OdometryIncrement& increment,
                                        const OdometryNoise& noise) {
    const double distance = std::fabs(increment.distance);
    const double rotation = std::fabs(increment.rotation);
    return {noise.distance * noise.distance * distance,
            noise.turn * noise.turn * rotation + noise.drift * noise.drift * distance};
}

std::array<double, 3> rangeJacobian(RayFace face, double angle, double range) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    if (face == RayFace::Vertical) {
        return {-1.0 / c, 0.0, range * s / c};
    }
    if (face == RayFace::Horizontal) {
        return {0.0, -1.0 / s, -range * c / s};
    }
    return {0.0, 0.0, 0.0};
}

PoseEkf::PoseEkf(const Pose& initial, const Matrix3& covariance)
    : pose_(initial), covariance_(covariance) {}

void PoseEkf::predict(const OdometryIncrement& measured, const std::array<double, 2>& m) {
    const double ds = measured.distance;
    const double theta = pose_.theta + measured.rotation;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    // F = d(pose') / d(pose); V = d(pose') / d(ds, dphi).
    const Matrix3 f{1.0, 0.0, -ds * s, 0.0, 1.0, ds * c, 0.0, 0.0, 1.0};
    const double v[3][2] = {{c, -ds * s}, {s, ds * c}, {0.0, 1.0}};
    Matrix3 predicted = multiply(multiply(f, covariance_), transpose(f));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            predicted[3 * i + j] += v[i][0] * m[0] * v[j][0] + v[i][1] * m[1] * v[j][1];
        }
    }
    covariance_ = symmetrize(predicted);
    pose_ = applyIncrement(pose_, measured);
}

UpdateStats PoseEkf::update(const GridMap& map, const std::vector<double>& offsets,
                            const std::vector<double>& ranges, const std::vector<bool>& hits,
                            double max_range, double range_sigma, double gate) {
    UpdateStats stats;
    const double r = range_sigma * range_sigma;
    Matrix3 information{};               // sum H^T H / R
    std::array<double, 3> correction{};  // sum H^T nu / R
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        if (!hits[i]) {
            continue;
        }
        const double angle = pose_.theta + offsets[i];
        const RayHit predicted = castRay(map, {pose_.x, pose_.y}, angle, max_range);
        if (!predicted.hit || predicted.range <= 0.0) {
            continue;
        }
        const double incidence = predicted.face == RayFace::Vertical     ? std::cos(angle)
                                 : predicted.face == RayFace::Horizontal ? std::sin(angle)
                                                                         : 0.0;
        if (std::fabs(incidence) < kMinIncidenceCosine) {
            continue;  // corner or grazing beam
        }
        const std::array<double, 3> h = rangeJacobian(predicted.face, angle, predicted.range);
        const double innovation = ranges[i] - predicted.range;
        const double variance = quadratic(h, covariance_) + r;
        if (innovation * innovation > gate * gate * variance) {
            ++stats.rejected;
            continue;
        }
        ++stats.used;
        for (int a = 0; a < 3; ++a) {
            correction[a] += h[a] * innovation / r;
            for (int b = 0; b < 3; ++b) {
                information[3 * a + b] += h[a] * h[b] / r;
            }
        }
    }
    if (stats.used == 0) {
        return stats;
    }
    Matrix3 posterior_information = inverse(covariance_);
    for (int k = 0; k < 9; ++k) {
        posterior_information[k] += information[k];
    }
    covariance_ = symmetrize(inverse(posterior_information));
    double delta[3] = {0.0, 0.0, 0.0};
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            delta[a] += covariance_[3 * a + b] * correction[b];
        }
    }
    pose_ = {pose_.x + delta[0], pose_.y + delta[1], wrapAngle(pose_.theta + delta[2])};
    return stats;
}

bool MeasurementModelConfig::classic() const {
    const MeasurementModelConfig defaults;
    return surface == SurfaceKind::Grid && grazing == GrazingPolicy::Cutoff &&
           max_incidence == defaults.max_incidence && bias_sigma == 0.0 && map_sigma == 0.0 &&
           map_residual == 0.0;
}

AugmentedEkf::AugmentedEkf(const Pose& initial, const Matrix3& covariance,
                           const SurfaceModel& surface, const MeasurementModelConfig& model,
                           double map_resolution)
    : surface_(surface), model_(model),
      resolution_(map_resolution),
      n_(model.bias_sigma > 0.0 ? 4 : 3), x_(n_, 0.0), p_(n_ * n_, 0.0),
      has_bias_(model.bias_sigma > 0.0), map_begin_(n_) {
    x_[0] = initial.x;
    x_[1] = initial.y;
    x_[2] = initial.theta;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            at(i, j) = covariance[3 * i + j];
        }
    }
    if (has_bias_) {
        at(3, 3) = model.bias_sigma * model.bias_sigma;
    }
}

Pose AugmentedEkf::pose() const {
    return {x_[0], x_[1], x_[2]};
}

Matrix3 AugmentedEkf::covariance() const {
    Matrix3 c{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            c[3 * i + j] = at(i, j);
        }
    }
    return symmetrize(c);
}

double AugmentedEkf::bias() const {
    return has_bias_ ? x_[3] : 0.0;
}

double AugmentedEkf::biasVariance() const {
    return has_bias_ ? at(3, 3) : 0.0;
}

void AugmentedEkf::predict(const OdometryIncrement& measured, const std::array<double, 2>& m,
                           double dt) {
    const double ds = measured.distance;
    const double theta = x_[2] + measured.rotation;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double f[3][3] = {{1.0, 0.0, -ds * s}, {0.0, 1.0, ds * c}, {0.0, 0.0, 1.0}};
    const double v[3][2] = {{c, -ds * s}, {s, ds * c}, {0.0, 1.0}};
    double a = 1.0;  // bias decay over dt
    if (has_bias_ && model_.bias_time > 0.0) {
        a = std::exp(-std::max(dt, 0.0) / model_.bias_time);
    }
    // P' = F P F^T + Q with F = identity except the pose block (and the bias decay).
    // Rows first (F P), then columns ((F P) F^T).
    for (std::size_t j = 0; j < n_; ++j) {
        const double p0 = at(0, j), p1 = at(1, j), p2 = at(2, j);
        at(0, j) = f[0][0] * p0 + f[0][1] * p1 + f[0][2] * p2;
        at(1, j) = f[1][0] * p0 + f[1][1] * p1 + f[1][2] * p2;
        at(2, j) = p2;
        if (has_bias_) {
            at(3, j) *= a;
        }
    }
    for (std::size_t i = 0; i < n_; ++i) {
        const double p0 = at(i, 0), p1 = at(i, 1), p2 = at(i, 2);
        at(i, 0) = f[0][0] * p0 + f[0][1] * p1 + f[0][2] * p2;
        at(i, 1) = f[1][0] * p0 + f[1][1] * p1 + f[1][2] * p2;
        at(i, 2) = p2;
        if (has_bias_) {
            at(i, 3) *= a;
        }
    }
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            at(i, j) += v[i][0] * m[0] * v[j][0] + v[i][1] * m[1] * v[j][1];
        }
    }
    if (has_bias_) {
        at(3, 3) += (1.0 - a * a) * model_.bias_sigma * model_.bias_sigma;
        x_[3] *= a;
    }
    for (std::size_t i = 0; i < n_; ++i) {  // remove rounding asymmetry
        for (std::size_t j = i + 1; j < n_; ++j) {
            const double mean = 0.5 * (at(i, j) + at(j, i));
            at(i, j) = mean;
            at(j, i) = mean;
        }
    }
    const Pose next = applyIncrement({x_[0], x_[1], x_[2]}, measured);
    x_[0] = next.x;
    x_[1] = next.y;
    x_[2] = next.theta;
}

std::size_t AugmentedEkf::addState(double variance) {
    const std::size_t n = n_ + 1;
    std::vector<double> grown(n * n, 0.0);
    for (std::size_t i = 0; i < n_; ++i) {
        for (std::size_t j = 0; j < n_; ++j) {
            grown[i * n + j] = at(i, j);
        }
    }
    grown[n_ * n + n_] = variance;
    p_ = std::move(grown);
    x_.push_back(0.0);
    n_ = n;
    return n_ - 1;
}

void AugmentedEkf::mapErrorTerms(const SurfaceHit& hit, double cosine, std::vector<Term>& terms) {
    const WorldPoint point = hit.point;
    const WorldPoint normal = hit.normal;
    constexpr double kWallAngle = 0.98480775301220802;  // cos 10 deg
    constexpr double kWallGap = 1.0;                    // m
    if (model_.map_sigma > 0.0) {
        Wall* wall = nullptr;
        double s = 0.0;
        for (Wall& w : walls_) {
            if (hit.feature >= 0) {
                if (w.feature == hit.feature) {
                    wall = &w;
                    s = -w.normal.y * point.x + w.normal.x * point.y;
                    break;
                }
                continue;
            }
            if (w.normal.x * normal.x + w.normal.y * normal.y < kWallAngle) {
                continue;
            }
            if (std::fabs(w.normal.x * point.x + w.normal.y * point.y - w.rho) > 0.5 * resolution_) {
                continue;
            }
            const double along = -w.normal.y * point.x + w.normal.x * point.y;
            if (along < w.s_min - kWallGap || along > w.s_max + kWallGap) {
                continue;
            }
            wall = &w;
            s = along;
            break;
        }
        if (wall == nullptr) {
            s = -normal.y * point.x + normal.x * point.y;
            const std::size_t offset = addState(model_.map_sigma * model_.map_sigma);
            addState(model_.map_tilt * model_.map_tilt);
            walls_.push_back({hit.feature, normal, normal.x * point.x + normal.y * point.y, s, s, s, offset,
                              scan_count_});
            wall = &walls_.back();
        }
        wall->s_min = std::min(wall->s_min, s);
        wall->s_max = std::max(wall->s_max, s);
        wall->last_used = scan_count_;
        terms.push_back({wall->state, 1.0 / cosine});
        terms.push_back({wall->state + 1, (s - wall->s0) / cosine});
    }
    if (model_.map_residual > 0.0) {
        const int direction = std::fabs(normal.x) >= std::fabs(normal.y) ? (normal.x > 0.0 ? 0 : 1)
                                                                           : (normal.y > 0.0 ? 2 : 3);
        const auto ix = static_cast<long long>(std::floor(point.x / resolution_));
        const auto iy = static_cast<long long>(std::floor(point.y / resolution_));
        const long long key = ((ix + (1LL << 20)) * (1LL << 21) + (iy + (1LL << 20))) * 4 + direction;
        Patch* patch = nullptr;
        for (Patch& candidate : patches_) {
            if (candidate.key == key) {
                patch = &candidate;
                break;
            }
        }
        if (patch == nullptr) {
            patches_.push_back({key, addState(model_.map_residual * model_.map_residual), scan_count_});
            patch = &patches_.back();
        }
        patch->last_used = scan_count_;
        terms.push_back({patch->state, 1.0 / cosine});
    }
}

void AugmentedEkf::evictMapStates() {
    if (n_ - map_begin_ <= model_.max_map_states) {
        return;
    }
    // Least recently used first; a wall frees 2 states, a patch 1.
    struct Item {
        std::size_t last_used;
        bool wall;
        std::size_t k;
    };
    std::vector<Item> items;
    for (std::size_t k = 0; k < walls_.size(); ++k) {
        items.push_back({walls_[k].last_used, true, k});
    }
    for (std::size_t k = 0; k < patches_.size(); ++k) {
        items.push_back({patches_[k].last_used, false, k});
    }
    std::stable_sort(items.begin(), items.end(),
                     [](const Item& a, const Item& b) { return a.last_used < b.last_used; });
    std::vector<bool> drop(n_, false);
    std::vector<bool> drop_wall(walls_.size(), false), drop_patch(patches_.size(), false);
    std::size_t states = n_ - map_begin_;
    for (const Item& item : items) {
        if (states <= model_.max_map_states) {
            break;
        }
        if (item.wall) {
            drop[walls_[item.k].state] = drop[walls_[item.k].state + 1] = true;
            drop_wall[item.k] = true;
            states -= 2;
        } else {
            drop[patches_[item.k].state] = true;
            drop_patch[item.k] = true;
            states -= 1;
        }
    }
    std::vector<std::size_t> new_index(n_, 0);
    std::vector<std::size_t> keep;
    for (std::size_t i = 0; i < n_; ++i) {
        if (!drop[i]) {
            new_index[i] = keep.size();
            keep.push_back(i);
        }
    }
    const std::size_t n = keep.size();
    std::vector<double> shrunk(n * n);
    std::vector<double> x(n);
    for (std::size_t i = 0; i < n; ++i) {
        x[i] = x_[keep[i]];
        for (std::size_t j = 0; j < n; ++j) {
            shrunk[i * n + j] = at(keep[i], keep[j]);
        }
    }
    std::vector<Wall> walls;
    for (std::size_t k = 0; k < walls_.size(); ++k) {
        if (!drop_wall[k]) {
            Wall w = walls_[k];
            w.state = new_index[w.state];
            walls.push_back(w);
        }
    }
    std::vector<Patch> patches;
    for (std::size_t k = 0; k < patches_.size(); ++k) {
        if (!drop_patch[k]) {
            Patch q = patches_[k];
            q.state = new_index[q.state];
            patches.push_back(q);
        }
    }
    p_ = std::move(shrunk);
    x_ = std::move(x);
    n_ = n;
    walls_ = std::move(walls);
    patches_ = std::move(patches);
}

UpdateStats AugmentedEkf::update(const std::vector<double>& offsets,
                                 const std::vector<double>& ranges, const std::vector<bool>& hits,
                                 double max_range, double range_sigma, double gate) {
    ++scan_count_;
    UpdateStats stats;
    struct Beam {
        double innovation;     // z - h(prior)
        std::size_t index[8];  // state indices with nonzero H
        double h[8];
        std::size_t count;
        double noise;          // white variance (plus linearization variance)
    };
    const bool model_grazing = model_.grazing == GrazingPolicy::Model;
    const double min_cosine =
        model_grazing ? kMinModelIncidenceCosine : std::cos(model_.max_incidence);
    const bool map_states = model_.map_sigma > 0.0 || model_.map_residual > 0.0;
    std::vector<Term> terms;
    const Pose prior = pose();
    Matrix3 pose_cov{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            pose_cov[3 * i + j] = at(i, j);
        }
    }
    std::vector<Beam> beams;
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        if (!hits[i]) {
            continue;
        }
        const double angle = prior.theta + offsets[i];
        const SurfaceHit predicted = surface_.cast({prior.x, prior.y}, angle, max_range);
        if (!predicted.hit || predicted.range <= 0.0 || !predicted.smooth) {
            continue;
        }
        const double cosine = incidenceCosine(predicted, angle);
        if (cosine < min_cosine) {
            continue;  // grazing (cutoff policy), or numerically degenerate
        }
        if (model_grazing) {
            // A grazing beam slides along the wall by (normal error) x tan(alpha): when
            // the slide (2 sigma) can carry it past the end of the wall, the real beam
            // may hit something else entirely. Normal error: pose along the normal
            // plus the wall's own offset.
            const WorldPoint n = predicted.normal;
            const double pose_normal = n.x * n.x * at(0, 0) + 2.0 * n.x * n.y * at(0, 1) + n.y * n.y * at(1, 1);
            const double sigma = std::sqrt(pose_normal + model_.map_sigma * model_.map_sigma);
            const double tangent = std::sqrt(std::max(1.0 - cosine * cosine, 0.0)) / cosine;
            if (predicted.edge_distance < 2.0 * sigma * tangent) {
                continue;
            }
        }
        Beam beam{};
        const std::array<double, 3> jacobian = surfaceJacobian(predicted, angle);
        for (std::size_t k = 0; k < 3; ++k) {
            beam.index[k] = k;
            beam.h[k] = jacobian[k];
        }
        beam.count = 3;
        if (has_bias_) {
            beam.index[beam.count] = 3;
            beam.h[beam.count++] = 1.0;
        }
        if (map_states) {
            terms.clear();
            mapErrorTerms(predicted, cosine, terms);
            for (const Term& term : terms) {
                beam.index[beam.count] = term.index;
                beam.h[beam.count++] = term.h;
            }
        }
        beam.noise = range_sigma * range_sigma;
        if (model_grazing) {
            // 0.5 tr(G P G P): variance of the second-order term of the range.
            const std::array<double, 9> g = surfaceHessian(predicted, angle);
            Matrix3 gp{};
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    for (int k = 0; k < 3; ++k) {
                        gp[3 * r + c] += g[3 * r + k] * pose_cov[3 * k + c];
                    }
                }
            }
            double trace = 0.0;
            for (int r = 0; r < 3; ++r) {
                for (int k = 0; k < 3; ++k) {
                    trace += gp[3 * r + k] * gp[3 * k + r];
                }
            }
            beam.noise += 0.5 * trace;
            // What offset and tilt leave of the map error (measured: about 0.3 of the
            // offset std, perception_experiment maperror), magnified by 1/cos(alpha)
            // like any error along the normal; unless residual states model it.
            if (model_.map_residual == 0.0) {
                const double residual = kResidualMapFraction * model_.map_sigma / cosine;
                beam.noise += residual * residual;
            }
        }
        // Predicted reading: surface range plus the current bias and map-error estimates.
        beam.innovation = ranges[i] - predicted.range;
        for (std::size_t a = 3; a < beam.count; ++a) {
            beam.innovation -= beam.h[a] * x_[beam.index[a]];
        }
        beams.push_back(beam);
    }
    // Gate every beam against the prior (order-independent), then fuse one at a time.
    std::vector<Beam> accepted;
    for (const Beam& beam : beams) {
        double variance = beam.noise;
        for (std::size_t a = 0; a < beam.count; ++a) {
            for (std::size_t b = 0; b < beam.count; ++b) {
                variance += beam.h[a] * beam.h[b] * at(beam.index[a], beam.index[b]);
            }
        }
        if (beam.innovation * beam.innovation > gate * gate * variance) {
            ++stats.rejected;
        } else {
            accepted.push_back(beam);
        }
    }
    const std::vector<double> x_prior = x_;
    std::vector<double> ph(n_);
    for (const Beam& beam : accepted) {
        // Innovation relinearized at the prior: z - h(x0) - H (x - x0).
        double innovation = beam.innovation;
        for (std::size_t a = 0; a < beam.count; ++a) {
            double dx = x_[beam.index[a]] - x_prior[beam.index[a]];
            if (beam.index[a] == 2) {
                dx = wrapAngle(dx);
            }
            innovation -= beam.h[a] * dx;
        }
        std::fill(ph.begin(), ph.end(), 0.0);
        for (std::size_t a = 0; a < beam.count; ++a) {
            const std::size_t col = beam.index[a];
            for (std::size_t r = 0; r < n_; ++r) {
                ph[r] += at(r, col) * beam.h[a];
            }
        }
        double variance = beam.noise;
        for (std::size_t a = 0; a < beam.count; ++a) {
            variance += beam.h[a] * ph[beam.index[a]];
        }
        for (std::size_t r = 0; r < n_; ++r) {
            x_[r] += ph[r] * innovation / variance;
        }
        for (std::size_t r = 0; r < n_; ++r) {
            const double scaled = ph[r] / variance;
            for (std::size_t c = 0; c < n_; ++c) {
                at(r, c) -= scaled * ph[c];
            }
        }
        ++stats.used;
    }
    x_[2] = wrapAngle(x_[2]);
    for (std::size_t i = 0; i < n_; ++i) {
        for (std::size_t j = i + 1; j < n_; ++j) {
            const double mean = 0.5 * (at(i, j) + at(j, i));
            at(i, j) = mean;
            at(j, i) = mean;
        }
    }
    evictMapStates();
    return stats;
}

std::vector<LocalizationSample> runLocalization(const GridMap& map,
                                                const std::vector<TrajectorySample>& trajectory,
                                                const std::vector<LidarScan>& scans,
                                                const LidarConfig& lidar,
                                                const EkfConfig& config,
                                                const std::vector<bool>& kidnapped) {
    std::vector<LocalizationSample> samples;
    if (!config.enabled || trajectory.empty()) {
        return samples;
    }
    const bool with_lidar = !scans.empty() && scans.size() == trajectory.size();
    const std::vector<double> offsets = beamOffsets(lidar);
    const double position_var = config.initial_position_sigma * config.initial_position_sigma;
    const double heading_var = config.initial_heading_sigma * config.initial_heading_sigma;
    // The start pose is known only to within the initial uncertainty: the filter
    // (and dead reckoning) start from the true pose plus an error drawn from it.
    std::mt19937 rng(config.odometry.seed);
    std::normal_distribution<double> unit(0.0, 1.0);
    const Pose truth0 = poseOf(trajectory.front().state);
    const double dx = config.initial_position_sigma * unit(rng);
    const double dy = config.initial_position_sigma * unit(rng);
    const double dtheta = config.initial_heading_sigma * unit(rng);
    const Pose start{truth0.x + dx, truth0.y + dy, wrapAngle(truth0.theta + dtheta)};
    const Matrix3 initial{position_var, 0.0, 0.0, 0.0, position_var, 0.0, 0.0, 0.0, heading_var};
    // The classic model runs the 3-state filter (bit-identical to before); any other
    // measurement model runs the augmented one.
    const bool classic = config.model.classic();
    PoseEkf ekf(start, initial);
    const std::unique_ptr<SurfaceModel> surface =
        classic ? nullptr : makeSurface(map, config.model.surface);
    std::optional<AugmentedEkf> augmented;
    if (!classic) {
        augmented.emplace(start, initial, *surface, config.model, map.resolution());
    }
    Pose odometry = start;

    samples.reserve(trajectory.size());
    for (std::size_t k = 0; k < trajectory.size(); ++k) {
        LocalizationSample sample;
        sample.time = trajectory[k].state.time;
        sample.truth = poseOf(trajectory[k].state);
        if (k > 0) {
            // Noisy odometry of the true motion since the previous sample. The error
            // variances follow the commanded motion (see the header).
            const OdometryIncrement truth =
                trueIncrement(trajectory[k - 1].state, trajectory[k].state);
            const double dt = trajectory[k].state.time - trajectory[k - 1].state.time;
            const OdometryIncrement commanded{trajectory[k].linear_velocity * dt,
                                              trajectory[k].angular_velocity * dt};
            const std::array<double, 2> var = odometryVariances(commanded, config.odometry);
            const double distance_noise = std::sqrt(var[0]) * unit(rng);
            const double rotation_noise = std::sqrt(var[1]) * unit(rng);
            const bool jump = k < kidnapped.size() && kidnapped[k];
            const OdometryIncrement measured =
                jump ? OdometryIncrement{0.0, 0.0}
                     : OdometryIncrement{truth.distance + distance_noise, truth.rotation + rotation_noise};
            odometry = applyIncrement(odometry, measured);
            if (classic) {
                ekf.predict(measured, var);
            } else {
                augmented->predict(measured, var, dt);
            }
        }
        if (with_lidar) {
            sample.update =
                classic ? ekf.update(map, offsets, scans[k].ranges, scans[k].hits,
                                     lidar.max_range, config.range_sigma, config.gate)
                        : augmented->update(offsets, scans[k].ranges, scans[k].hits,
                                            lidar.max_range, config.range_sigma, config.gate);
        }
        sample.odometry = odometry;
        sample.estimate = classic ? ekf.pose() : augmented->pose();
        sample.covariance = classic ? ekf.covariance() : augmented->covariance();
        if (!classic) {
            sample.bias = augmented->bias();
            sample.map_states = augmented->mapStates();
        }
        sample.nees = quadratic(poseError(sample.truth, sample.estimate),
                                inverse(sample.covariance));
        samples.push_back(sample);
    }
    return samples;
}

LocalizationSummary summarizeLocalization(const std::vector<LocalizationSample>& samples) {
    LocalizationSummary summary;
    if (samples.empty()) {
        return summary;
    }
    double est_sq = 0.0, odom_sq = 0.0, est_heading_sq = 0.0, odom_heading_sq = 0.0;
    double nees = 0.0;
    for (const LocalizationSample& s : samples) {
        const std::array<double, 3> est = poseError(s.truth, s.estimate);
        const std::array<double, 3> odom = poseError(s.truth, s.odometry);
        const double est_pos = std::hypot(est[0], est[1]);
        const double odom_pos = std::hypot(odom[0], odom[1]);
        est_sq += est_pos * est_pos;
        odom_sq += odom_pos * odom_pos;
        est_heading_sq += est[2] * est[2];
        odom_heading_sq += odom[2] * odom[2];
        summary.estimate.position_max = std::max(summary.estimate.position_max, est_pos);
        summary.odometry.position_max = std::max(summary.odometry.position_max, odom_pos);
        summary.estimate.position_final = est_pos;
        summary.odometry.position_final = odom_pos;
        summary.beams_used += s.update.used;
        summary.beams_rejected += s.update.rejected;
        nees += s.nees;
    }
    const double n = static_cast<double>(samples.size());
    summary.estimate.position_rms = std::sqrt(est_sq / n);
    summary.odometry.position_rms = std::sqrt(odom_sq / n);
    summary.estimate.heading_rms = std::sqrt(est_heading_sq / n);
    summary.odometry.heading_rms = std::sqrt(odom_heading_sq / n);
    summary.mean_nees = nees / n;
    return summary;
}

}  // namespace amr
