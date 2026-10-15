#include "obstacle_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "diff_drive.hpp"

namespace amr {

namespace {

constexpr double kPi = 3.14159265358979323846;

double distanceBetween(WorldPoint a, WorldPoint b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

/// Solve the symmetric 3 x 3 (or 2 x 2 when n == 2) system A x = b by Cramer's rule.
bool solve(int n, const double a[3][3], const double b[3], double x[3]) {
    if (n == 2) {
        const double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        if (std::fabs(det) < 1e-18) {
            return false;
        }
        x[0] = (b[0] * a[1][1] - a[0][1] * b[1]) / det;
        x[1] = (a[0][0] * b[1] - b[0] * a[1][0]) / det;
        return true;
    }
    const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                       a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                       a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (std::fabs(det) < 1e-18) {
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        double m[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m[i][j] = j == k ? b[i] : a[i][j];
            }
        }
        x[k] = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) /
               det;
    }
    return true;
}

/// Gauss-Newton fit of |p - c| = R: center (and radius when `fit_radius`).
bool gaussNewton(const std::vector<WorldPoint>& points, WorldPoint& c, double& r, bool fit_radius) {
    const int n = fit_radius ? 3 : 2;
    for (int iteration = 0; iteration < 30; ++iteration) {
        double a[3][3] = {}, b[3] = {};
        for (const WorldPoint& p : points) {
            const double dx = p.x - c.x;
            const double dy = p.y - c.y;
            const double d = std::hypot(dx, dy);
            if (d < 1e-12) {
                return false;
            }
            const double residual = d - r;
            const double j[3] = {-dx / d, -dy / d, -1.0};
            for (int i = 0; i < n; ++i) {
                b[i] -= j[i] * residual;
                for (int k = 0; k < n; ++k) {
                    a[i][k] += j[i] * j[k];
                }
            }
        }
        double step[3] = {};
        if (!solve(n, a, b, step)) {
            return false;
        }
        c.x += step[0];
        c.y += step[1];
        if (fit_radius) {
            r += step[2];
        }
        if (std::fabs(step[0]) + std::fabs(step[1]) + (fit_radius ? std::fabs(step[2]) : 0.0) < 1e-9) {
            return true;
        }
    }
    return true;
}

}  // namespace

Detection fitDisc(const std::vector<WorldPoint>& points, WorldPoint origin,
                  const DetectionConfig& config) {
    Detection d;
    d.points = points.size();
    WorldPoint mean{0.0, 0.0};
    for (const WorldPoint& p : points) {
        mean.x += p.x;
        mean.y += p.y;
    }
    mean.x /= static_cast<double>(points.size());
    mean.y /= static_cast<double>(points.size());
    const double range = std::max(distanceBetween(origin, mean), 1e-9);
    const WorldPoint away{(mean.x - origin.x) / range, (mean.y - origin.y) / range};
    const auto behind = [&](double r) { return WorldPoint{mean.x + r * away.x, mean.y + r * away.y}; };
    // Full fit first: with 3+ points spread over 60+ degrees of the disc the radius is
    // observable; otherwise use the prior radius.
    if (points.size() >= 3) {
        WorldPoint c = behind(config.prior_radius);
        double r = config.prior_radius;
        if (gaussNewton(points, c, r, true) && r >= config.min_radius && r <= config.max_radius &&
            (c.x - mean.x) * away.x + (c.y - mean.y) * away.y > 0.0) {
            double lo = kPi, hi = -kPi;
            const double reference = std::atan2(origin.y - c.y, origin.x - c.x);
            for (const WorldPoint& p : points) {
                const double angle = wrapAngle(std::atan2(p.y - c.y, p.x - c.x) - reference);
                lo = std::min(lo, angle);
                hi = std::max(hi, angle);
            }
            if (hi - lo >= kPi / 3.0) {
                d.center = c;
                d.radius = r;
                d.radius_fitted = true;
                d.sigma = 0.03;
                return d;
            }
        }
    }
    WorldPoint c = behind(config.prior_radius);
    double r = config.prior_radius;
    if (!gaussNewton(points, c, r, false) || (c.x - mean.x) * away.x + (c.y - mean.y) * away.y <= 0.0 ||
        distanceBetween(c, mean) > 1.5 * config.prior_radius) {
        c = behind(config.prior_radius);
    }
    d.center = c;
    d.radius = config.prior_radius;
    d.sigma = 0.06;
    return d;
}

std::vector<Detection> detectMovingObstacles(const GridMap& map, const LidarScan& scan,
                                             double max_range, bool full_circle,
                                             const DetectionConfig& config) {
    const std::size_t n = scan.ranges.size();
    std::vector<bool> foreign(n, false);
    std::vector<WorldPoint> points(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (!scan.hits[i]) {
            continue;
        }
        const RayHit expected = castRay(map, scan.origin, scan.angles[i], max_range);
        const double expected_range = expected.hit ? expected.range : std::numeric_limits<double>::infinity();
        if (scan.ranges[i] < expected_range - config.threshold) {
            foreign[i] = true;
            points[i] = {scan.origin.x + scan.ranges[i] * std::cos(scan.angles[i]),
                         scan.origin.y + scan.ranges[i] * std::sin(scan.angles[i])};
        }
    }
    // Start the cluster scan at a beam that is not foreign, so a cluster never wraps
    // across the start of a full circle.
    std::size_t start = 0;
    if (full_circle) {
        while (start < n && foreign[start]) {
            ++start;
        }
        if (start == n) {
            start = 0;
        }
    }
    std::vector<std::vector<WorldPoint>> clusters;
    std::vector<WorldPoint> cluster;
    const auto flush = [&]() {
        if (!cluster.empty()) {
            clusters.push_back(cluster);
        }
        cluster.clear();
    };
    const std::size_t count = full_circle ? n : n - start;
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t i = (start + k) % n;
        if (!foreign[i]) {
            flush();
            continue;
        }
        if (!cluster.empty() && distanceBetween(cluster.back(), points[i]) > config.join_distance) {
            flush();
        }
        cluster.push_back(points[i]);
    }
    flush();
    // One obstacle can be split into several clusters (a wall or another obstacle in
    // front of part of it): merge clusters whose discs overlap, then fit.
    std::vector<Detection> fitted;
    for (const auto& members : clusters) {
        fitted.push_back(fitDisc(members, scan.origin, config));
    }
    for (bool merged = true; merged;) {
        merged = false;
        for (std::size_t a = 0; a < clusters.size() && !merged; ++a) {
            for (std::size_t b = a + 1; b < clusters.size() && !merged; ++b) {
                if (distanceBetween(fitted[a].center, fitted[b].center) < 0.8 * (fitted[a].radius + fitted[b].radius)) {
                    clusters[a].insert(clusters[a].end(), clusters[b].begin(), clusters[b].end());
                    clusters.erase(clusters.begin() + static_cast<long>(b));
                    fitted.erase(fitted.begin() + static_cast<long>(b));
                    fitted[a] = fitDisc(clusters[a], scan.origin, config);
                    merged = true;
                }
            }
        }
    }
    std::vector<Detection> detections;
    for (std::size_t k = 0; k < clusters.size(); ++k) {
        if (clusters[k].size() >= config.min_points) {
            detections.push_back(fitted[k]);
        }
    }
    return detections;
}

double Track::positionSigma() const {
    return std::sqrt(std::max(p[0], p[5]));
}

void ObstacleTracker::predict(Track& track, double dt) const {
    if (dt <= 0.0) {
        return;
    }
    // x' = F x with F = [I dt I; 0 I]; P' = F P F^T + Q (white acceleration).
    track.x[0] += dt * track.x[2];
    track.x[1] += dt * track.x[3];
    double f[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};
    double fp[16] = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                fp[4 * i + j] += f[4 * i + k] * track.p[4 * k + j];
            }
        }
    }
    double next[16] = {};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                next[4 * i + j] += fp[4 * i + k] * f[4 * j + k];
            }
        }
    }
    const double q = config_.acceleration_noise * config_.acceleration_noise;
    const double q11 = q * dt * dt * dt * dt / 4.0;
    const double q13 = q * dt * dt * dt / 2.0;
    const double q33 = q * dt * dt;
    next[0] += q11;
    next[5] += q11;
    next[2] += q13;
    next[8] += q13;
    next[7] += q13;
    next[13] += q13;
    next[10] += q33;
    next[15] += q33;
    std::copy(next, next + 16, track.p);
}

void ObstacleTracker::learnLane(Track& track) const {
    const std::size_t n = track.history.size();
    if (n < 10) {
        return;
    }
    double mx = 0.0, my = 0.0;
    for (const WorldPoint& p : track.history) {
        mx += p.x;
        my += p.y;
    }
    mx /= static_cast<double>(n);
    my /= static_cast<double>(n);
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const WorldPoint& p : track.history) {
        sxx += (p.x - mx) * (p.x - mx);
        sxy += (p.x - mx) * (p.y - my);
        syy += (p.y - my) * (p.y - my);
    }
    const double angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    WorldPoint axis{std::cos(angle), std::sin(angle)};
    if (track.lane && axis.x * track.lane_axis.x + axis.y * track.lane_axis.y < 0.0) {
        axis = {-axis.x, -axis.y};  // keep a consistent orientation
    }
    double perpendicular = 0.0, lo = 0.0, hi = 0.0;
    for (const WorldPoint& p : track.history) {
        const double s = (p.x - mx) * axis.x + (p.y - my) * axis.y;
        const double t = -(p.x - mx) * axis.y + (p.y - my) * axis.x;
        perpendicular += t * t;
        lo = std::min(lo, s);
        hi = std::max(hi, s);
    }
    perpendicular = std::sqrt(perpendicular / static_cast<double>(n));
    if (hi - lo < 0.5 || perpendicular > config_.line_tolerance) {
        track.lane = false;
        return;
    }
    track.lane = true;
    track.lane_center = {mx, my};
    track.lane_axis = axis;
    // Reversals: the velocity along the axis changes sign (clearly, both sides). The
    // turning point is the farthest position reached before the reversal.
    const double v = track.x[2] * axis.x + track.x[3] * axis.y;
    const int sign = v > config_.reversal_speed ? 1 : v < -config_.reversal_speed ? -1 : 0;
    if (sign != 0 && track.last_axis_sign != 0 && sign != track.last_axis_sign) {
        // The extreme of the history in the old direction of travel (recent part),
        // and whether the turn was watched: no detection gap between a sample
        // before the extreme and now.
        double extreme = track.last_axis_sign > 0 ? -1e300 : 1e300;
        std::size_t extreme_index = n - 1;
        const std::size_t recent = std::min<std::size_t>(n, 80);
        for (std::size_t k = n - recent; k < n; ++k) {
            const WorldPoint& p = track.history[k];
            const double s = (p.x - mx) * axis.x + (p.y - my) * axis.y;
            if (track.last_axis_sign > 0 ? s > extreme : s < extreme) {
                extreme = s;
                extreme_index = k;
            }
        }
        bool watched = extreme_index > 0;
        for (std::size_t k = extreme_index; watched && k < n; ++k) {
            watched = track.history_time[k] - track.history_time[k - 1] <= config_.max_detection_gap;
        }
        if (!watched) {
            // Turned out of sight: where is unknown.
        } else if (track.last_axis_sign > 0) {
            track.high_end = track.has_high_end ? std::max(track.high_end, extreme) : extreme;
            track.has_high_end = true;
        } else {
            track.low_end = track.has_low_end ? std::min(track.low_end, extreme) : extreme;
            track.has_low_end = true;
        }
    }
    if (sign != 0) {
        track.last_axis_sign = sign;
    }
}

bool ObstacleTracker::contradicted(const Track& track, const GridMap& map, const LidarScan& scan,
                                   double max_range) const {
    const WorldPoint c = track.position();
    const double distance = distanceBetween(scan.origin, c);
    const double r = track.radius;
    if (distance <= r || distance - r > max_range) {
        return false;  // inside it, or out of range: no information
    }
    const double bearing = std::atan2(c.y - scan.origin.y, c.x - scan.origin.x);
    const double half_width = std::asin(std::min(1.0, r / distance));
    int passed = 0;
    for (std::size_t i = 0; i < scan.angles.size(); ++i) {
        const double offset = wrapAngle(scan.angles[i] - bearing);
        if (std::fabs(offset) > 0.7 * half_width) {
            continue;  // only the disc's core: beams near its edge are ambiguous
        }
        // Where this beam would enter the disc.
        const double along = distance * std::cos(offset);
        const double across = distance * std::sin(offset);
        const double front = along - std::sqrt(std::max(r * r - across * across, 0.0));
        const RayHit wall = castRay(map, scan.origin, scan.angles[i], max_range);
        if (wall.hit && wall.range < front + 0.05) {
            continue;  // the map hides the disc along this beam
        }
        const double measured = scan.hits[i] ? scan.ranges[i] : max_range;
        if (front < max_range && measured > front + 0.15) {
            ++passed;  // the beam went on through where the obstacle should be
        }
    }
    return passed > 0;
}

void ObstacleTracker::update(double time, const std::vector<Detection>& detections, const GridMap& map,
                             const LidarScan& scan, double max_range) {
    const double dt = tracks_.empty() && next_id_ == 0 ? 0.0 : time - time_;
    for (Track& track : tracks_) {
        predict(track, dt);
    }
    time_ = time;
    // Candidate pairs: inside the chi-square gate, or (a possible maneuver) beyond it
    // but within the hard distance limit. Gated pairs first, closest first.
    struct Pair {
        std::size_t track, detection;
        double distance;  // Mahalanobis squared
        bool gated;
    };
    std::vector<Pair> pairs;
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
        const Track& track = tracks_[t];
        for (std::size_t d = 0; d < detections.size(); ++d) {
            const double dx = detections[d].center.x - track.x[0];
            const double dy = detections[d].center.y - track.x[1];
            if (std::hypot(dx, dy) > config_.max_association) {
                continue;
            }
            const double r2 = detections[d].sigma * detections[d].sigma;
            const double s00 = track.p[0] + r2, s01 = track.p[1], s11 = track.p[5] + r2;
            const double det = s00 * s11 - s01 * s01;
            const double m = (dx * dx * s11 - 2.0 * dx * dy * s01 + dy * dy * s00) / det;
            pairs.push_back({t, d, m, m <= config_.gate});
        }
    }
    // Gated pairs first, then confirmed tracks before tentative ones (a stray fragment
    // must not steal an established track's detection), then the closest.
    std::sort(pairs.begin(), pairs.end(), [this](const Pair& a, const Pair& b) {
        if (a.gated != b.gated) {
            return a.gated;
        }
        const bool ca = tracks_[a.track].confirmed, cb = tracks_[b.track].confirmed;
        return ca != cb ? ca : a.distance < b.distance;
    });
    std::vector<bool> track_used(tracks_.size(), false), detection_used(detections.size(), false);
    for (const Pair& pair : pairs) {
        if (track_used[pair.track] || detection_used[pair.detection]) {
            continue;
        }
        Track& track = tracks_[pair.track];
        // Beyond the gate only for confirmed tracks (a reversal); tentative ones may
        // simply be wrong.
        if (!pair.gated && !track.confirmed) {
            continue;
        }
        track_used[pair.track] = true;
        detection_used[pair.detection] = true;
        const Detection& d = detections[pair.detection];
        track.recent_nis[0] = track.recent_nis[1];
        track.recent_nis[1] = track.recent_nis[2];
        track.recent_nis[2] = pair.distance;
        const double window = track.recent_nis[0] + track.recent_nis[1] + track.recent_nis[2];
        if (pair.distance > config_.maneuver_gate ||
            (track.hits >= 4 && window > config_.maneuver_window_gate)) {
            track.recent_nis[0] = track.recent_nis[1] = track.recent_nis[2] = 0.0;
            // The constant-velocity model cannot explain this: the obstacle changed
            // velocity. Forget the velocity (and its correlation with the position).
            const double v2 = config_.initial_speed_sigma * config_.initial_speed_sigma;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    if (i >= 2 || j >= 2) {
                        track.p[4 * i + j] = (i == j) ? v2 : 0.0;
                    }
                }
            }
            ++track.maneuvers;
        }
        if (track.hits == 1 && !track.history_time.empty() && time > track.history_time.back()) {
            // Two-point initialization: the second detection fixes the velocity
            // (difference of the two positions), with its covariance. A Kalman update
            // from velocity 0 would need several scans to catch up with a fast obstacle.
            const double dt0 = time - track.history_time.back();
            const WorldPoint first = track.history.back();
            const double r0 = track.p[0];  // variance of the first position
            const double r2 = d.sigma * d.sigma;
            track.x[0] = d.center.x;
            track.x[1] = d.center.y;
            track.x[2] = (d.center.x - first.x) / dt0;
            track.x[3] = (d.center.y - first.y) / dt0;
            std::fill(track.p, track.p + 16, 0.0);
            track.p[0] = track.p[5] = r2;
            track.p[2] = track.p[8] = track.p[7] = track.p[13] = r2 / dt0;
            track.p[10] = track.p[15] = (r0 + r2) / (dt0 * dt0);
            if (d.radius_fitted) {
                track.radius = 0.5 * (track.radius + d.radius);
            }
            ++track.hits;
            track.misses = 0;
            track.last_seen = time;
            track.confirmed = track.confirmed || track.hits >= config_.confirm_hits;
            track.history.push_back(track.position());
            track.history_time.push_back(time);
            continue;
        }
        // Kalman update with the position measurement.
        const double r2 = d.sigma * d.sigma;
        const double s00 = track.p[0] + r2, s01 = track.p[1], s10 = track.p[4], s11 = track.p[5] + r2;
        const double det = s00 * s11 - s01 * s10;
        const double i00 = s11 / det, i01 = -s01 / det, i10 = -s10 / det, i11 = s00 / det;
        double k[4][2];
        for (int i = 0; i < 4; ++i) {
            const double pi0 = track.p[4 * i + 0];
            const double pi1 = track.p[4 * i + 1];
            k[i][0] = pi0 * i00 + pi1 * i10;
            k[i][1] = pi0 * i01 + pi1 * i11;
        }
        const double y0 = d.center.x - track.x[0];
        const double y1 = d.center.y - track.x[1];
        for (int i = 0; i < 4; ++i) {
            track.x[i] += k[i][0] * y0 + k[i][1] * y1;
        }
        double next[16];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                next[4 * i + j] = track.p[4 * i + j] - k[i][0] * track.p[0 * 4 + j] - k[i][1] * track.p[1 * 4 + j];
            }
        }
        for (int i = 0; i < 4; ++i) {  // symmetrize
            for (int j = 0; j < 4; ++j) {
                track.p[4 * i + j] = 0.5 * (next[4 * i + j] + next[4 * j + i]);
            }
        }
        if (d.radius_fitted) {
            track.radius = 0.8 * track.radius + 0.2 * d.radius;
        }
        ++track.hits;
        track.misses = 0;
        track.last_seen = time;
        track.confirmed = track.confirmed || track.hits >= config_.confirm_hits;
        track.history.push_back(track.position());
        track.history_time.push_back(time);
        if (track.history.size() > 600) {
            track.history.erase(track.history.begin());
            track.history_time.erase(track.history_time.begin());
        }
        if (config_.prediction == MotionPrediction::LearnedReversals) {
            learnLane(track);
        }
    }
    // Undetected tracks: negative evidence or coasting.
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
        if (!track_used[t] && contradicted(tracks_[t], map, scan, max_range)) {
            ++tracks_[t].misses;
        }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& track) {
                                     if (!track.confirmed) {
                                         return track.misses >= 1 || time - track.last_seen > 0.5;
                                     }
                                     return track.misses >= config_.delete_misses ||
                                            time - track.last_seen > config_.coast_time ||
                                            track.positionSigma() > config_.max_position_sigma;
                                 }),
                  tracks_.end());
    // Births, except where a detection overlaps a confirmed track's disc (a fragment
    // of an obstacle already tracked).
    for (std::size_t d = 0; d < detections.size(); ++d) {
        if (detection_used[d]) {
            continue;
        }
        const bool overlaps = std::any_of(tracks_.begin(), tracks_.end(), [&](const Track& track) {
            return track.confirmed && distanceBetween(track.position(), detections[d].center) <
                                          track.radius + detections[d].radius;
        });
        if (overlaps) {
            continue;
        }
        Track track;
        track.id = next_id_++;
        track.x[0] = detections[d].center.x;
        track.x[1] = detections[d].center.y;
        const double r2 = detections[d].sigma * detections[d].sigma;
        const double v2 = config_.initial_speed_sigma * config_.initial_speed_sigma;
        track.p[0] = r2;
        track.p[5] = r2;
        track.p[10] = v2;
        track.p[15] = v2;
        track.radius = detections[d].radius;
        track.hits = 1;
        track.last_seen = time;
        track.created = time;
        track.confirmed = config_.confirm_hits <= 1;
        track.history.push_back(track.position());
        track.history_time.push_back(time);
        tracks_.push_back(track);
    }
}

std::vector<PredictedPosition> ObstacleTracker::predictPath(const Track& track, double dt,
                                                            std::size_t steps) const {
    std::vector<PredictedPosition> path;
    path.reserve(steps + 1);
    const double sigma_p = track.positionSigma();
    const double sigma_v = std::sqrt(std::max(track.p[10], track.p[15]));
    const WorldPoint position = track.position();
    const WorldPoint velocity = track.confirmed ? track.velocity() : WorldPoint{0.0, 0.0};
    const bool lane = track.confirmed && track.lane && config_.prediction == MotionPrediction::LearnedReversals;
    double s = 0.0, v = 0.0, lo = -1e300, hi = 1e300;
    WorldPoint offset{0.0, 0.0};
    if (lane) {
        const WorldPoint a = track.lane_axis;
        s = (position.x - track.lane_center.x) * a.x + (position.y - track.lane_center.y) * a.y;
        v = velocity.x * a.x + velocity.y * a.y;
        offset = {position.x - (track.lane_center.x + s * a.x), position.y - (track.lane_center.y + s * a.y)};
        if (track.has_low_end) {
            lo = track.low_end;
        }
        if (track.has_high_end) {
            hi = track.high_end;
        }
        if (lo >= hi) {
            lo = -1e300;
            hi = 1e300;
        }
    }
    for (std::size_t j = 0; j <= steps; ++j) {
        const double tau = static_cast<double>(j) * dt;
        PredictedPosition p;
        if (lane) {
            if (j > 0) {
                s += v * dt;
                // Reflect at the learned turning points (possibly several times).
                for (int bounce = 0; bounce < 4 && (s > hi || s < lo); ++bounce) {
                    if (s > hi) {
                        s = 2.0 * hi - s;
                        v = -std::fabs(v);
                    } else {
                        s = 2.0 * lo - s;
                        v = std::fabs(v);
                    }
                }
            }
            const WorldPoint a = track.lane_axis;
            p.center = {track.lane_center.x + s * a.x + offset.x, track.lane_center.y + s * a.y + offset.y};
        } else {
            p.center = {position.x + velocity.x * tau, position.y + velocity.y * tau};
        }
        p.sigma = std::sqrt(sigma_p * sigma_p + (track.confirmed ? sigma_v * sigma_v * tau * tau : 0.0));
        path.push_back(p);
    }
    return path;
}

}  // namespace amr
