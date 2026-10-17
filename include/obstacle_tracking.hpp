// Moving obstacles seen through the LiDAR alone: map subtraction, clustering, circle
// fitting and multi-target tracking, with a motion predictor that survives reversals.
//
// Detection (detectMovingObstacles). The static map is known, so every beam's
// expected range is known too (castRay from the robot). A return clearly shorter
// than expected (by more than `threshold`) hit something the map does not have.
// Consecutive such returns (in beam order, wrapping round a full circle) whose hit
// points are closer than `join_distance` form a cluster, and each cluster is one
// disc: its center is fitted by least squares to |p - c| = R (Gauss-Newton). With
// enough points spanning enough of the disc, R is fitted too; otherwise R is the
// prior radius. The robot sees only the near side of a disc, so the fitted center
// lies behind the hit points, away from the robot.
//
// Tracking (ObstacleTracker). Each track is a constant-velocity Kalman filter
// (state x, y, vx, vy; white acceleration noise). Each scan:
//   1. predict every track to the scan time;
//   2. associate detections to tracks greedily by Mahalanobis distance, within a
//      chi-square gate (and a hard distance limit);
//   3. maneuvers: an associated detection whose innovation exceeds the gate of the
//      constant-velocity model (NIS), or three in a row that do together (a slow
//      obstacle turning round shifts each innovation only a little, but always the
//      same way), means the obstacle changed velocity: the velocity uncertainty is
//      reset before the update, so the estimate re-converges in a few scans instead
//      of lagging;
//   4. update; unassociated detections start tentative tracks (velocity unknown);
//      a track is confirmed after `confirm_hits` detections;
//   5. a track that was not detected is checked against the scan: if a beam passed
//      through where the obstacle should be, that is negative evidence (a miss);
//      `delete_misses` misses in a row delete it. If the obstacle's predicted
//      position is hidden (behind a wall or another return, or out of range), the
//      track coasts: it keeps being predicted, with growing uncertainty, for up to
//      `coast_time` (it may still be there, just occluded).
//
// Prediction (ObstacleTracker::predictPath). Constant velocity breaks when an
// obstacle turns round: the predicted sweep flips with every reversal, and so do
// the robot's detours (back-and-forth livelocks). A track therefore learns where its
// obstacle turns: once it has moved along a straight line (positions within
// `line_tolerance` of their principal axis), every observed reversal of the velocity
// along that axis records a turning point at that end. The predictor then moves the
// obstacle along the axis at its current speed and reflects it at the known turning
// points (ping-pong), as the obstacle will actually do; without a known end in the
// direction of travel it extrapolates at constant velocity. The prediction's
// position uncertainty (from the track covariance, growing with time) is reported
// with every predicted position, so planners can inflate the obstacle by it.
#pragma once

#include <cstddef>
#include <vector>

#include "grid_map.hpp"
#include "lidar.hpp"
#include "tracking_config.hpp"
#include "types.hpp"

namespace amr {

/// One detected disc.
struct Detection {
    WorldPoint center;
    double radius = 0.0;
    bool radius_fitted = false;
    std::size_t points = 0;
    double sigma = 0.05;  ///< position std of the center estimate, m
};

/// Detect discs that are not in the map in one scan taken from `origin`.
std::vector<Detection> detectMovingObstacles(const GridMap& map, const LidarScan& scan,
                                             double max_range, bool full_circle,
                                             const DetectionConfig& config);

/// Least-squares disc through points (center and radius, or center only for a
/// fixed radius), seen from `origin` (the center starts behind the points).
Detection fitDisc(const std::vector<WorldPoint>& points, WorldPoint origin, const DetectionConfig& config);

/// One predicted position of a track, with its standard deviation.
struct PredictedPosition {
    WorldPoint center;
    double sigma = 0.0;
};

struct Track {
    int id = 0;
    double x[4] = {0.0, 0.0, 0.0, 0.0};  ///< x, y, vx, vy
    double p[16] = {};                   ///< covariance, row-major
    double radius = 0.3;
    std::size_t hits = 0;
    std::size_t misses = 0;
    bool confirmed = false;
    double last_seen = 0.0;
    double created = 0.0;
    std::size_t maneuvers = 0;            ///< velocity resets (reversals detected)
    double recent_nis[3] = {0.0, 0.0, 0.0};  ///< last three normalized innovations
    // Learned lane (LearnedReversals).
    std::vector<WorldPoint> history;      ///< filtered positions, one per detection
    std::vector<double> history_time;     ///< their times, s
    bool lane = false;
    WorldPoint lane_center, lane_axis;
    bool has_low_end = false, has_high_end = false;
    double low_end = 0.0, high_end = 0.0; ///< turning points along the axis
    int last_axis_sign = 0;               ///< sign of the last clear axis velocity

    WorldPoint position() const { return {x[0], x[1]}; }
    WorldPoint velocity() const { return {x[2], x[3]}; }
    double positionSigma() const;         ///< sqrt of the largest position variance
};

class ObstacleTracker {
public:
    explicit ObstacleTracker(const TrackerConfig& config = {}) : config_(config) {}

    /// Process one scan at time t: predict, associate, update, birth, deletion.
    /// `scan` is used for negative evidence about undetected tracks.
    void update(double time, const std::vector<Detection>& detections, const GridMap& map,
                const LidarScan& scan, double max_range);

    const std::vector<Track>& tracks() const { return tracks_; }

    /// Positions of a track every `dt` from now (entry 0 = now), `steps` steps.
    std::vector<PredictedPosition> predictPath(const Track& track, double dt, std::size_t steps) const;

    const TrackerConfig& config() const { return config_; }

private:
    void predict(Track& track, double dt) const;
    void learnLane(Track& track) const;
    /// Visible and absent: a beam passed through where the track's disc should be.
    bool contradicted(const Track& track, const GridMap& map, const LidarScan& scan,
                      double max_range) const;

    TrackerConfig config_;
    std::vector<Track> tracks_;
    double time_ = 0.0;
    int next_id_ = 0;
};

}  // namespace amr
