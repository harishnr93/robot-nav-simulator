// Configuration of LiDAR obstacle detection and tracking (obstacle_tracking.hpp), in
// a header of its own so the simulator's dynamic-obstacle settings can hold it.
#pragma once

#include <cstddef>

namespace amr {

struct DetectionConfig {
    double threshold = 0.1;        ///< a return this much shorter than the map's is not the map, m
    double join_distance = 0.3;    ///< hit points closer than this belong to one cluster, m
    double prior_radius = 0.3;     ///< disc radius when it cannot be fitted, m
    double min_radius = 0.1;       ///< fitted radii are kept within [min, max], m
    double max_radius = 1.0;
    std::size_t min_points = 2;    ///< smaller clusters are ignored (single stray returns)
};

enum class MotionPrediction {
    ConstantVelocity,  ///< straight on at the estimated velocity
    LearnedReversals,  ///< reflect at the turning points observed so far
};

struct TrackerConfig {
    double acceleration_noise = 0.5;   ///< white acceleration std of the CV model, m/s^2
    double initial_speed_sigma = 1.0;  ///< velocity std of a new track, m/s
    double gate = 13.82;               ///< association gate, chi-square 2 dof (99.9 %)
    double max_association = 1.0;      ///< hard association distance, m
    double maneuver_gate = 9.21;       ///< NIS above this (99 %): reset the velocity uncertainty
    double maneuver_window_gate = 16.81;  ///< sum of the last three NIS above this (chi^2 6 dof, 99 %): reset too
    std::size_t confirm_hits = 2;      ///< detections to confirm a track
    std::size_t delete_misses = 3;     ///< visible misses in a row to delete it
    double coast_time = 10.0;          ///< how long an unseen, possibly occluded track is kept, s
    double max_position_sigma = 1.5;   ///< tracks less certain than this are dropped, m
    double line_tolerance = 0.12;      ///< RMS distance from the principal axis of a straight lane, m
    double reversal_speed = 0.08;      ///< |velocity| along the axis needed on both sides of a reversal, m/s
    double max_detection_gap = 0.3;    ///< a reversal counts only if watched without longer gaps, s
    MotionPrediction prediction = MotionPrediction::LearnedReversals;
};

}  // namespace amr
