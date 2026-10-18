// Planning for good localization: prefer routes along walls and features the LiDAR
// can see, where the EKF's uncertainty stays small.
//
// Localizability. A scan taken at cell c (beams and range of the robot's LiDAR, from
// the cell center; a full circle, so the heading does not matter) constrains the pose
// through the exact range Jacobians of section 13 (ekf.hpp):
//
//     I(c) = sum over beams with a return of  H^T H / sigma^2  (+ a weak prior)
//
// The position block of I^-1 is the position covariance that scan alone leaves; its
// largest eigenvalue is the variance in the worst direction. localizability(c) is
// that standard deviation, m: millimetres next to a corner, large along a featureless
// corridor (no information along it) and huge in open space beyond the LiDAR's range
// (no return at all; the prior's 10 m). Beams beyond 78 deg incidence and corner hits
// are left out, as in the classic EKF.
//
// Planning (planForLocalization). A* on the planning map (8-connected, no corner
// cutting) with step cost
//
//     length x (1 + weight x penalty(c)),   penalty(c) = clamp(log2(u(c) / u_good), 0, 4)
//
// for the cell entered (u_good = 5 cm: a cell that pins the position to 5 cm costs
// nothing extra; every doubling of the uncertainty beyond costs one more unit, up to
// 4 for blind cells). Odometry drift grows with the distance driven without good
// observations, so the extra cost is distance in poorly observable places. With
// weight 0 this is the shortest path; the octile heuristic stays admissible because
// every step costs at least its length.
#pragma once

#include <vector>

#include "grid_map.hpp"
#include "lidar.hpp"
#include "types.hpp"

namespace amr {

struct LocalizabilityConfig {
    int beams = 72;
    double max_range = 5.0;
    double range_sigma = 0.05;   ///< m, as assumed by the EKF
    double prior_sigma = 10.0;   ///< m (weak prior that keeps the inverse finite)
};

/// Per-cell worst-direction position std after one scan from the cell center, m
/// (row-major, width x height; occupied cells get +infinity).
std::vector<double> computeLocalizability(const GridMap& map, const LocalizabilityConfig& config);

struct LocalizationPlanConfig {
    double weight = 1.0;     ///< extra cost per unit of penalty, per meter
    double good = 0.05;      ///< u at which the penalty starts, m
    double max_penalty = 4.0;
};

/// The penalty of one cell (see the file comment).
double localizationPenalty(double localizability, const LocalizationPlanConfig& config);

/// Plan on `planning_map` (already inflated) with the localization-aware cost. The
/// result's path_cost holds the exact grid cost of the path (straight, diagonal).
PlanResult planForLocalization(const GridMap& planning_map, GridCoord start, GridCoord goal,
                               const std::vector<double>& localizability,
                               const LocalizationPlanConfig& config);

}  // namespace amr
