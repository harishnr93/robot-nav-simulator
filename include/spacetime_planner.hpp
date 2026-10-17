// Planning over space and time: safe interval path planning (SIPP, Phillips &
// Likhachev 2011) on the grid, against the predicted motion of moving obstacles, with
// the differential drive's turning time (turn-aware planning).
//
// Time is discretized at dt (the simulation step); predictions cover `horizon`
// seconds from now (index k = time k dt), and beyond the horizon every cell counts as
// safe. A cell is unsafe at step k when a predicted obstacle center is closer to the
// cell center than obstacle radius + its prediction inflation + `clearance` (robot
// radius + margin) + half the obstacle's displacement over one step (it moves between
// samples). A cell's safe intervals are the maximal runs of safe steps.
//
// Search states are (cell, heading, safe interval), with the arrival time as cost.
// Moving to a neighbour takes ceil(length / speed / dt) steps; with turn_aware, a
// change of heading first takes ceil(angle / max_angular_speed / dt + turn_overhead)
// steps of turning in place (the turn-then-drive controller of simulator.hpp). From a
// state arrived at step g in an interval ending at E, the robot may wait and turn in
// its cell and leave at any step d <= E; it arrives at step a = d + move. The earliest
// a inside each safe interval of the neighbour is used, after checking that the
// segment between the two cell centers stays safe at every step from d to a (the
// robot moves between cells too); if not, the departure is delayed step by step.
// The heuristic (straight-line distance / speed, no turns) is admissible, so the
// first arrival at the goal is the earliest.
//
// Without obstacles this is turn-aware A*: the path that is fastest for the
// differential drive, turns included, rather than the shortest.
#pragma once

#include <cstddef>
#include <vector>

#include "grid_map.hpp"
#include "types.hpp"

namespace amr {

/// Predicted motion of one obstacle: center at times 0, dt, 2 dt, ... (relative to
/// the start of the plan), with a radius and an extra inflation per sample.
struct ObstaclePrediction {
    std::vector<WorldPoint> centers;
    std::vector<double> inflation;  ///< same length as centers (or empty: 0)
    double radius = 0.0;
};

struct SpaceTimeConfig {
    double dt = 0.1;
    double horizon = 20.0;        ///< s of predictions considered
    double clearance = 0.0;       ///< robot radius + margin, m
    double speed = 0.5;           ///< m/s
    Connectivity connectivity = Connectivity::Four;
    bool turn_aware = false;      ///< differential drive: heading changes cost time
    double max_angular_speed = 1.5;
    double turn_overhead = 0.3;   ///< s per turn (the P controller settling)
    std::size_t max_expansions = 200000;
    double start_time = 0.0;      ///< s from now until the robot is at the start cell
};

/// One step of a timed plan: be at `cell` from `arrive` and leave at `depart`
/// (seconds after the plan's start, i.e. now; the start cell is reached at
/// config.start_time).
struct TimedWaypoint {
    GridCoord cell;
    double arrive = 0.0;
    double depart = 0.0;
};

struct SpaceTimePlan {
    bool success = false;
    std::vector<TimedWaypoint> waypoints;  ///< start first; the goal's depart = its arrive
    double arrival = 0.0;                  ///< s at the goal
    std::size_t expanded = 0;
    double planning_time_ms = 0.0;
};

/// Plan from `start` (heading `start_heading`, rad) to `goal` on the planning map
/// (already inflated for the robot's footprint) around the predicted obstacles.
SpaceTimePlan planSpaceTime(const GridMap& planning_map, GridCoord start, double start_heading,
                            GridCoord goal, const std::vector<ObstaclePrediction>& predictions,
                            const SpaceTimeConfig& config);

/// The cells of a timed plan, in order.
std::vector<GridCoord> planCells(const SpaceTimePlan& plan);

}  // namespace amr
