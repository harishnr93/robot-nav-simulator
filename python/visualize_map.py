#!/usr/bin/env python3
"""Static visualization of the map, the planned A* path and the robot trajectory.

Reads (all written by build/robot_simulator):
    --map FILE                           occupancy grid CSV (0 free, 1 occupied)
    RESULTS_DIR/planning_metrics.csv
    RESULTS_DIR/path.csv
    RESULTS_DIR/trajectory.csv           optional: skipped if absent or empty
    RESULTS_DIR/simulation_metrics.csv   optional: needed with trajectory.csv

Writes:
    OUTPUT_DIR/map_visualization.png     map with start and goal
    OUTPUT_DIR/planned_path.png          map with the planned path, or a
                                         "No valid path" message on failure
    OUTPUT_DIR/trajectory.png            planned path vs. actual robot trajectory
    OUTPUT_DIR/robot_position_vs_time.png  robot x(t) and y(t)
    OUTPUT_DIR/lidar_scan.png            LiDAR scan with the closest return (LiDAR on)
    OUTPUT_DIR/localization.png          true vs. dead-reckoning vs. EKF trajectory (--ekf)
    OUTPUT_DIR/localization_errors.png   estimation errors with the EKF's 3-sigma bounds
    OUTPUT_DIR/replanning.png            moving obstacles, replans and clearance over time
                                         (--dynamic-obstacles)

Plots use grid coordinates (x = column, y = row, origin upper-left, y down),
with a secondary axis in meters: world = (grid + 0.5) * resolution.

Example:
    ./build/robot_simulator --map maps/obstacles.csv --start 1 1 --goal 18 18
    python3 python/visualize_map.py --map maps/obstacles.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Ellipse, Patch

FREE_COLOR = "#f2f2f2"
OCCUPIED_COLOR = "#3c3c3c"
START_COLOR = "#2ca02c"
GOAL_COLOR = "#d62728"
PATH_COLOR = "#1f77b4"
ROBOT_COLOR = "#ff7f0e"
INFLATED_COLOR = "#e7c9a9"   # free cells where the robot footprint does not fit
TRAJECTORY_COLOR = "#7b2d9e"
GRID_LINE_COLOR = "#c8c8c8"
MAX_LABELED_TICKS = 25
FIGURE_DPI = 120
FIGURE_SIZE = (7.5, 8.5)  # inches

METRICS_COLUMNS = (
    "status", "success", "map_width", "map_height", "resolution",
    "start_x", "start_y", "goal_x", "goal_y", "planning_time_ms",
    "expanded_nodes", "waypoint_count", "path_length_steps", "path_length_m",
)
PATH_COLUMNS = ("waypoint_index", "grid_x", "grid_y", "world_x", "world_y")
TRAJECTORY_COLUMNS = ("time", "x", "y", "target_waypoint_index")
# Column indices of the array returned by load_trajectory().
TRAJ_TIME, TRAJ_X, TRAJ_Y, TRAJ_TARGET = range(4)
# Extra columns of a differential-drive trajectory (see has_heading()).
HEADING_COLUMNS = ("theta", "linear_velocity", "angular_velocity")
TRAJ_THETA, TRAJ_V, TRAJ_OMEGA = range(4, 7)
HEADING_COLOR = "#1a1a1a"   # heading arrows / line (neutral ink, not a series color)
LIDAR_RAY_COLOR = "#2a9d8f"   # LiDAR beams
LIDAR_HIT_COLOR = "#14605a"   # LiDAR returns (obstacle points)
LIDAR_COLUMNS = ("time", "beam", "angle", "range", "hit")
EKF_COLOR = "#2a9d8f"             # EKF estimate (LiDAR-corrected: same hue as the LiDAR)
DEAD_RECKONING_COLOR = "#8a8a8a"  # odometry only
LOCALIZATION_COLUMNS = (
    "time", "true_x", "true_y", "true_theta", "odom_x", "odom_y", "odom_theta",
    "est_x", "est_y", "est_theta", "var_x", "cov_xy", "var_y", "cov_x_theta", "cov_y_theta",
    "var_theta", "beams_used", "beams_rejected", "nees")
ELLIPSE_SIGMA = 3.0
OBSTACLE_COLOR = "#9c6644"        # moving obstacles
CURRENT_PLAN_COLOR = PATH_COLOR   # the plan the robot follows now (dashed)
DYNAMIC_COLUMNS = ("time", "obstacle", "x", "y", "radius", "visible")
REPLAN_COLUMNS = ("replan", "time", "from_x", "from_y", "status", "waypoints",
                  "path_cost_cells", "expanded_nodes", "planning_time_ms", "blocked_cells")
REPLANNED_PATH_COLUMNS = ("replan", "waypoint_index", "grid_x", "grid_y")
SIMULATION_COLUMNS = (
    "status", "success", "navigation_success", "steps", "travel_time_s",
    "distance_traveled_m", "final_goal_error_m", "collision_count",
    "speed_mps", "dt_s", "goal_tolerance_m", "max_time_s",
)


class ResultFileError(Exception):
    """Raised when an input file is missing or malformed."""


@dataclass
class PlanningMetrics:
    status: str
    success: bool
    resolution: float
    start: tuple[int, int]
    goal: tuple[int, int]
    map_size: tuple[int, int]
    planning_time_ms: float
    expanded_nodes: int
    path_length_steps: int | None
    path_length_m: float | None
    robot_radius: float = 0.0       # m; 0 = point robot (older results have no column)
    connectivity: int = 4           # 4 or 8 (older results have no column)
    diagonal_steps: int = 0


def load_map(path: Path) -> np.ndarray:
    """Load an occupancy grid with the same rules as the C++ loader."""
    if not path.is_file():
        raise ResultFileError(f"map file '{path}' does not exist")
    rows = []
    for line_number, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        tokens = [token.strip() for token in line.split(",")]
        if any(token not in ("0", "1") for token in tokens):
            raise ResultFileError(
                f"map file '{path}' line {line_number}: cells must be 0 or 1")
        if rows and len(tokens) != len(rows[0]):
            raise ResultFileError(
                f"map file '{path}' line {line_number}: grid is not rectangular")
        rows.append([int(token) for token in tokens])
    if not rows:
        raise ResultFileError(f"map file '{path}' contains no grid rows")
    return np.array(rows, dtype=np.uint8)


def _read_csv(path: Path, required_columns: tuple[str, ...]) -> list[dict[str, str]]:
    if not path.is_file():
        raise ResultFileError(
            f"'{path}' not found; run build/robot_simulator first")
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = set(required_columns) - set(reader.fieldnames or ())
        if missing:
            raise ResultFileError(
                f"'{path}' is missing columns: {', '.join(sorted(missing))}")
        return list(reader)


def load_metrics(path: Path) -> PlanningMetrics:
    rows = _read_csv(path, METRICS_COLUMNS)
    if len(rows) != 1:
        raise ResultFileError(f"'{path}' must contain exactly one data row")
    row = rows[0]
    try:
        success = row["success"] == "1"
        return PlanningMetrics(
            status=row["status"],
            success=success,
            resolution=float(row["resolution"]),
            start=(int(row["start_x"]), int(row["start_y"])),
            goal=(int(row["goal_x"]), int(row["goal_y"])),
            map_size=(int(row["map_width"]), int(row["map_height"])),
            planning_time_ms=float(row["planning_time_ms"]),
            expanded_nodes=int(row["expanded_nodes"]),
            path_length_steps=int(row["path_length_steps"]) if success else None,
            path_length_m=float(row["path_length_m"]) if success else None,
            robot_radius=float(row.get("robot_radius_m") or 0.0),
            connectivity=int(row.get("connectivity") or 4),
            diagonal_steps=int(row.get("diagonal_steps") or 0),
        )
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error


def load_inflated_map(results_dir: Path, grid: np.ndarray,
                      metrics: PlanningMetrics) -> np.ndarray | None:
    """The planning map written by the simulator for a robot footprint, or None.

    The inflation itself is computed only in C++; Python just displays it.
    """
    if metrics.robot_radius <= 0.0:
        return None
    path = results_dir / "inflated_map.csv"
    inflated = load_map(path)
    if inflated.shape != grid.shape or np.any(grid > inflated):
        raise ResultFileError(f"'{path}' does not match the map (re-run the simulator)")
    return inflated


def load_path(path: Path) -> np.ndarray:
    """Return an (N, 2) array of grid (x, y) waypoints; N may be 0."""
    rows = _read_csv(path, PATH_COLUMNS)
    try:
        points = [(int(row["grid_x"]), int(row["grid_y"])) for row in rows]
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error
    return np.array(points, dtype=int).reshape(-1, 2)


@dataclass
class SimulationMetrics:
    status: str
    navigation_success: bool
    steps: int
    travel_time_s: float | None
    distance_traveled_m: float | None
    final_goal_error_m: float | None
    collision_count: int
    speed_mps: float
    dt_s: float
    robot_model: str = "point"           # "point" or "diff-drive" (older results: point)
    turns: int | None = None             # differential drive only
    turn_time_s: float | None = None


def _optional_float(text: str) -> float | None:
    return float(text) if text != "" else None


def load_simulation_metrics(path: Path) -> SimulationMetrics:
    rows = _read_csv(path, SIMULATION_COLUMNS)
    if len(rows) != 1:
        raise ResultFileError(f"'{path}' must contain exactly one data row")
    row = rows[0]
    try:
        return SimulationMetrics(
            status=row["status"],
            navigation_success=row["navigation_success"] == "1",
            steps=int(row["steps"]),
            travel_time_s=_optional_float(row["travel_time_s"]),
            distance_traveled_m=_optional_float(row["distance_traveled_m"]),
            final_goal_error_m=_optional_float(row["final_goal_error_m"]),
            collision_count=int(row["collision_count"]),
            speed_mps=float(row["speed_mps"]),
            dt_s=float(row["dt_s"]),
            robot_model=row.get("robot_model") or "point",
            turns=int(row["turns"]) if row.get("turns") else None,
            turn_time_s=_optional_float(row.get("turn_time_s") or ""),
        )
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error


def load_trajectory(path: Path) -> np.ndarray:
    """Return an (N, 4) array of (time_s, x_m, y_m, target_waypoint_index), or (N, 7)
    with (theta_rad, v_mps, omega_radps) appended for a differential-drive trajectory.

    N may be 0. Columns are addressed with the TRAJ_* constants.
    """
    rows = _read_csv(path, TRAJECTORY_COLUMNS)
    with_heading = bool(rows) and all(c in rows[0] for c in HEADING_COLUMNS)
    width = 7 if with_heading else 4
    try:
        data = np.array([(float(r["time"]), float(r["x"]), float(r["y"]),
                          int(r["target_waypoint_index"]))
                         + (tuple(float(r[c]) for c in HEADING_COLUMNS) if with_heading else ())
                         for r in rows], dtype=float).reshape(-1, width)
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error
    if not np.isfinite(data).all():
        raise ResultFileError(f"'{path}' contains non-finite values")
    if len(data) > 1 and not (np.diff(data[:, 0]) > 0).all():
        raise ResultFileError(f"'{path}' timestamps are not strictly increasing")
    return data


def has_heading(trajectory: np.ndarray) -> bool:
    """True for a differential-drive trajectory (heading and velocity columns)."""
    return trajectory.ndim == 2 and trajectory.shape[1] > TRAJ_THETA


def heading_arrow_indices(trajectory: np.ndarray, count: int = 14) -> np.ndarray:
    """Sample indices for heading arrows, evenly spaced along the travelled distance."""
    steps = np.hypot(np.diff(trajectory[:, TRAJ_X]), np.diff(trajectory[:, TRAJ_Y]))
    distance = np.concatenate([[0.0], np.cumsum(steps)])
    if distance[-1] == 0.0:
        return np.array([0])
    marks = np.linspace(0.0, distance[-1], count)
    return np.unique(np.searchsorted(distance, marks).clip(0, len(trajectory) - 1))


def world_to_grid(meters: np.ndarray, resolution: float) -> np.ndarray:
    """Inverse of world = (grid + 0.5) * resolution (continuous)."""
    return np.asarray(meters) / resolution - 0.5


def check_results_match_map(grid: np.ndarray, metrics: PlanningMetrics,
                            waypoints: np.ndarray, map_path: Path) -> None:
    """Guard against plotting results that were produced for a different map."""
    height, width = grid.shape
    if metrics.map_size != (width, height):
        raise ResultFileError(
            f"results were produced for a {metrics.map_size[0]} x "
            f"{metrics.map_size[1]} map, but '{map_path}' is {width} x {height}")
    if metrics.success != (len(waypoints) > 0):
        raise ResultFileError("path.csv and planning_metrics.csv disagree; "
                              "re-run build/robot_simulator")
    if len(waypoints) == 0:
        return
    in_bounds = ((waypoints[:, 0] >= 0) & (waypoints[:, 0] < width)
                 & (waypoints[:, 1] >= 0) & (waypoints[:, 1] < height))
    if not in_bounds.all() or grid[waypoints[:, 1], waypoints[:, 0]].any():
        raise ResultFileError(
            f"the planned path crosses obstacles of '{map_path}'; the results "
            f"were probably produced for a different map")


def _tick_step(size: int) -> int:
    return max(1, int(np.ceil(size / MAX_LABELED_TICKS)))


def draw_grid(ax, grid: np.ndarray, resolution: float) -> None:
    height, width = grid.shape
    cmap = ListedColormap([FREE_COLOR, OCCUPIED_COLOR])
    # extent puts cell (x, y) centered on integer coordinates; origin upper-left.
    ax.imshow(grid, cmap=cmap, vmin=0, vmax=1, interpolation="nearest",
              extent=(-0.5, width - 0.5, height - 0.5, -0.5))
    ax.set_xticks(np.arange(-0.5, width, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, height, 1), minor=True)
    ax.grid(which="minor", color=GRID_LINE_COLOR, linewidth=0.5)
    ax.tick_params(which="minor", length=0)
    ax.set_xticks(np.arange(0, width, _tick_step(width)))
    ax.set_yticks(np.arange(0, height, _tick_step(height)))
    ax.set_xlabel("grid x (column)")
    ax.set_ylabel("grid y (row, increases downward)")

    def to_meters(g):
        return (np.asarray(g) + 0.5) * resolution

    def to_grid(m):
        return np.asarray(m) / resolution - 0.5

    ax.secondary_xaxis("top", functions=(to_meters, to_grid)).set_xlabel("x (m)")
    ax.secondary_yaxis("right", functions=(to_meters, to_grid)).set_ylabel("y (m)")


def draw_inflation(ax, grid: np.ndarray, inflated: np.ndarray | None) -> list:
    """Shade free cells that are blocked for the robot center. Returns legend handles."""
    if inflated is None:
        return []
    band = (inflated == 1) & (grid == 0)
    height, width = grid.shape
    ax.imshow(np.ma.masked_where(~band, band), cmap=ListedColormap([INFLATED_COLOR]),
              vmin=0, vmax=1, interpolation="nearest", zorder=0.5,
              extent=(-0.5, width - 0.5, height - 0.5, -0.5))
    return [Patch(facecolor=INFLATED_COLOR, label="Inflated (too close for the robot)")]


def robot_disc(center_grid: tuple[float, float], radius_m: float, resolution: float,
               label: str | None = None) -> Circle:
    """True-size robot footprint in grid coordinates."""
    return Circle(center_grid, radius_m / resolution, facecolor=ROBOT_COLOR,
                  edgecolor="black", linewidth=1.0, alpha=0.9, zorder=3.5, label=label)


def draw_start_goal(ax, start: tuple[int, int], goal: tuple[int, int]) -> None:
    ax.plot(*start, marker="o", markersize=11, color=START_COLOR,
            markeredgecolor="white", markeredgewidth=1.5, linestyle="none",
            label=f"Start {start}", zorder=4)
    ax.plot(*goal, marker="*", markersize=17, color=GOAL_COLOR,
            markeredgecolor="white", markeredgewidth=1.2, linestyle="none",
            label=f"Goal {goal}", zorder=4)


def _legend_handles():
    return [Patch(facecolor=FREE_COLOR, edgecolor=GRID_LINE_COLOR, label="Free cell"),
            Patch(facecolor=OCCUPIED_COLOR, label="Obstacle")]


def _finish(fig, ax, output: Path, extra_handles: list | None = None) -> None:
    handles, _ = ax.get_legend_handles_labels()
    fig.legend(handles=_legend_handles() + (extra_handles or []) + handles,
               loc="outside lower center", ncol=3, frameon=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    print(f"Saved {output}")


def _radius_note(robot_radius: float) -> str:
    return f", robot radius {robot_radius:g} m" if robot_radius > 0 else ""


def plot_map(grid: np.ndarray, resolution: float, start, goal, output: Path,
             inflated: np.ndarray | None = None, robot_radius: float = 0.0):
    fig, ax = plt.subplots(figsize=FIGURE_SIZE, layout="constrained")
    draw_grid(ax, grid, resolution)
    extra = draw_inflation(ax, grid, inflated)
    if start is not None and goal is not None:
        draw_start_goal(ax, start, goal)
    height, width = grid.shape
    fig.suptitle(f"Occupancy grid  ({width} x {height} cells, {resolution:g} m/cell"
                 f"{_radius_note(robot_radius)})")
    _finish(fig, ax, output, extra)
    return fig


def plot_planned_path(grid: np.ndarray, metrics: PlanningMetrics,
                      waypoints: np.ndarray, output: Path,
                      inflated: np.ndarray | None = None):
    fig, ax = plt.subplots(figsize=FIGURE_SIZE, layout="constrained")
    draw_grid(ax, grid, metrics.resolution)
    extra = draw_inflation(ax, grid, inflated)
    if metrics.success and len(waypoints) > 0:
        ax.plot(waypoints[:, 0], waypoints[:, 1], color=PATH_COLOR, linewidth=2.5,
                marker="o", markersize=3.5, label="Planned A* path", zorder=3)
        moves = (f"{metrics.path_length_steps} steps"
                 + (f" ({metrics.diagonal_steps} diagonal)" if metrics.connectivity == 8 else ""))
        kind = "8-connected A*" if metrics.connectivity == 8 else "A*"
        title = (f"{kind} path: {moves}, "
                 f"{metrics.path_length_m:.2f} m, {metrics.expanded_nodes} nodes "
                 f"expanded, {metrics.planning_time_ms:.3f} ms")
    else:
        ax.text(0.5, 0.5, f"No valid path\n({metrics.status})",
                transform=ax.transAxes, ha="center", va="center", fontsize=18,
                fontweight="bold", color="black", zorder=5,
                bbox=dict(boxstyle="round,pad=0.6", facecolor="white",
                          edgecolor=GOAL_COLOR, linewidth=2))
        title = f"Planning failed: {metrics.status}"
    draw_start_goal(ax, metrics.start, metrics.goal)
    fig.suptitle(title + _radius_note(metrics.robot_radius))
    _finish(fig, ax, output, extra)
    return fig


def plot_trajectory(grid: np.ndarray, metrics: PlanningMetrics, waypoints: np.ndarray,
                    trajectory: np.ndarray, sim: SimulationMetrics, output: Path,
                    inflated: np.ndarray | None = None):
    """Planned path (blue) underneath the actual robot trajectory (purple)."""
    fig, ax = plt.subplots(figsize=FIGURE_SIZE, layout="constrained")
    draw_grid(ax, grid, metrics.resolution)
    extra = draw_inflation(ax, grid, inflated)
    if len(waypoints) > 0:
        ax.plot(waypoints[:, 0], waypoints[:, 1], color=PATH_COLOR, linewidth=6,
                alpha=0.35, solid_capstyle="round", label="Planned A* path", zorder=2)
    traj_x = world_to_grid(trajectory[:, TRAJ_X], metrics.resolution)
    traj_y = world_to_grid(trajectory[:, TRAJ_Y], metrics.resolution)
    ax.plot(traj_x, traj_y, color=TRAJECTORY_COLOR, linewidth=1.8,
            label="Robot trajectory", zorder=3)
    draw_start_goal(ax, metrics.start, metrics.goal)
    # Drawn beneath the goal star (zorder 4) so both stay visible at the goal.
    if metrics.robot_radius > 0:
        ax.add_patch(robot_disc((traj_x[-1], traj_y[-1]), metrics.robot_radius,
                                metrics.resolution,
                                f"Robot footprint, final (r = {metrics.robot_radius:g} m)"))
    else:
        ax.plot(traj_x[-1], traj_y[-1], marker="o", markersize=15, color=ROBOT_COLOR,
                markeredgecolor="black", markeredgewidth=1.0, linestyle="none",
                label="Robot (final position)", zorder=3.5)
    title_extra = ""
    if has_heading(trajectory):
        # Heading arrows (unit length in cells) along the trajectory, plus the final heading.
        idx = heading_arrow_indices(trajectory)
        theta = trajectory[idx, TRAJ_THETA]
        ax.quiver(traj_x[idx], traj_y[idx], np.cos(theta), np.sin(theta), angles="xy",
                  scale_units="xy", scale=1.25, width=0.004, color=HEADING_COLOR,
                  zorder=4.5, label="Heading (differential drive)")
        title_extra = (f", {sim.turns} turns ({sim.turn_time_s:.1f} s)"
                       if sim.turns is not None and sim.turn_time_s is not None else "")
    time_s = trajectory[-1, TRAJ_TIME] - trajectory[0, TRAJ_TIME]
    distance = sim.distance_traveled_m or 0.0
    fig.suptitle(f"Navigation: {sim.status}  |  {time_s:.1f} s, {distance:.2f} m "
                 f"traveled, {sim.collision_count} collision(s){title_extra}")
    _finish(fig, ax, output, extra)
    return fig


def plot_position_vs_time(trajectory: np.ndarray, sim: SimulationMetrics, output: Path):
    """x(t) and y(t) in meters, one panel each (same units, separate axes); for a
    differential-drive trajectory a third panel shows the heading theta(t)."""
    if has_heading(trajectory):
        fig, (ax_x, ax_y, ax_theta) = plt.subplots(3, 1, figsize=(8, 7.5), sharex=True,
                                                   layout="constrained")
        ax_theta.plot(trajectory[:, TRAJ_TIME],
                      np.degrees(np.unwrap(trajectory[:, TRAJ_THETA])),
                      color=TRAJECTORY_COLOR, linewidth=2)
        ax_theta.set_ylabel("heading θ (deg, unwrapped)")
        ax_theta.grid(color=GRID_LINE_COLOR, linewidth=0.5)
        ax_theta.spines[["top", "right"]].set_visible(False)
    else:
        fig, (ax_x, ax_y) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True,
                                         layout="constrained")
        ax_theta = None
    time_s = trajectory[:, TRAJ_TIME]
    for ax, values, name in ((ax_x, trajectory[:, TRAJ_X], "x"),
                             (ax_y, trajectory[:, TRAJ_Y], "y")):
        ax.plot(time_s, values, color=TRAJECTORY_COLOR, linewidth=2)
        ax.set_ylabel(f"{name} (m)")
        ax.grid(color=GRID_LINE_COLOR, linewidth=0.5)
        ax.spines[["top", "right"]].set_visible(False)
    (ax_theta or ax_y).set_xlabel("simulation time (s)")
    ax_y.invert_yaxis()  # world +y points down, matching the map figures
    ax_y.set_ylabel("y (m, increases downward)")
    fig.suptitle(f"Robot position vs. time  ({sim.status}, speed {sim.speed_mps:g} m/s, "
                 f"dt {sim.dt_s:g} s)")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    print(f"Saved {output}")
    return fig


@dataclass
class LidarScans:
    """All scans of a run: one row per scan, one column per beam."""
    times: np.ndarray     # (S,)
    angles: np.ndarray    # (S, B), world frame, rad
    ranges: np.ndarray    # (S, B), m
    hits: np.ndarray      # (S, B), bool: False = no return within the maximum range


def load_lidar(path: Path) -> LidarScans | None:
    """lidar_scans.csv written by the simulator, or None if there is none."""
    if not path.is_file():
        return None
    rows = _read_csv(path, LIDAR_COLUMNS)
    if not rows:
        return None
    try:
        data = np.array([(float(r["time"]), int(r["beam"]), float(r["angle"]),
                          float(r["range"]), int(r["hit"])) for r in rows], dtype=float)
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error
    beams = int(data[:, 1].max()) + 1
    if len(data) % beams != 0 or not np.array_equal(data[:, 1],
                                                     np.tile(np.arange(beams), len(data) // beams)):
        raise ResultFileError(f"'{path}': every scan must list beams 0..{beams - 1} in order")
    shape = (len(data) // beams, beams)
    times = data[:, 0].reshape(shape)
    if not np.all(times == times[:, :1]):
        raise ResultFileError(f"'{path}': beams of one scan have different times")
    return LidarScans(times[:, 0], data[:, 2].reshape(shape), data[:, 3].reshape(shape),
                      data[:, 4].reshape(shape).astype(bool))


def check_lidar_matches_trajectory(scans: LidarScans, trajectory: np.ndarray) -> None:
    if len(scans.times) != len(trajectory) or not np.allclose(scans.times,
                                                               trajectory[:, TRAJ_TIME]):
        raise ResultFileError("lidar_scans.csv does not match trajectory.csv "
                              "(one scan per trajectory sample); re-run the simulator")


def lidar_segments(scans: LidarScans, trajectory: np.ndarray, index: int,
                   resolution: float) -> tuple[np.ndarray, np.ndarray]:
    """Rays of scan `index` as grid-coordinate segments, and the return points."""
    ox, oy = trajectory[index, TRAJ_X], trajectory[index, TRAJ_Y]
    ends_x = ox + scans.ranges[index] * np.cos(scans.angles[index])
    ends_y = oy + scans.ranges[index] * np.sin(scans.angles[index])
    starts = np.column_stack([np.full_like(ends_x, ox), np.full_like(ends_y, oy)])
    ends = np.column_stack([ends_x, ends_y])
    segments = world_to_grid(np.stack([starts, ends], axis=1), resolution)
    hits = world_to_grid(ends[scans.hits[index]], resolution)
    return segments, hits


def plot_lidar_scan(grid: np.ndarray, metrics: PlanningMetrics, trajectory: np.ndarray,
                    scans: LidarScans, output: Path, inflated: np.ndarray | None = None):
    """The scan with the closest return: rays on the map, and range vs. beam angle."""
    masked = np.where(scans.hits, scans.ranges, np.inf)
    index = int(np.argmin(masked.min(axis=1)))
    fig = plt.figure(figsize=(13, 6.6), layout="constrained")
    ax = fig.add_subplot(1, 2, 1)
    ax_polar = fig.add_subplot(1, 2, 2, projection="polar")
    draw_grid(ax, grid, metrics.resolution)
    extra = draw_inflation(ax, grid, inflated)
    segments, hits = lidar_segments(scans, trajectory, index, metrics.resolution)
    ax.add_collection(LineCollection(segments, colors=LIDAR_RAY_COLOR, linewidths=0.8,
                                     alpha=0.7, zorder=2.5, label="LiDAR rays"))
    ax.plot(hits[:, 0], hits[:, 1], linestyle="none", marker="o", markersize=3.5,
            color=LIDAR_HIT_COLOR, zorder=2.6, label="LiDAR returns")
    traj_x = world_to_grid(trajectory[:, TRAJ_X], metrics.resolution)
    traj_y = world_to_grid(trajectory[:, TRAJ_Y], metrics.resolution)
    ax.plot(traj_x[: index + 1], traj_y[: index + 1], color=TRAJECTORY_COLOR, linewidth=1.8,
            label="Robot trajectory", zorder=3)
    draw_start_goal(ax, metrics.start, metrics.goal)
    ax.plot(traj_x[index], traj_y[index], marker="o", markersize=10, color=ROBOT_COLOR,
            markeredgecolor="black", linestyle="none", label="Robot", zorder=3.5)
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(handles=_legend_handles() + extra + handles, loc="upper center",
              bbox_to_anchor=(0.5, -0.09), ncol=3, frameon=False, fontsize=9)
    # Polar view in the image convention (+y down) so it matches the map: angles
    # grow clockwise from +x.
    ax_polar.set_theta_direction(-1)
    ax_polar.plot(np.append(scans.angles[index], scans.angles[index][0]),
                  np.append(scans.ranges[index], scans.ranges[index][0]),
                  color=LIDAR_RAY_COLOR, linewidth=1.2)
    ax_polar.plot(scans.angles[index][scans.hits[index]], scans.ranges[index][scans.hits[index]],
                  linestyle="none", marker="o", markersize=3.5, color=LIDAR_HIT_COLOR)
    ax_polar.set_title("range (m) by beam direction (0° = +x, 90° = down the map)",
                       fontsize=10, pad=18)
    closest = masked[index].min()
    fig.suptitle(f"LiDAR scan at t = {scans.times[index]:.1f} s: closest return "
                 f"{closest:.3f} m  ({scans.ranges.shape[1]} beams, "
                 f"{100 * scans.hits.mean():.0f} % of all beams returned)")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    print(f"Saved {output}")
    return fig


@dataclass
class Localization:
    """localization.csv: poses are (N, 3) arrays of (x_m, y_m, theta_rad)."""
    times: np.ndarray
    truth: np.ndarray
    odometry: np.ndarray
    estimate: np.ndarray
    covariance: np.ndarray   # (N, 3, 3), m^2 / m rad / rad^2
    beams_used: np.ndarray
    beams_rejected: np.ndarray
    nees: np.ndarray

    def errors(self, poses: np.ndarray) -> np.ndarray:
        """truth - poses as (N, 3), heading error wrapped to (-pi, pi]."""
        error = self.truth - poses
        error[:, 2] = np.angle(np.exp(1j * error[:, 2]))
        return error


def load_localization(path: Path) -> Localization | None:
    """localization.csv written with --ekf, or None if there is none."""
    if not path.is_file():
        return None
    rows = _read_csv(path, LOCALIZATION_COLUMNS)
    if not rows:
        return None
    try:
        data = np.array([[float(r[c]) for c in LOCALIZATION_COLUMNS] for r in rows], dtype=float)
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error
    if not np.isfinite(data).all():
        raise ResultFileError(f"'{path}' contains non-finite values")
    column = {name: data[:, i] for i, name in enumerate(LOCALIZATION_COLUMNS)}
    xx, xy, yy = column["var_x"], column["cov_xy"], column["var_y"]
    xt, yt, tt = column["cov_x_theta"], column["cov_y_theta"], column["var_theta"]
    covariance = np.stack([np.stack([xx, xy, xt], -1), np.stack([xy, yy, yt], -1),
                           np.stack([xt, yt, tt], -1)], -2)
    def poses(prefix):
        return np.column_stack([column[f"{prefix}_x"], column[f"{prefix}_y"],
                                column[f"{prefix}_theta"]])
    return Localization(column["time"], poses("true"), poses("odom"), poses("est"), covariance,
                        column["beams_used"].astype(int), column["beams_rejected"].astype(int),
                        column["nees"])


def check_localization_matches_trajectory(loc: Localization, trajectory: np.ndarray) -> None:
    if (len(loc.times) != len(trajectory)
            or not np.allclose(loc.times, trajectory[:, TRAJ_TIME])
            or not np.allclose(loc.truth[:, :2], trajectory[:, [TRAJ_X, TRAJ_Y]], atol=1e-5)):
        raise ResultFileError("localization.csv does not match trajectory.csv "
                              "(one sample per trajectory sample); re-run the simulator")


def covariance_ellipse(center_grid: tuple[float, float], covariance_xy: np.ndarray,
                       resolution: float, sigma: float = ELLIPSE_SIGMA, **kwargs) -> Ellipse:
    """The sigma-ellipse of a 2 x 2 position covariance (m^2), in grid units."""
    values, vectors = np.linalg.eigh(covariance_xy)
    values = np.clip(values, 0.0, None)
    angle = np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1]))  # major axis
    width = 2 * sigma * np.sqrt(values[1]) / resolution
    height = 2 * sigma * np.sqrt(values[0]) / resolution
    return Ellipse(center_grid, width, height, angle=angle, **kwargs)


def _position_errors(loc: Localization) -> tuple[np.ndarray, np.ndarray]:
    est = loc.errors(loc.estimate)
    odom = loc.errors(loc.odometry)
    return np.hypot(est[:, 0], est[:, 1]), np.hypot(odom[:, 0], odom[:, 1])


def plot_localization(grid: np.ndarray, metrics: PlanningMetrics, loc: Localization,
                      output: Path, inflated: np.ndarray | None = None):
    """True trajectory vs. dead reckoning vs. the EKF estimate on the map, with
    3-sigma position ellipses of the EKF along the way."""
    # Taller than the other map figures: room for a three-line title.
    fig, ax = plt.subplots(figsize=(FIGURE_SIZE[0], FIGURE_SIZE[1] + 0.6), layout="constrained")
    draw_grid(ax, grid, metrics.resolution)
    extra = draw_inflation(ax, grid, inflated)
    res = metrics.resolution
    def grid_xy(poses):
        return world_to_grid(poses[:, 0], res), world_to_grid(poses[:, 1], res)
    # The truth as a wide translucent band: the EKF estimate usually lies on top of it.
    ax.plot(*grid_xy(loc.truth), color=TRAJECTORY_COLOR, linewidth=5, alpha=0.45,
            solid_capstyle="round", label="True trajectory", zorder=3)
    ax.plot(*grid_xy(loc.odometry), color=DEAD_RECKONING_COLOR, linewidth=1.8,
            linestyle="--", label="Dead reckoning (odometry only)", zorder=3.25)
    ax.plot(*grid_xy(loc.estimate), color=EKF_COLOR, linewidth=1.6,
            label="EKF estimate", zorder=3.2)
    odom_x, odom_y = grid_xy(loc.odometry)
    ax.plot(odom_x[-1], odom_y[-1], marker="X", markersize=11, color=DEAD_RECKONING_COLOR,
            markeredgecolor="black", linestyle="none", zorder=3.6,
            label="Dead reckoning, final")
    draw_start_goal(ax, metrics.start, metrics.goal)
    # Ellipses are clipped to the view of the map and trajectories, not fitted.
    limits = ax.get_xlim(), ax.get_ylim()
    largest = ELLIPSE_SIGMA * np.sqrt(np.linalg.eigvalsh(loc.covariance[:, :2, :2])[:, -1].clip(0))
    if largest.max() / res >= 0.15:  # otherwise too small to see at this scale
        est_x, est_y = grid_xy(loc.estimate)
        for i in np.unique(np.linspace(0, len(loc.times) - 1, 12).astype(int)):
            # Not part of the layout: clipped ellipses must not push the labels away.
            ax.add_patch(covariance_ellipse((est_x[i], est_y[i]), loc.covariance[i, :2, :2],
                                            res, facecolor="none", edgecolor=EKF_COLOR,
                                            linewidth=1.0, zorder=3.3, in_layout=False))
        extra.append(Patch(facecolor="none", edgecolor=EKF_COLOR,
                           label=f"EKF {ELLIPSE_SIGMA:g}σ position uncertainty"))
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    # Dead reckoning can leave the map, making the view taller than wide: grow the
    # figure with it so the title and legend keep their room.
    aspect = abs(np.diff(limits[1])[0]) / abs(np.diff(limits[0])[0])
    if aspect > 1:
        fig.set_figheight(fig.get_figheight() + 0.8 * FIGURE_SIZE[0] * (aspect - 1))
    est_err, odom_err = _position_errors(loc)
    used = int(loc.beams_used.sum())
    correction = (f"{used} LiDAR beams fused" if used else "no LiDAR: prediction only")
    fig.suptitle(f"EKF localization ({correction})\nfinal position error: EKF "
                 f"{est_err[-1]:.3f} m, dead reckoning {odom_err[-1]:.3f} m\n"
                 f"largest EKF {ELLIPSE_SIGMA:g}σ position uncertainty: {largest.max():.3f} m")
    _finish(fig, ax, output, extra)
    return fig


def plot_localization_errors(loc: Localization, output: Path):
    """Position error of both estimates (log scale), and the EKF's x, y and heading
    errors inside its 3-sigma bounds."""
    fig, axes = plt.subplots(4, 1, figsize=(8.5, 10), sharex=True, layout="constrained")
    t = loc.times
    est_err, odom_err = _position_errors(loc)
    floor = 1e-5  # log axis: exact zeros (e.g. at t = 0) are shown at the floor
    axes[0].semilogy(t, np.maximum(est_err, floor), color=EKF_COLOR, linewidth=1.8,
                     label=f"EKF (RMS {np.sqrt(np.mean(est_err**2)):.4f} m)")
    # Dashed on top: without LiDAR updates the two curves coincide.
    axes[0].semilogy(t, np.maximum(odom_err, floor), color=DEAD_RECKONING_COLOR, linewidth=1.8,
                     linestyle="--",
                     label=f"dead reckoning (RMS {np.sqrt(np.mean(odom_err**2)):.3f} m)")
    axes[0].set_ylabel("position error (m)")
    axes[0].legend(frameon=False, fontsize=9, loc="best")
    errors = loc.errors(loc.estimate)
    sigma = np.sqrt(np.clip(np.diagonal(loc.covariance, axis1=1, axis2=2), 0.0, None))
    labels = ("x error (m)", "y error (m)", "heading error (deg)")
    for k, (ax, label) in enumerate(zip(axes[1:], labels, strict=True)):
        scale = np.degrees(1.0) if k == 2 else 1.0
        bound = ELLIPSE_SIGMA * sigma[:, k] * scale
        ax.fill_between(t, -bound, bound, color=EKF_COLOR, alpha=0.18, linewidth=0,
                        label=f"±{ELLIPSE_SIGMA:g}σ (EKF covariance)")
        ax.plot(t, errors[:, k] * scale, color=EKF_COLOR, linewidth=1.4, label="EKF error")
        ax.axhline(0.0, color=GRID_LINE_COLOR, linewidth=0.8)
        ax.set_ylabel(label)
        inside = np.mean(np.abs(errors[:, k] * scale) <= bound)
        ax.text(0.99, 0.95, f"{100 * inside:.1f} % of samples inside ±{ELLIPSE_SIGMA:g}σ",
                transform=ax.transAxes, ha="right", va="top", fontsize=9)
    axes[1].legend(frameon=False, fontsize=9, loc="lower left")
    for ax in axes:
        ax.grid(color=GRID_LINE_COLOR, linewidth=0.5)
        ax.spines[["top", "right"]].set_visible(False)
    axes[-1].set_xlabel("simulation time (s)")
    fig.suptitle(f"EKF estimation errors (true − estimate); mean NEES {loc.nees.mean():.2f} "
                 f"(3 = consistent)")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    print(f"Saved {output}")
    return fig


@dataclass
class DynamicObstacles:
    """dynamic_obstacles.csv: positions (m) at every trajectory sample."""
    times: np.ndarray     # (T,)
    x: np.ndarray         # (T, K)
    y: np.ndarray         # (T, K)
    radius: np.ndarray    # (K,)
    visible: np.ndarray   # (T, K) bool: seen by the robot at that sample


@dataclass
class Replans:
    """replans.csv and replanned_paths.csv."""
    times: np.ndarray                 # (N,)
    from_cells: np.ndarray            # (N, 2) grid cells the new paths start from
    success: np.ndarray               # (N,) bool
    paths: list[np.ndarray | None]    # (M, 2) grid cells per attempt, None if no path

    def current_path(self, initial: np.ndarray, t: float) -> np.ndarray:
        """The path being followed at time t (the latest successful replan's path
        from where it was planned; the initial path before any)."""
        done = np.flatnonzero(self.success & (self.times <= t + 1e-9))
        return initial if len(done) == 0 else self.paths[done[-1]]


def load_dynamic(results_dir: Path) -> tuple[DynamicObstacles, Replans] | None:
    """The dynamic-obstacle outputs, or None if the run had no moving obstacles."""
    path = results_dir / "dynamic_obstacles.csv"
    if not path.is_file():
        return None
    rows = _read_csv(path, DYNAMIC_COLUMNS)
    try:
        data = np.array([[float(r[c]) for c in DYNAMIC_COLUMNS] for r in rows],
                        dtype=float).reshape(-1, len(DYNAMIC_COLUMNS))
        events = _read_csv(results_dir / "replans.csv", REPLAN_COLUMNS)
        path_rows = _read_csv(results_dir / "replanned_paths.csv", REPLANNED_PATH_COLUMNS)
        cells = np.array([[int(r["replan"]), int(r["grid_x"]), int(r["grid_y"])]
                          for r in path_rows], dtype=int).reshape(-1, 3)
        times = np.array([float(e["time"]) for e in events])
        from_cells = np.array([[int(e["from_x"]), int(e["from_y"])] for e in events],
                              dtype=int).reshape(-1, 2)
    except (ValueError, KeyError) as error:
        raise ResultFileError(f"'{results_dir}': malformed dynamic-obstacle output: {error}") \
            from error
    count = int(data[:, 1].max()) + 1 if len(data) else 0
    if count == 0 or len(data) % count or not np.array_equal(
            data[:, 1], np.tile(np.arange(count), len(data) // count)):
        raise ResultFileError(f"'{path}': every sample must list obstacles 0..{count - 1}")
    shape = (len(data) // count, count)
    obstacles = DynamicObstacles(data[::count, 0], data[:, 2].reshape(shape),
                                 data[:, 3].reshape(shape), data[:count, 4],
                                 data[:, 5].reshape(shape).astype(bool))
    success = np.array([e["status"] == "SUCCESS" for e in events], dtype=bool)
    paths = [cells[cells[:, 0] == i, 1:] if ok else None for i, ok in enumerate(success)]
    return obstacles, Replans(times, from_cells, success, paths)


def check_dynamic_matches_trajectory(obstacles: DynamicObstacles, trajectory: np.ndarray) -> None:
    if len(obstacles.times) != len(trajectory) or not np.allclose(obstacles.times,
                                                                   trajectory[:, TRAJ_TIME]):
        raise ResultFileError("dynamic_obstacles.csv does not match trajectory.csv "
                              "(one row per obstacle and trajectory sample); re-run the simulator")


def obstacle_clearance(obstacles: DynamicObstacles, trajectory: np.ndarray,
                       robot_radius: float) -> np.ndarray:
    """Gap between the robot and the nearest obstacle edge at every sample, m."""
    dx = obstacles.x - trajectory[:, [TRAJ_X]]
    dy = obstacles.y - trajectory[:, [TRAJ_Y]]
    return (np.hypot(dx, dy) - obstacles.radius - robot_radius).min(axis=1)


def waiting_mask(trajectory: np.ndarray) -> np.ndarray:
    """True for samples that ended a step in which the robot neither moved nor turned."""
    still = np.zeros(len(trajectory), dtype=bool)
    moved = np.hypot(np.diff(trajectory[:, TRAJ_X]), np.diff(trajectory[:, TRAJ_Y])) > 1e-12
    turned = (np.abs(np.diff(trajectory[:, TRAJ_THETA])) > 1e-12 if has_heading(trajectory)
              else np.zeros(len(trajectory) - 1, dtype=bool))
    still[1:] = ~moved & ~turned
    return still


def plot_replanning(grid: np.ndarray, metrics: PlanningMetrics, waypoints: np.ndarray,
                    trajectory: np.ndarray, obstacles: DynamicObstacles, replans: Replans,
                    sim: SimulationMetrics, output: Path, inflated: np.ndarray | None = None):
    """Map: obstacle routes, initial plan, replan points and the executed
    trajectory. Timeline: clearance to the nearest obstacle, waiting and replans."""
    fig = plt.figure(figsize=(13, 8.4), layout="constrained")
    grid_spec = fig.add_gridspec(1, 2, width_ratios=(1.0, 1.05))
    ax = fig.add_subplot(grid_spec[0])
    ax_t = fig.add_subplot(grid_spec[1])
    res = metrics.resolution
    draw_grid(ax, grid, res)
    extra = draw_inflation(ax, grid, inflated)
    ax.plot(waypoints[:, 0], waypoints[:, 1], color=PATH_COLOR, linewidth=6, alpha=0.3,
            solid_capstyle="round", label="Initial plan", zorder=2)
    for k in range(obstacles.x.shape[1]):
        ox, oy = world_to_grid(obstacles.x[:, k], res), world_to_grid(obstacles.y[:, k], res)
        ax.plot(ox, oy, color=OBSTACLE_COLOR, linewidth=1.4, linestyle=":", zorder=2.2,
                label="Moving obstacle routes" if k == 0 else None)
        ax.add_patch(Circle((ox[0], oy[0]), obstacles.radius[k] / res, facecolor=OBSTACLE_COLOR,
                            alpha=0.55, edgecolor="none", zorder=2.3,
                            label="Moving obstacles at t = 0" if k == 0 else None))
    traj_x = world_to_grid(trajectory[:, TRAJ_X], res)
    traj_y = world_to_grid(trajectory[:, TRAJ_Y], res)
    ax.plot(traj_x, traj_y, color=TRAJECTORY_COLOR, linewidth=2, label="Executed trajectory",
            zorder=3)
    ok = replans.success
    if ok.any():
        ax.plot(replans.from_cells[ok, 0], replans.from_cells[ok, 1], linestyle="none",
                marker="D", markersize=7, color=CURRENT_PLAN_COLOR, markeredgecolor="white",
                label="Replanned here", zorder=3.5)
    if (~ok).any():
        ax.plot(replans.from_cells[~ok, 0], replans.from_cells[~ok, 1], linestyle="none",
                marker="x", markersize=8, color=GOAL_COLOR, label="No path (waited)", zorder=3.5)
    draw_start_goal(ax, metrics.start, metrics.goal)
    handles, _ = ax.get_legend_handles_labels()
    fig.legend(handles=_legend_handles() + extra + handles, loc="outside lower center",
               ncol=5, frameon=False, fontsize=9)

    t = trajectory[:, TRAJ_TIME]
    clearance = obstacle_clearance(obstacles, trajectory, metrics.robot_radius)
    waiting = waiting_mask(trajectory)
    ax_t.fill_between(t, 0, 1, where=waiting, transform=ax_t.get_xaxis_transform(),
                      color=GRID_LINE_COLOR, alpha=0.6, linewidth=0, label="Waiting", step="pre")
    ax_t.plot(t, clearance, color=TRAJECTORY_COLOR, linewidth=1.8,
              label="Gap to the nearest obstacle edge")
    ax_t.axhline(0.0, color=GOAL_COLOR, linewidth=1.2, label="Contact")
    for i, time in enumerate(replans.times[ok]):
        ax_t.axvline(time, color=CURRENT_PLAN_COLOR, linewidth=1.0, linestyle="--",
                     label="Replan" if i == 0 else None)
    ax_t.set_xlabel("simulation time (s)")
    ax_t.set_ylabel("gap (m)")
    ax_t.set_ylim(bottom=min(-0.1, float(clearance.min()) - 0.1))
    ax_t.grid(color=GRID_LINE_COLOR, linewidth=0.5)
    ax_t.spines[["top", "right"]].set_visible(False)
    ax_t.legend(frameon=False, fontsize=9, loc="upper right")
    failed = int((~ok).sum())
    fig.suptitle(f"Dynamic obstacles: {sim.status}  |  {int(ok.sum())} replan(s)"
                 + (f", {failed} attempt(s) without a path" if failed else "")
                 + f", {waiting.sum() * sim.dt_s:.1f} s waiting, closest gap "
                 f"{clearance.min():.3f} m (at the samples)")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    print(f"Saved {output}")
    return fig


def plot_simulation_results(grid: np.ndarray, metrics: PlanningMetrics,
                            waypoints: np.ndarray, results_dir: Path,
                            output_dir: Path, inflated: np.ndarray | None = None) -> None:
    """Trajectory figures, if the simulator produced a trajectory."""
    trajectory_file = results_dir / "trajectory.csv"
    if not trajectory_file.is_file():
        print(f"Note: {trajectory_file} not found; skipping trajectory figures")
        return
    trajectory = load_trajectory(trajectory_file)
    sim = load_simulation_metrics(results_dir / "simulation_metrics.csv")
    if len(trajectory) == 0:
        print(f"Note: no robot trajectory (navigation status {sim.status}); "
              f"skipping trajectory figures")
        return
    plot_trajectory(grid, metrics, waypoints, trajectory, sim,
                    output_dir / "trajectory.png", inflated)
    plot_position_vs_time(trajectory, sim, output_dir / "robot_position_vs_time.png")
    scans = load_lidar(results_dir / "lidar_scans.csv")
    if scans is not None:
        check_lidar_matches_trajectory(scans, trajectory)
        plot_lidar_scan(grid, metrics, trajectory, scans, output_dir / "lidar_scan.png",
                        inflated)
    localization = load_localization(results_dir / "localization.csv")
    if localization is not None:
        check_localization_matches_trajectory(localization, trajectory)
        plot_localization(grid, metrics, localization, output_dir / "localization.png", inflated)
        plot_localization_errors(localization, output_dir / "localization_errors.png")
    dynamic = load_dynamic(results_dir)
    if dynamic is not None:
        check_dynamic_matches_trajectory(dynamic[0], trajectory)
        plot_replanning(grid, metrics, waypoints, trajectory, *dynamic, sim,
                        output_dir / "replanning.png", inflated)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot an occupancy grid and the planned A* path.")
    parser.add_argument("--map", type=Path, required=True, help="map CSV file")
    parser.add_argument("--results-dir", type=Path, default=Path("results"),
                        help="directory with the robot_simulator CSV outputs "
                             "(default results/)")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="where to save PNGs (default: --results-dir)")
    parser.add_argument("--map-only", action="store_true",
                        help="only plot the map; do not read planner results")
    parser.add_argument("--start", type=int, nargs=2, metavar=("X", "Y"),
                        help="start cell to mark with --map-only")
    parser.add_argument("--goal", type=int, nargs=2, metavar=("X", "Y"),
                        help="goal cell to mark with --map-only")
    parser.add_argument("--resolution", type=float, default=0.5,
                        help="cell size in meters with --map-only (default 0.5)")
    parser.add_argument("--show", action="store_true",
                        help="open interactive windows in addition to saving")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.show:
        plt.switch_backend("Agg")  # file output only; no display needed
    output_dir = args.output_dir or args.results_dir

    try:
        grid = load_map(args.map)
        if args.map_only:
            start = tuple(args.start) if args.start else None
            goal = tuple(args.goal) if args.goal else None
            plot_map(grid, args.resolution, start, goal,
                     output_dir / "map_visualization.png")
        else:
            metrics = load_metrics(args.results_dir / "planning_metrics.csv")
            waypoints = load_path(args.results_dir / "path.csv")
            check_results_match_map(grid, metrics, waypoints, args.map)
            inflated = load_inflated_map(args.results_dir, grid, metrics)
            plot_map(grid, metrics.resolution, metrics.start, metrics.goal,
                     output_dir / "map_visualization.png", inflated, metrics.robot_radius)
            plot_planned_path(grid, metrics, waypoints,
                              output_dir / "planned_path.png", inflated)
            plot_simulation_results(grid, metrics, waypoints, args.results_dir,
                                    output_dir, inflated)
    except ResultFileError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
