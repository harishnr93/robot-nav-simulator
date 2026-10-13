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
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Patch

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
        )
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error


def load_trajectory(path: Path) -> np.ndarray:
    """Return an (N, 4) array of (time_s, x_m, y_m, target_waypoint_index).

    N may be 0. Columns are addressed with the TRAJ_* constants.
    """
    rows = _read_csv(path, TRAJECTORY_COLUMNS)
    try:
        data = np.array([(float(r["time"]), float(r["x"]), float(r["y"]),
                          int(r["target_waypoint_index"]))
                         for r in rows], dtype=float).reshape(-1, 4)
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error
    if not np.isfinite(data).all():
        raise ResultFileError(f"'{path}' contains non-finite values")
    if len(data) > 1 and not (np.diff(data[:, 0]) > 0).all():
        raise ResultFileError(f"'{path}' timestamps are not strictly increasing")
    return data


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
    time_s = trajectory[-1, TRAJ_TIME] - trajectory[0, TRAJ_TIME]
    distance = sim.distance_traveled_m or 0.0
    fig.suptitle(f"Navigation: {sim.status}  |  {time_s:.1f} s, {distance:.2f} m "
                 f"traveled, {sim.collision_count} collision(s)")
    _finish(fig, ax, output, extra)
    return fig


def plot_position_vs_time(trajectory: np.ndarray, sim: SimulationMetrics, output: Path):
    """x(t) and y(t) in meters, one panel each (same units, separate axes)."""
    fig, (ax_x, ax_y) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True,
                                     layout="constrained")
    time_s = trajectory[:, TRAJ_TIME]
    for ax, values, name in ((ax_x, trajectory[:, TRAJ_X], "x"),
                             (ax_y, trajectory[:, TRAJ_Y], "y")):
        ax.plot(time_s, values, color=TRAJECTORY_COLOR, linewidth=2)
        ax.set_ylabel(f"{name} (m)")
        ax.grid(color=GRID_LINE_COLOR, linewidth=0.5)
        ax.spines[["top", "right"]].set_visible(False)
    ax_y.set_xlabel("simulation time (s)")
    ax_y.invert_yaxis()  # world +y points down, matching the map figures
    ax_y.set_ylabel("y (m, increases downward)")
    fig.suptitle(f"Robot position vs. time  ({sim.status}, speed {sim.speed_mps:g} m/s, "
                 f"dt {sim.dt_s:g} s)")
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
