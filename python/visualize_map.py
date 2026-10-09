#!/usr/bin/env python3
"""Static visualization of an occupancy grid and the A* path planned by C++.

Reads:
    --map FILE                         occupancy grid CSV (0 free, 1 occupied)
    RESULTS_DIR/planning_metrics.csv   written by build/robot_simulator
    RESULTS_DIR/path.csv               written by build/robot_simulator

Writes:
    OUTPUT_DIR/map_visualization.png   map with start and goal
    OUTPUT_DIR/planned_path.png        map with the planned path, or a
                                       "No valid path" message on failure

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
from matplotlib.patches import Patch

FREE_COLOR = "#f2f2f2"
OCCUPIED_COLOR = "#3c3c3c"
START_COLOR = "#2ca02c"
GOAL_COLOR = "#d62728"
PATH_COLOR = "#1f77b4"
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
        )
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error


def load_path(path: Path) -> np.ndarray:
    """Return an (N, 2) array of grid (x, y) waypoints; N may be 0."""
    rows = _read_csv(path, PATH_COLUMNS)
    try:
        points = [(int(row["grid_x"]), int(row["grid_y"])) for row in rows]
    except ValueError as error:
        raise ResultFileError(f"'{path}' has a malformed value: {error}") from error
    return np.array(points, dtype=int).reshape(-1, 2)


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


def _finish(fig, ax, output: Path) -> None:
    handles, _ = ax.get_legend_handles_labels()
    fig.legend(handles=_legend_handles() + handles, loc="outside lower center",
               ncol=3, frameon=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    print(f"Saved {output}")


def plot_map(grid: np.ndarray, resolution: float, start, goal, output: Path):
    fig, ax = plt.subplots(figsize=FIGURE_SIZE, layout="constrained")
    draw_grid(ax, grid, resolution)
    if start is not None and goal is not None:
        draw_start_goal(ax, start, goal)
    height, width = grid.shape
    fig.suptitle(f"Occupancy grid  ({width} x {height} cells, {resolution:g} m/cell)")
    _finish(fig, ax, output)
    return fig


def plot_planned_path(grid: np.ndarray, metrics: PlanningMetrics,
                      waypoints: np.ndarray, output: Path):
    fig, ax = plt.subplots(figsize=FIGURE_SIZE, layout="constrained")
    draw_grid(ax, grid, metrics.resolution)
    if metrics.success and len(waypoints) > 0:
        ax.plot(waypoints[:, 0], waypoints[:, 1], color=PATH_COLOR, linewidth=2.5,
                marker="o", markersize=3.5, label="Planned A* path", zorder=3)
        title = (f"A* path: {metrics.path_length_steps} steps, "
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
    fig.suptitle(title)
    _finish(fig, ax, output)
    return fig


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot an occupancy grid and the planned A* path.")
    parser.add_argument("--map", type=Path, required=True, help="map CSV file")
    parser.add_argument("--results-dir", type=Path, default=Path("results"),
                        help="directory with path.csv and planning_metrics.csv "
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
            plot_map(grid, metrics.resolution, metrics.start, metrics.goal,
                     output_dir / "map_visualization.png")
            plot_planned_path(grid, metrics, waypoints,
                              output_dir / "planned_path.png")
    except ResultFileError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
