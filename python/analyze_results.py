#!/usr/bin/env python3
"""Performance analysis of simulator results.

Two modes:

run        Analyze one simulator run (results/ by default). The metrics are
           recomputed from the raw CSV files and cross-checked against the values
           the simulator reported:
             path length  = sum of distances between consecutive waypoints
             travel time  = final timestamp - initial timestamp
             success      = planning succeeded, robot within goal tolerance,
                            no collision, no timeout
           Writes run_metrics.csv and robot_position_vs_time.png.

benchmark  Summarize benchmark_runs.csv (from run_benchmark.py). Failed runs are
           kept in every count and rate. Writes:
             benchmark_summary.csv / benchmark_summary.md   per (N, density)
             planning_time_vs_grid_size.png
             path_length_vs_grid_size.png
             success_rate_vs_obstacle_density.png
             expanded_nodes_vs_grid_size.png
             robot_position_vs_time.png   (from --run-dir, if it has a trajectory)

Every number and figure comes from measured data; nothing is synthesized.

Examples:
    python3 python/analyze_results.py run --results-dir results
    python3 python/analyze_results.py benchmark \\
        --input results/benchmark/benchmark_runs.csv --output-dir results/benchmark
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import visualize_map as vm

FIGURE_DPI = 120
# Ordered series (obstacle density, grid size) use one hue from light to dark,
# so the order of the series can be read from the color as well as the legend.
SEQUENTIAL_BLUES = ("#6baed6", "#4292c6", "#2171b5", "#08519c", "#08306b", "#041c40")
REFERENCE_LINE_COLOR = "#7f7f7f"
GRID_COLOR = "#dddddd"
MARKERS = ("o", "s", "^", "D", "v", "P")
RESOLUTION_M = 0.5
METRIC_TOLERANCE = 1e-3   # m and s; CSV values are rounded to 4-6 decimals

BENCHMARK_COLUMNS = (
    "grid_size", "density", "seed", "planning_status", "planning_success",
    "planning_time_ms", "expanded_nodes", "path_length_steps", "path_length_m",
    "reference_bfs_cost", "optimal", "navigation_status", "navigation_success",
    "travel_time_s", "collision_count",
)


class AnalysisError(Exception):
    """Raised when result files are missing or inconsistent."""


# ---------------------------------------------------------------------------
# Single run
# ---------------------------------------------------------------------------

def _read_rows(path: Path, required: tuple[str, ...]) -> list[dict[str, str]]:
    if not path.is_file():
        raise AnalysisError(f"'{path}' not found")
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = set(required) - set(reader.fieldnames or ())
        if missing:
            raise AnalysisError(f"'{path}' is missing columns: {', '.join(sorted(missing))}")
        return list(reader)


def analyze_run(results_dir: Path) -> dict[str, object]:
    """Recompute the run metrics from raw CSV files and compare with the simulator."""
    try:
        plan = vm.load_metrics(results_dir / "planning_metrics.csv")
        sim = vm.load_simulation_metrics(results_dir / "simulation_metrics.csv")
        trajectory = vm.load_trajectory(results_dir / "trajectory.csv")
    except vm.ResultFileError as error:
        raise AnalysisError(str(error)) from error
    sim_row = _read_rows(results_dir / "simulation_metrics.csv", ("goal_tolerance_m",))[0]
    path_rows = _read_rows(results_dir / "path.csv", vm.PATH_COLUMNS)
    world = np.array([(float(r["world_x"]), float(r["world_y"])) for r in path_rows],
                     dtype=float).reshape(-1, 2)

    path_length = float(np.linalg.norm(np.diff(world, axis=0), axis=1).sum())
    travel_time = (float(trajectory[-1, vm.TRAJ_TIME] - trajectory[0, vm.TRAJ_TIME])
                   if len(trajectory) else None)
    goal_world = (np.array(plan.goal) + 0.5) * plan.resolution
    goal_error = (float(np.linalg.norm(trajectory[-1, [vm.TRAJ_X, vm.TRAJ_Y]] - goal_world))
                  if len(trajectory) else None)
    tolerance = float(sim_row["goal_tolerance_m"])
    success = (plan.success and goal_error is not None and goal_error <= tolerance
               and sim.collision_count == 0 and sim.status != "TIMEOUT")

    metrics = {
        "planning_status": plan.status,
        "planning_success": int(plan.success),
        "planning_time_ms": plan.planning_time_ms,
        "expanded_nodes": plan.expanded_nodes,
        "path_length_m": round(path_length, 6) if plan.success else "",
        "travel_time_s": round(travel_time, 6) if travel_time is not None else "",
        "final_goal_error_m": round(goal_error, 6) if goal_error is not None else "",
        "collision_count": sim.collision_count,
        "navigation_status": sim.status,
        "navigation_success": int(success),
    }

    # Cross-check against the simulator's own numbers.
    problems = []
    if plan.success and abs(path_length - (plan.path_length_m or 0.0)) > METRIC_TOLERANCE:
        problems.append(f"path length {path_length:.4f} m != reported {plan.path_length_m} m")
    if travel_time is not None and sim.travel_time_s is not None and \
            abs(travel_time - sim.travel_time_s) > METRIC_TOLERANCE:
        problems.append(f"travel time {travel_time:.4f} s != reported {sim.travel_time_s} s")
    if success != sim.navigation_success:
        problems.append(f"navigation success {success} != reported {sim.navigation_success}")
    if problems:
        raise AnalysisError("results are inconsistent: " + "; ".join(problems))
    return metrics


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_run_position(results_dir: Path, output: Path) -> bool:
    """robot_position_vs_time.png from a run directory. False if there is no trajectory."""
    trajectory_file = results_dir / "trajectory.csv"
    if not trajectory_file.is_file():
        return False
    trajectory = vm.load_trajectory(trajectory_file)
    if len(trajectory) == 0:
        return False
    sim = vm.load_simulation_metrics(results_dir / "simulation_metrics.csv")
    plt.close(vm.plot_position_vs_time(trajectory, sim, output))
    return True


def command_run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir or args.results_dir
    try:
        metrics = analyze_run(args.results_dir)
    except (AnalysisError, vm.ResultFileError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    width = max(len(k) for k in metrics)
    for key, value in metrics.items():
        print(f"  {key:<{width}}  {value}")
    print("  (path length, travel time and success recomputed from the raw CSV files; "
          "they match the simulator's values)")
    write_rows(output_dir / "run_metrics.csv", [metrics])
    print(f"Saved {output_dir / 'run_metrics.csv'}")
    if not plot_run_position(args.results_dir, output_dir / "robot_position_vs_time.png"):
        print("Note: no trajectory; robot_position_vs_time.png not created")
    return 0


# ---------------------------------------------------------------------------
# Benchmark
# ---------------------------------------------------------------------------

def _float_or_nan(text: str) -> float:
    return float(text) if text != "" else math.nan


def load_benchmark(path: Path) -> dict[str, np.ndarray]:
    rows = _read_rows(path, BENCHMARK_COLUMNS)
    if not rows:
        raise AnalysisError(f"'{path}' contains no runs")
    try:
        return {
            "grid_size": np.array([int(r["grid_size"]) for r in rows]),
            "density": np.array([float(r["density"]) for r in rows]),
            "planning_success": np.array([r["planning_success"] == "1" for r in rows]),
            "planning_time_ms": np.array([float(r["planning_time_ms"]) for r in rows]),
            "expanded_nodes": np.array([int(r["expanded_nodes"]) for r in rows]),
            "path_length_m": np.array([_float_or_nan(r["path_length_m"]) for r in rows]),
            "optimal": np.array([_float_or_nan(r["optimal"]) for r in rows]),
            "navigation_success": np.array([r["navigation_success"] == "1" for r in rows]),
            "travel_time_s": np.array([_float_or_nan(r["travel_time_s"]) for r in rows]),
            "collision_count": np.array([int(r["collision_count"]) for r in rows]),
        }
    except ValueError as error:
        raise AnalysisError(f"'{path}' has a malformed value: {error}") from error


def summarize(data: dict[str, np.ndarray]) -> list[dict[str, object]]:
    """One row per (grid size, density). All runs count; failures are not dropped."""
    summary = []
    for size in np.unique(data["grid_size"]):
        for density in np.unique(data["density"]):
            sel = (data["grid_size"] == size) & (data["density"] == density)
            if not sel.any():
                continue
            runs = int(sel.sum())
            times = data["planning_time_ms"][sel]
            planned = sel & data["planning_success"]
            lengths = data["path_length_m"][planned]
            checked = data["optimal"][sel]
            checked = checked[~np.isnan(checked)]
            summary.append({
                "grid_size": int(size),
                "density": float(density),
                "runs": runs,
                "planning_successes": int(planned.sum()),
                "navigation_successes": int((sel & data["navigation_success"]).sum()),
                "navigation_success_rate": round(float(data["navigation_success"][sel].mean()), 4),
                "planning_time_median_ms": round(float(np.median(times)), 6),
                "planning_time_mean_ms": round(float(times.mean()), 6),
                "planning_time_min_ms": round(float(times.min()), 6),
                "planning_time_max_ms": round(float(times.max()), 6),
                "planning_time_p95_ms": round(float(np.percentile(times, 95)), 6),
                "expanded_nodes_median": float(np.median(data["expanded_nodes"][sel])),
                "expanded_nodes_mean": round(float(data["expanded_nodes"][sel].mean()), 1),
                "path_length_median_m": round(float(np.median(lengths)), 4) if len(lengths) else "",
                "path_length_mean_m": round(float(lengths.mean()), 4) if len(lengths) else "",
                "travel_time_median_s": (round(float(np.median(data["travel_time_s"][planned])), 4)
                                         if planned.any() else ""),
                "collisions": int(data["collision_count"][sel].sum()),
                "bfs_checked": int(len(checked)),
                "bfs_optimal": int(checked.sum()),
            })
    return summary


def write_markdown_summary(path: Path, summary: list[dict[str, object]]) -> None:
    header = ("| N | density | runs | nav. success | median ms | mean ms | min ms | "
              "max ms | p95 ms | median expanded | median path (m) | A* = BFS |")
    lines = [header, "|" + "---|" * 12]
    for row in summary:
        lines.append(
            f"| {row['grid_size']} | {row['density']:.0%} | {row['runs']} | "
            f"{row['navigation_successes']}/{row['runs']} | "
            f"{row['planning_time_median_ms']:.4f} | {row['planning_time_mean_ms']:.4f} | "
            f"{row['planning_time_min_ms']:.4f} | {row['planning_time_max_ms']:.4f} | "
            f"{row['planning_time_p95_ms']:.4f} | {row['expanded_nodes_median']:.0f} | "
            f"{row['path_length_median_m'] if row['path_length_median_m'] != '' else '—'} | "
            f"{row['bfs_optimal']}/{row['bfs_checked']} |")
    path.write_text("\n".join(lines) + "\n")


def _style(ax, xlabel: str, ylabel: str) -> None:
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(color=GRID_COLOR, linewidth=0.6)
    ax.spines[["top", "right"]].set_visible(False)


def _save(fig, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def _per_density_series(data, metric, sizes, success_only=False):
    """Yield (density, median, q25, q75) arrays over grid sizes."""
    for density in np.unique(data["density"]):
        med, lo, hi = [], [], []
        for size in sizes:
            sel = (data["grid_size"] == size) & (data["density"] == density)
            if success_only:
                sel &= data["planning_success"]
            values = data[metric][sel]
            values = values[~np.isnan(values)] if values.dtype.kind == "f" else values
            if len(values) == 0:
                med.append(np.nan), lo.append(np.nan), hi.append(np.nan)
                continue
            med.append(np.median(values))
            lo.append(np.percentile(values, 25))
            hi.append(np.percentile(values, 75))
        yield density, np.array(med, float), np.array(lo, float), np.array(hi, float)


def _error_bars(med: np.ndarray, lo: np.ndarray, hi: np.ndarray, scale=1.0):
    """Asymmetric error-bar lengths, clipped at 0 (percentiles can differ from the
    median by rounding error; NaN marks configurations without data)."""
    return [np.clip((med - lo) / scale, 0.0, None), np.clip((hi - med) / scale, 0.0, None)]


def _offsets(count: int, step: float) -> np.ndarray:
    """Small horizontal offsets so error bars of different series do not overlap."""
    return (np.arange(count) - (count - 1) / 2) * step


def plot_metric_vs_grid_size(data, metric: str, ylabel: str, title: str, output: Path,
                             *, log_y: bool, reference=None) -> None:
    """Median over successful plans per (N, density); error bars: 25th–75th percentile."""
    sizes = np.unique(data["grid_size"])
    densities = np.unique(data["density"])
    offsets = _offsets(len(densities), 0.9)
    fig, ax = plt.subplots(figsize=(8, 5.2), layout="constrained")
    for i, (density, med, lo, hi) in enumerate(
            _per_density_series(data, metric, sizes, success_only=True)):
        color = SEQUENTIAL_BLUES[min(i, len(SEQUENTIAL_BLUES) - 1)]
        x = sizes + offsets[i]
        ax.errorbar(x, med, yerr=_error_bars(med, lo, hi), color=color, ecolor=color,
                    elinewidth=1.2, capsize=3, marker=MARKERS[i % len(MARKERS)],
                    markersize=7, linewidth=2, label=f"{density:.0%} obstacles")
    if reference is not None:
        ref_label, ref_values = reference
        ax.plot(sizes, ref_values, color=REFERENCE_LINE_COLOR, linestyle="--",
                linewidth=1.5, label=ref_label)
    if log_y:
        ax.set_yscale("log")
    ax.set_xticks(sizes)
    _style(ax, "grid size N (map of N × N cells)", ylabel)
    ax.legend(frameon=False)
    ax.set_title(title, fontsize=11)
    _save(fig, output)


def plot_path_length(data, output: Path) -> None:
    """Absolute path length, and path length relative to the obstacle-free minimum."""
    sizes = np.unique(data["grid_size"])
    densities = np.unique(data["density"])
    offsets = _offsets(len(densities), 0.9)
    minimum = 2 * (sizes - 1) * RESOLUTION_M
    fig, (ax_len, ax_ratio) = plt.subplots(2, 1, figsize=(8, 7.5), sharex=True,
                                           layout="constrained")
    for i, (density, med, lo, hi) in enumerate(
            _per_density_series(data, "path_length_m", sizes, success_only=True)):
        color = SEQUENTIAL_BLUES[min(i, len(SEQUENTIAL_BLUES) - 1)]
        style = dict(color=color, ecolor=color, elinewidth=1.2, capsize=3,
                     marker=MARKERS[i % len(MARKERS)], markersize=7, linewidth=2)
        x = sizes + offsets[i]
        ax_len.errorbar(x, med, yerr=_error_bars(med, lo, hi), label=f"{density:.0%} obstacles",
                        **style)
        ax_ratio.errorbar(x, med / minimum, yerr=_error_bars(med, lo, hi, minimum),
                          **style)
    ax_len.plot(sizes, minimum, color=REFERENCE_LINE_COLOR, linestyle="--", linewidth=1.5,
                label="obstacle-free minimum 2(N−1) × 0.5 m")
    ax_ratio.axhline(1.0, color=REFERENCE_LINE_COLOR, linestyle="--", linewidth=1.5)
    _style(ax_len, "", "planned path length (m)")
    _style(ax_ratio, "grid size N (map of N × N cells)",
           "path length / obstacle-free minimum")
    ax_ratio.set_xticks(sizes)
    ax_len.legend(frameon=False)
    ax_len.set_title("Planned path length vs. grid size (successful plans; "
                     "median, bars: 25th–75th percentile)", fontsize=11)
    _save(fig, output)


def wilson_interval(successes: int, total: int, z: float = 1.96) -> tuple[float, float]:
    """95% Wilson score interval for a success proportion."""
    if total == 0:
        return math.nan, math.nan
    p = successes / total
    denom = 1 + z * z / total
    center = (p + z * z / (2 * total)) / denom
    half = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denom
    return center - half, center + half


def plot_success_rate(data, output: Path) -> None:
    sizes = np.unique(data["grid_size"])
    densities = np.unique(data["density"])
    fig, ax = plt.subplots(figsize=(8, 5.2), layout="constrained")
    for i, size in enumerate(sizes):
        rates = [100.0 * data["navigation_success"][(data["grid_size"] == size)
                                                    & (data["density"] == d)].mean()
                 for d in densities]
        color = SEQUENTIAL_BLUES[min(i, len(SEQUENTIAL_BLUES) - 1)]
        ax.plot(densities * 100, rates, color=color, marker=MARKERS[i % len(MARKERS)],
                markersize=6, linewidth=1, alpha=0.55, label=f"{size} × {size}")
    pooled, low, high, counts = [], [], [], []
    for d in densities:
        sel = data["density"] == d
        successes, total = int(data["navigation_success"][sel].sum()), int(sel.sum())
        lo, hi = wilson_interval(successes, total)
        pooled.append(100.0 * successes / total)
        low.append(100.0 * lo)
        high.append(100.0 * hi)
        counts.append(total)
    pooled = np.array(pooled)
    ax.errorbar(densities * 100, pooled, yerr=_error_bars(pooled, np.array(low), np.array(high)),
                color="#1a1a1a", ecolor="#1a1a1a", elinewidth=1.5, capsize=4, marker="o",
                markersize=8, linewidth=2.8, label=f"all sizes ({counts[0]} maps, 95% CI)",
                zorder=5)
    runs_per_point = int(((data["grid_size"] == sizes[0]) &
                          (data["density"] == densities[0])).sum())
    ax.set_xticks(densities * 100)
    ax.set_ylim(-3, 103)
    _style(ax, "obstacle density (%)", "navigation success rate (%)")
    ax.legend(title="grid size", frameon=False, loc="lower left")
    ax.set_title(f"Navigation success rate vs. obstacle density "
                 f"({runs_per_point} random maps per size, failures included)", fontsize=11)
    _save(fig, output)


def command_benchmark(args: argparse.Namespace) -> int:
    try:
        data = load_benchmark(args.input)
    except AnalysisError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    out = args.output_dir
    summary = summarize(data)
    write_rows(out / "benchmark_summary.csv", summary)
    write_markdown_summary(out / "benchmark_summary.md", summary)
    print(f"Saved {out / 'benchmark_summary.csv'} and {out / 'benchmark_summary.md'}")

    sizes = np.unique(data["grid_size"])
    bars = "successful plans; median, bars: 25th–75th percentile"
    plot_metric_vs_grid_size(
        data, "planning_time_ms", "A* planning time (ms, log scale)",
        f"A* planning time vs. grid size ({bars})",
        out / "planning_time_vs_grid_size.png", log_y=True)
    plot_path_length(data, out / "path_length_vs_grid_size.png")
    plot_metric_vs_grid_size(
        data, "expanded_nodes", "expanded nodes (log scale)",
        f"A* expanded nodes vs. grid size ({bars})",
        out / "expanded_nodes_vs_grid_size.png", log_y=True,
        reference=("all cells (N²)", sizes.astype(float) ** 2))
    plot_success_rate(data, out / "success_rate_vs_obstacle_density.png")
    if args.run_dir is not None and not plot_run_position(
            args.run_dir, out / "robot_position_vs_time.png"):
        print(f"Note: no trajectory in {args.run_dir}; robot_position_vs_time.png skipped")

    total = len(data["grid_size"])
    checked = data["optimal"][~np.isnan(data["optimal"])]
    print(f"{total} runs: {int(data['planning_success'].sum())} planned, "
          f"{int(data['navigation_success'].sum())} navigated successfully, "
          f"{int(data['collision_count'].sum())} collisions, "
          f"A* = BFS in {int(checked.sum())}/{len(checked)} checked runs")
    return 0 if checked.all() else 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze simulator results.")
    sub = parser.add_subparsers(dest="mode", required=True)
    run = sub.add_parser("run", help="analyze one simulator run")
    run.add_argument("--results-dir", type=Path, default=Path("results"),
                     help="directory with the simulator CSV files (default results/)")
    run.add_argument("--output-dir", type=Path, default=None,
                     help="where to write outputs (default: --results-dir)")
    bench = sub.add_parser("benchmark", help="summarize benchmark_runs.csv")
    bench.add_argument("--input", type=Path,
                       default=Path("results/benchmark/benchmark_runs.csv"),
                       help="benchmark runs CSV (default results/benchmark/benchmark_runs.csv)")
    bench.add_argument("--output-dir", type=Path, default=Path("results/benchmark"),
                       help="where to write the summary and plots (default results/benchmark/)")
    bench.add_argument("--run-dir", type=Path, default=Path("results"),
                       help="single-run directory for robot_position_vs_time.png "
                            "(default results/)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plt.switch_backend("Agg")
    return command_run(args) if args.mode == "run" else command_benchmark(args)


if __name__ == "__main__":
    sys.exit(main())
