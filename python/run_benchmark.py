#!/usr/bin/env python3
"""Repeatable planning and navigation benchmark.

For every combination of grid size N, obstacle density d and seed s:
    1. generate an N x N random map (each cell occupied with probability d,
       seeded with (s, N, round(100 d)) so every configuration is reproducible);
    2. clear only the start cell (0, 0) and the goal cell (N-1, N-1). A path is
       NOT guaranteed: unreachable goals are recorded as failures, never discarded;
    3. run build/robot_simulator with --reference-bfs, which plans with A*,
       verifies the cost against the reference BFS, and simulates the robot;
    4. record the simulator's own measurements (planning time is measured inside
       the C++ process with std::chrono::steady_clock around the search only).

Runs are executed one at a time so they do not compete for the CPU.

Writes (to --output-dir, default results/benchmark/):
    benchmark_runs.csv       one row per run, including failed runs
    benchmark_metadata.json  machine, compiler, build type, versions, parameters
and then calls analyze_results.py to produce the summary tables and plots.

Example:
    python3 python/run_benchmark.py                      # full sweep, 480 runs
    python3 python/run_benchmark.py --sizes 10 20 --seeds 3
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import platform
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import matplotlib
import numpy as np

import analyze_results
import generate_map

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SIMULATOR = REPO_ROOT / "build" / "robot_simulator"
DEFAULT_SIZES = (10, 20, 40, 60, 80, 100)
DEFAULT_DENSITIES = (0.0, 0.1, 0.2, 0.3)
DEFAULT_SEEDS = 20
# Generous limit so that no run times out merely because the default 600 s limit is
# too short for a long path on a 100 x 100 map (99 m straight-line minimum).
BENCHMARK_MAX_TIME_S = 5000.0

# robot_simulator exit codes that still produce a complete result set.
EXIT_OK, EXIT_NO_PATH, EXIT_NAV_FAILED, EXIT_REFERENCE_MISMATCH = 0, 4, 6, 7
EXPECTED_EXIT_CODES = {EXIT_OK, EXIT_NO_PATH, EXIT_NAV_FAILED, EXIT_REFERENCE_MISMATCH}

RUN_COLUMNS = (
    "grid_size", "density", "seed", "actual_density",
    "planning_status", "planning_success", "planning_time_ms", "expanded_nodes",
    "path_length_steps", "path_length_m", "reference_bfs_cost", "optimal",
    "navigation_status", "navigation_success", "travel_time_s",
    "distance_traveled_m", "final_goal_error_m", "collision_count", "exit_code",
)


class BenchmarkError(Exception):
    """Raised when the benchmark cannot continue."""


def make_map(size: int, density: float, seed: int) -> np.ndarray:
    rng = np.random.default_rng([seed, size, int(round(density * 100))])
    grid = generate_map.generate_random(size, size, density, rng)
    grid[0, 0] = generate_map.FREE
    grid[size - 1, size - 1] = generate_map.FREE
    return grid


def read_single_row(path: Path) -> dict[str, str]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 1:
        raise BenchmarkError(f"'{path}' must contain exactly one data row")
    return rows[0]


def run_once(simulator: Path, size: int, density: float, seed: int,
             work_dir: Path) -> dict[str, object]:
    grid = make_map(size, density, seed)
    map_file = work_dir / "map.csv"
    generate_map.write_map(grid, map_file)
    out_dir = work_dir / "out"
    command = [str(simulator), "--map", str(map_file),
               "--start", "0", "0", "--goal", str(size - 1), str(size - 1),
               "--max-time", repr(BENCHMARK_MAX_TIME_S),
               "--reference-bfs", "--output", str(out_dir)]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode not in EXPECTED_EXIT_CODES:
        raise BenchmarkError(
            f"robot_simulator exited with {completed.returncode} for N={size}, "
            f"d={density}, seed={seed}:\n{completed.stderr.strip()}")

    plan = read_single_row(out_dir / "planning_metrics.csv")
    sim = read_single_row(out_dir / "simulation_metrics.csv")
    planning_success = plan["success"] == "1"
    reference = plan["reference_bfs_cost"]
    optimal = ""
    if planning_success and reference not in ("", "-1"):
        optimal = int(int(plan["path_length_steps"]) == int(reference))
    elif not planning_success and reference == "-1":
        optimal = 1  # both agree that there is no path
    return {
        "grid_size": size, "density": density, "seed": seed,
        "actual_density": round(float(grid.mean()), 6),
        "planning_status": plan["status"],
        "planning_success": int(planning_success),
        "planning_time_ms": plan["planning_time_ms"],
        "expanded_nodes": plan["expanded_nodes"],
        "path_length_steps": plan["path_length_steps"],
        "path_length_m": plan["path_length_m"],
        "reference_bfs_cost": reference,
        "optimal": optimal,
        "navigation_status": sim["status"],
        "navigation_success": sim["navigation_success"],
        "travel_time_s": sim["travel_time_s"],
        "distance_traveled_m": sim["distance_traveled_m"],
        "final_goal_error_m": sim["final_goal_error_m"],
        "collision_count": sim["collision_count"],
        "exit_code": completed.returncode,
    }


def command_output(command: list[str]) -> str:
    try:
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
    except OSError:
        return ""
    return completed.stdout.strip()


def cmake_cache_value(key: str) -> str:
    cache = REPO_ROOT / "build" / "CMakeCache.txt"
    if not cache.is_file():
        return ""
    for line in cache.read_text().splitlines():
        if line.startswith(key + ":"):
            return line.split("=", 1)[1]
    return ""


def cpu_model() -> str:
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor()


def collect_metadata(args: argparse.Namespace, elapsed_s: float, run_count: int) -> dict:
    compiler = cmake_cache_value("CMAKE_CXX_COMPILER")
    git_commit = command_output(["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"])
    git_dirty = bool(command_output(["git", "-C", str(REPO_ROOT), "status", "--porcelain"]))
    return {
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "machine": {
            "cpu": cpu_model(),
            "logical_cpus": os.cpu_count(),
            "platform": platform.platform(),
        },
        "build": {
            "compiler": compiler,
            "compiler_version": command_output([compiler, "--version"]).splitlines()[0]
            if compiler else "",
            "cmake_build_type": cmake_cache_value("CMAKE_BUILD_TYPE"),
            "git_commit": git_commit + ("-dirty" if git_dirty else ""),
        },
        "software": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "matplotlib": matplotlib.__version__,
        },
        "parameters": {
            "grid_sizes": list(args.sizes),
            "densities": list(args.densities),
            "seeds_per_configuration": args.seeds,
            "runs": run_count,
            "start": "(0, 0)",
            "goal": "(N-1, N-1)",
            "resolution_m": 0.5,
            "speed_mps": 0.5,
            "dt_s": 0.1,
            "goal_tolerance_m": 0.05,
            "max_time_s": BENCHMARK_MAX_TIME_S,
            "map_seed": "numpy default_rng([seed, N, round(100 * density)])",
        },
        "elapsed_wall_time_s": round(elapsed_s, 1),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark A* planning and robot navigation on random maps.")
    parser.add_argument("--sizes", type=int, nargs="+", default=list(DEFAULT_SIZES),
                        help="grid sizes N (N x N maps)")
    parser.add_argument("--densities", type=float, nargs="+",
                        default=list(DEFAULT_DENSITIES), help="obstacle densities, 0..1")
    parser.add_argument("--seeds", type=int, default=DEFAULT_SEEDS,
                        help=f"random seeds per configuration (default {DEFAULT_SEEDS})")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "results" / "benchmark",
                        help="output directory (default results/benchmark/)")
    parser.add_argument("--simulator", type=Path, default=DEFAULT_SIMULATOR,
                        help="robot_simulator executable (default build/robot_simulator)")
    parser.add_argument("--no-analyze", action="store_true",
                        help="only collect data; do not run analyze_results.py")
    args = parser.parse_args(argv)
    if any(n < 2 for n in args.sizes):
        parser.error("--sizes must be at least 2")
    if any(not 0.0 <= d <= 1.0 for d in args.densities):
        parser.error("--densities must be between 0 and 1")
    if args.seeds < 1:
        parser.error("--seeds must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.simulator.is_file():
        print(f"Error: simulator not found at '{args.simulator}'. Build it first:\n"
              f"    cmake -S . -B build && cmake --build build -j", file=sys.stderr)
        return 1
    build_type = cmake_cache_value("CMAKE_BUILD_TYPE")
    if build_type and build_type != "Release":
        print(f"Warning: build type is '{build_type}', not Release; planning times "
              f"will not be representative", file=sys.stderr)

    configurations = [(n, d, s) for n in args.sizes for d in args.densities
                      for s in range(args.seeds)]
    rows = []
    started = time.perf_counter()
    try:
        with tempfile.TemporaryDirectory(prefix="amr_benchmark_") as tmp:
            for index, (size, density, seed) in enumerate(configurations, start=1):
                rows.append(run_once(args.simulator, size, density, seed, Path(tmp)))
                if index % 40 == 0 or index == len(configurations):
                    print(f"  {index}/{len(configurations)} runs", flush=True)
    except BenchmarkError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    elapsed = time.perf_counter() - started

    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs_file = args.output_dir / "benchmark_runs.csv"
    with runs_file.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=RUN_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    metadata_file = args.output_dir / "benchmark_metadata.json"
    metadata_file.write_text(json.dumps(collect_metadata(args, elapsed, len(rows)), indent=2)
                             + "\n")
    mismatches = sum(1 for r in rows if r["exit_code"] == EXIT_REFERENCE_MISMATCH)
    print(f"Wrote {runs_file} ({len(rows)} runs in {elapsed:.1f} s) and {metadata_file}")
    if mismatches:
        print(f"Error: A* disagreed with the reference BFS in {mismatches} run(s)",
              file=sys.stderr)

    status = 1 if mismatches else 0
    if not args.no_analyze:
        status = analyze_results.main(["benchmark", "--input", str(runs_file),
                                       "--output-dir", str(args.output_dir)]) or status
    return status


if __name__ == "__main__":
    sys.exit(main())
