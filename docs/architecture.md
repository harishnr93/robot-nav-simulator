# Architecture

This document describes the modules of the simulator and how data flows
between them.

## Data flow

```text
 ┌─────────────────────┐   maps/*.csv   ┌──────────────────────────────────────────┐
 │ python/             │ ─────────────► │ build/robot_simulator  (src/main.cpp)    │
 │   generate_map.py   │                │   1. parse CLI arguments                 │
 └─────────────────────┘                │   2. loadGridMapCsv()      grid_map.cpp  │
                                        │   3. planAStar()           astar.cpp     │
                                        │   4. simulatePath()        simulator.cpp │
                                        │        └ stepToward()      robot.cpp     │
                                        │   5. write*Csv()           csv_writer.cpp│
                                        └───────────────┬──────────────────────────┘
                                                        │ results/path.csv
                                                        │ results/planning_metrics.csv
                                                        │ results/trajectory.csv
                                                        │ results/simulation_metrics.csv
                                        ┌───────────────▼──────────────────────────┐
                                        │ python/visualize_map.py                  │
                                        │   → map_visualization.png                │
                                        │   → planned_path.png                     │
                                        │   → trajectory.png                       │
                                        │   → robot_position_vs_time.png           │
                                        ├──────────────────────────────────────────┤
                                        │ python/animate_robot.py                  │
                                        │   → navigation.gif  (navigation.mp4)     │
                                        │   → navigation_failed.png (no path)      │
                                        └──────────────────────────────────────────┘

 python/run_demo.py runs robot_simulator (subprocess, argument list) and then
 visualize_map.main() and animate_robot.main() in-process.

 Benchmark:  run_benchmark.py ──► (random map → robot_simulator --reference-bfs) × 480
                    │                 one process at a time, results read back from CSV
                    ▼
             results/benchmark/benchmark_runs.csv + benchmark_metadata.json
                    │
             analyze_results.py benchmark ──► benchmark_summary.csv/.md + 4 plots
```

C++ and Python communicate only through CSV files. Python never re-implements the
planner, so the C++ A* implementation is the single source of truth.

## C++ modules

All C++ code lives in namespace `amr`. The library target `amr_core` contains
everything except `main.cpp`, so tests link against exactly the code the executable
uses.

| File | Responsibility |
|---|---|
| `include/types.hpp` | Value types shared by all modules: `GridCoord`, `WorldPoint`, `PlanStatus`, `PlanResult`. Documents the coordinate convention. |
| `include/grid_map.hpp`, `src/grid_map.cpp` | `GridMap` occupancy grid (row-major `std::vector<uint8_t>`), bounds/occupancy queries, grid→world conversion, CSV parsing and validation. |
| `include/astar.hpp`, `src/astar.cpp` | Request validation, A* search (`AStarPlanner`, reusable, plus the one-shot `planAStar()`), path reconstruction, path validity check, path length, and a reference BFS used only for validation. |
| `benchmarks/bench_planner.cpp` | In-process micro-benchmark of fresh vs. reused planning, with a result checksum for comparing builds. |
| `include/robot.hpp`, `src/robot.cpp` | `RobotState`, `MotionConfig` (+ validation), and `stepToward()`: one clamped kinematic step of the point robot. |
| `include/simulator.hpp`, `src/simulator.cpp` | `simulatePath()`: time-stepped waypoint following, segment collision checking, timeout, trajectory recording; `NavigationStatus`, `SimulationResult`. |
| `include/csv_writer.hpp`, `src/csv_writer.cpp` | Writes `path.csv`, `planning_metrics.csv`, `trajectory.csv` and `simulation_metrics.csv`. |
| `src/main.cpp` | CLI parsing, orchestration, console summary, exit codes. |

### Key data structures

- **`GridMap`**: `width × height` cells stored row-major (`index = y * width + x`).
  Out-of-bounds queries count as occupied, which makes neighbor checks in the planner a
  single `isFree()` call.
- **`AStarPlanner`**: one state byte per cell (visited, closed, 2-bit parent direction),
  a cost array written only for reached cells, two `std::priority_queue` min-heaps for
  the open set, and the list of touched cells used to reset the state between plans
  (details in [algorithms.md](algorithms.md)).
- **`PlanResult`**: status, path (start…goal inclusive), path cost in steps, expanded
  node count, and planning time.
- **`MotionConfig`**: speed (m/s), timestep `dt` (s), goal tolerance (m), time limit
  (s). Validated by `validateMotionConfig()`; `max_time / dt` is capped at 10⁷ steps to
  bound memory use.
- **`SimulationResult`**: navigation status, trajectory samples (`RobotState` + target
  waypoint index), steps, collision count, distance traveled, final goal error.
- **`MapLoadResult`**: `ok` flag + `GridMap` + human-readable error. Loading never
  throws on bad input. The `GridMap` constructor *does* throw `std::invalid_argument`,
  but only for programming errors (the loader validates before constructing).

### Error handling

| Situation | Handling | CLI exit code |
|---|---|---|
| Bad/missing CLI arguments | message + `--help` hint | 1 |
| Missing, empty, non-rectangular or invalid map | `MapLoadResult.error` with file and line number | 2 |
| Start/goal out of bounds or inside an obstacle | `PlanStatus` other than `Success` | 3 |
| Goal unreachable | `PlanStatus::NoPath`, empty path, simulation `NOT_RUN` | 4 |
| Cannot write results | message | 5 |
| Collision or timeout while moving | `NavigationStatus::Collision` / `Timeout`, partial trajectory | 6 |
| `--reference-bfs` and A* disagree with the BFS (a planner bug) | message; results are still written | 7 |
| Invalid motion parameters (`--speed`, `--dt`, ...) | rejected during argument parsing | 1 |

All four result files are written for every planning outcome (codes 0, 3, 4, 6). Files
without data contain only their header, so results from an earlier run can never be
mistaken for the current one. The simulator only runs when planning succeeded.

## Python modules

| File | Responsibility |
|---|---|
| `python/generate_map.py` | Creates `empty`, `obstacles`, `corridor`, `maze` and `random` maps. Seeded randomness; optional reachability guarantee via flood fill (a connectivity check, not a planner). |
| `python/visualize_map.py` | Loads a map and the C++ results, checks they belong together (size, path free on this map, path/metrics agree, trajectory timestamps increasing), and saves static figures. Its loaders and drawing helpers are shared with `animate_robot.py`. |
| `python/animate_robot.py` | Builds a `FuncAnimation` from `trajectory.csv` and exports GIF (Pillow) and optionally MP4 (FFmpeg) with blitted rendering. Draws a static "No valid path" figure instead when there is no trajectory. |
| `python/run_demo.py` | One-command demo: runs `build/robot_simulator`, then the static figures and the animation. |
| `python/run_benchmark.py` | Benchmark sweep: seeded random maps (reusing `generate_map.generate_random`), one `robot_simulator --reference-bfs` process per run, raw results plus machine/build metadata. |
| `python/analyze_results.py` | `run`: recomputes path length, travel time and success of one run from the raw CSV files and cross-checks them with the simulator. `benchmark`: per-configuration statistics (median, mean, min, max, p95, counts) and the performance plots. |

The reference BFS stays in C++ (`bfsShortestPathCost`). The benchmark reaches it through
`--reference-bfs`, so Python never re-implements a planner. The BFS runs after
`planAStar()` returns, so it is not part of the measured planning time.

### Animation timing

`trajectory.csv` is the source of truth. With playback speed `p` (simulation seconds
per animation second) and frame rate `fps`, the moving frames show the simulation
times `linspace(t_first, t_last, n + 1)` with `n = ceil((t_last − t_first) · fps / p)`.
That is evenly spaced, at most `p / fps` apart, and ending exactly at the final sample.
The robot is drawn at `np.interp(t, time, x)`, `np.interp(t, time, y)`. Linear
interpolation is exact here because the robot moves in a straight line within each
timestep. The final frame is then held for 1.5 s and shows the final navigation status.
The default `p` makes the animation about 12 s long, but never slower than real time.

### Rendering performance

Measured on the 34 s obstacles run (203 frames at 80 dpi):

1. Constrained layout is computed once and then frozen
   (`fig.set_layout_engine("none")`), so it is not recomputed for every frame.
2. **Blitting** (`render_frames_blitted`): the static background (grid, axes, legend,
   planned path) is rendered once and saved. Each frame restores it and draws only the
   changing artists (robot, trail, labels) plus every artist that lies above them in
   z-order (the start/goal markers), in the same z-order a full render uses.
3. **Render once, write everywhere:** `CanvasPillowWriter` and `CanvasFFMpegWriter`
   take the frame straight from the canvas buffer instead of re-rendering through
   `savefig`, so one rendered frame feeds the GIF and the MP4 together.

With an interactive backend (`--show`), export falls back to the standard
`Animation.save()`. The output is unchanged: the GIF is byte-identical to the
non-blitted exporter and the decoded MP4 frames are identical. A test compares blitted
frames with full renders pixel by pixel.

| Export | Before (1) | After (1) | After (1)–(3) |
|---|---|---|---|
| GIF + MP4 | 175 s | 75 s | 10.8 s |
| GIF only | — | 24.8 s | 8.7 s (about 6 s of it is Pillow's GIF encoding) |

## Coordinate conventions

- Grid coordinate `(x, y)`: `x` = column, `y` = row; origin `(0, 0)` = upper-left cell.
- `+x` points right; `+y` points **down**, in both grid and world frames.
- World coordinates are meters at the cell center:
  `world_x = (grid_x + 0.5) * resolution`, `world_y = (grid_y + 0.5) * resolution`.
- In map files, the first line is row `y = 0`.
- Figures draw the grid with `y` increasing downward, plus secondary axes in meters.

## Build targets

| Target | Type | Purpose |
|---|---|---|
| `amr_core` | static library | all simulator logic |
| `robot_simulator` | executable | command-line interface |
| `bench_planner` | executable | planner micro-benchmark (`-DAMR_BUILD_BENCHMARKS=OFF` to skip) |
| `test_grid_map`, `test_astar`, `test_robot` | executables | unit tests (registered with CTest) |
| `python_visualization` | CTest entry | Python tests; registered only when a Python with NumPy, Matplotlib and Pillow is found (`.venv/` preferred) |
