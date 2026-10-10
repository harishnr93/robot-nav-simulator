# Architecture

This document describes the modules that exist after Phases 1–3 and how data flows
between them. Later phases (animation, benchmarking) will extend this document as they
are implemented.

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
                                        └──────────────────────────────────────────┘
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
| `include/astar.hpp`, `src/astar.cpp` | Request validation, A* search, path reconstruction, path validity check, path length, and a reference BFS used only by tests. |
| `include/robot.hpp`, `src/robot.cpp` | `RobotState`, `MotionConfig` (+ validation), and `stepToward()`: one clamped kinematic step of the point robot. |
| `include/simulator.hpp`, `src/simulator.cpp` | `simulatePath()`: time-stepped waypoint following, segment collision checking, timeout, trajectory recording; `NavigationStatus`, `SimulationResult`. |
| `include/csv_writer.hpp`, `src/csv_writer.cpp` | Writes `path.csv`, `planning_metrics.csv`, `trajectory.csv` and `simulation_metrics.csv`. |
| `src/main.cpp` | CLI parsing, orchestration, console summary, exit codes. |

### Key data structures

- **`GridMap`**: `width × height` cells stored row-major (`index = y * width + x`).
  Out-of-bounds queries count as occupied, which makes neighbor checks in the planner a
  single `isFree()` call.
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
| Invalid motion parameters (`--speed`, `--dt`, ...) | rejected during argument parsing | 1 |

All four result files are written for every planning outcome (codes 0, 3, 4, 6). Files
without data contain only their header, so results from an earlier run can never be
mistaken for the current one. The simulator only runs when planning succeeded.

## Python modules

| File | Responsibility |
|---|---|
| `python/generate_map.py` | Creates `empty`, `obstacles`, `corridor`, `maze` and `random` maps. Seeded randomness; optional reachability guarantee via flood fill (a connectivity check, not a planner). |
| `python/visualize_map.py` | Loads a map and the C++ results, checks they belong together (size, path free on this map, path/metrics agree, trajectory timestamps increasing), and saves static figures. Its loaders are reused by later phases. |

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
| `test_grid_map`, `test_astar`, `test_robot` | executables | unit tests (registered with CTest) |
