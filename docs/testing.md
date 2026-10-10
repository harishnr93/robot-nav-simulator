# Testing

## How to run

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Each test executable can also be run directly for a per-case report:

```bash
./build/test_grid_map
./build/test_astar
./build/test_robot
.venv/bin/python tests/test_python_visualization.py
```

The tests use a small assertion framework in `tests/test_framework.hpp` (`CHECK`,
`CHECK_EQ`, `CHECK_NEAR`, `REQUIRE`) with no external dependencies. Floating-point
values are compared with explicit tolerances.

## CTest entries

| CTest name | What it runs |
|---|---|
| `test_grid_map` | map representation and parsing unit tests |
| `test_astar` | planner unit tests (TC01–TC11) |
| `test_robot` | motion and simulation unit tests (TC12, TC14, motion side of TC03–TC05) |
| `cli_help` | `--help` exits with 0 |
| `cli_plan_obstacles_map` | plans and simulates on `maps/obstacles.csv`, expects `GOAL_REACHED` |
| `cli_plan_maze_map` | plans and simulates on `maps/maze.csv`, expects `GOAL_REACHED` |
| `cli_navigation_timeout` | corridor map with `--max-time 2`, expects `TIMEOUT` |
| `cli_rejects_invalid_motion_parameters` | `--speed -1`, expects "invalid motion parameters" |
| `cli_reports_unreachable_goal` | `tests/data/unreachable_goal.csv`, expects `NO_PATH` and navigation `NOT_RUN` |
| `cli_rejects_occupied_start` | start inside an obstacle, expects `START_OCCUPIED` |
| `cli_rejects_malformed_map` | `tests/data/nonrectangular.csv`, expects a rectangularity error |
| `cli_rejects_missing_map` | nonexistent map, expects "does not exist" |
| `cli_rejects_bad_arguments` | non-integer coordinate, expects a nonzero exit code |
| `python_visualization` | `tests/test_python_visualization.py` (20 tests, about 7 s); only registered when CMake finds a Python with NumPy, Matplotlib and Pillow |

## Python visualization tests

`tests/test_python_visualization.py` uses `unittest` and runs the real `robot_simulator`
binary into temporary directories.

| Group | What is checked |
|---|---|
| Frame timing | frames start and end exactly at the first and last timestamps, are strictly increasing and at most `playback / fps` apart; real-time playback gives `fps` frames per second; zero-duration runs give one frame; the default playback speed targets about 12 s and is never slower than real time |
| Interpolation | the robot is exactly at recorded samples at their timestamps, linear in between, clamped after the end; the traveled trail ends at the robot; the target waypoint switches at the right sample |
| Result validation | non-increasing timestamps and missing columns are rejected |
| Animation integration | the robot in each frame is at the trajectory position for that frame's time, and in an independent check from the motion model, `x(t) = 0.75 + 0.5 t` on the first leg at a time *between* samples; the time label matches; the status reads `NAVIGATING …` and then `GOAL_REACHED` / `TIMEOUT` |
| Export | the GIF is valid and looping with several frames; a failed plan produces `navigation_failed.png`, no GIF, and removes a stale GIF; MP4 without FFmpeg reports an error and still writes the GIF; a trajectory from another run is rejected |
| `run_demo.py` | the simulator is called with an argument list (a map name containing spaces and `;` stays one argument); unset options are not passed, so the C++ defaults apply; animation runs only when requested; a missing executable gives exit code 127 |

## Test cases

All 14 specified test cases (TC01–TC14) are implemented. TC03, TC04 and TC05 are
tested in two parts: the planner side in `test_astar.cpp` and the motion side in
`test_robot.cpp`.

| ID | Scenario | Expected | Where |
|---|---|---|---|
| TC01 | Empty 10×10 map, (0,0)→(9,9) | success; cost = Manhattan distance (18); length 9.0 m; valid path; only path cells expanded | `test_astar.cpp` |
| TC02 | Single obstacle between start and goal | path avoids the obstacle, stays in bounds, cost 6 | `test_astar.cpp` |
| TC03 | Serpentine corridor with 1-cell gaps | path passes both gaps; cost 16 | `test_astar.cpp` |
| TC04 | Goal walled off; start sealed in | `NO_PATH`, empty path, BFS agrees | `test_astar.cpp` |
| TC05 | Start equals goal | exactly one waypoint, cost 0, length 0 m | `test_astar.cpp` |
| TC06 | Start outside the map (4 positions) | `START_OUT_OF_BOUNDS`, nothing expanded | `test_astar.cpp` |
| TC07 | Goal outside the map (3 positions) | `GOAL_OUT_OF_BOUNDS` | `test_astar.cpp` |
| TC08 | Start or goal on an obstacle | `START_OCCUPIED` / `GOAL_OCCUPIED` | `test_astar.cpp` |
| TC09 | 3×3 map with two equal-cost routes | cost 4; the documented tie-break selects the top-right route | `test_astar.cpp` |
| TC10 | Same random 30×30 map planned 6 times | identical status, cost, expansions and path | `test_astar.cpp` |
| TC11 | 500 random maps (1–40 cells per side, 0–45 % obstacles) | A* cost = BFS cost when reachable; `NO_PATH` exactly when BFS fails; both outcomes occur | `test_astar.cpp` |
| TC03 (motion) | Robot follows the corridor plan | `GOAL_REACHED`, no collision, every sample in a free cell, no overshoot | `test_robot.cpp` |
| TC04 (motion) | Planning failed | no trajectory: result is `NOT_RUN` with no samples; simulating the empty path gives `INVALID_PATH`, no samples | `test_robot.cpp`, `cli_reports_unreachable_goal` |
| TC05 (motion) | Start equals goal | `GOAL_REACHED` with 0 steps, one sample, 0 s, 0 m | `test_robot.cpp` |
| TC12 | Robot follows a 34-step path on a 20×20 obstacle map with default settings | `GOAL_REACHED`, goal error ≤ tolerance, every sample on the segment toward its target (no overshoot), all waypoints visited in order, exactly 340 steps / 34.0 s / 17.0 m, ends on the goal center | `test_robot.cpp` |
| TC13 | Malformed map text: ragged rows, values `2`/`a`/`-1`, empty cells, trailing comma, empty input, comments only, bad resolution | controlled error message (with line/column) | `test_grid_map.cpp` |
| TC14 | Same map, 5 settings: (0.5 m/s, 0.1 s), (0.7, 0.13), (0.3, 0.05), (2.0, 1.0: step longer than a cell), (1.0, 0.017) | timestamps start at 0 and rise by exactly `dt`; speed between samples ≤ configured speed; all values finite; samples = steps + 1; travel time = steps × dt; no overshoot; 6 steps per cell for 0.091 m steps | `test_robot.cpp` |

Additional unit tests:

- Row-major parsing (first line = row 0), whitespace/comment/CRLF tolerance.
- Grid→world conversion at cell centers; world→grid conversion, boundaries, out-of-map
  and non-finite points, and a round trip over every cell.
- Bounds and occupancy queries (out of bounds counts as occupied).
- `GridMap` constructor rejects bad dimensions, resolution, cell count and values.
- File loader: missing file, empty file, directory, malformed file, valid file.
- Every shipped map in `maps/` loads as 20×20 and connects its documented start/goal.
- Non-square maps (25×1, 1×17, 60×7).
- `isValidGridPath()` rejects occupied, skipped, diagonal and out-of-bounds steps.
- `stepToward()`: normal, diagonal (3-4-5), clamped and zero-distance steps.
- Motion configuration validation (non-positive speed/dt, negative tolerance, NaN,
  too many steps); invalid configuration is rejected by the simulator.
- Segment collision check: free, into an obstacle, diagonal across an obstacle, leaving
  the map, zero-length, inside an obstacle.
- Collision during motion: a path through an obstacle stops the robot at its last safe
  pose (x = 0.95 m, before the boundary at 1.0 m); starting inside an obstacle is
  reported at once.
- Timeout: `max_time` 5 s on a 19 s path stops after exactly 50 steps.
- Malformed paths (empty, jump, diagonal, out of bounds) give `INVALID_PATH`.

## Validation methods

- **Optimality:** A* is compared against an independent breadth-first search
  (`bfsShortestPathCost`), which is optimal on uniform-cost graphs by construction.
- **Collision-freedom:** every successful plan is checked with `isValidGridPath()`
  (all cells free, consecutive cells 4-adjacent), and path length = cost + 1 waypoints.
- **Determinism:** repeated runs must match exactly, and the tie-break is pinned by an
  exact expected route in TC09.
- **Trajectory physics:** every simulation test checks timestamps, speed limit,
  finiteness and the no-overshoot property (each sample lies on the segment toward its
  target waypoint).
- **Sensitivity checks (mutation testing):** the suites were run against deliberately
  broken code to confirm they detect real bugs. Results:

  | Broken variant | Failing test cases |
  |---|---|
  | Planner ignores obstacles (Phase 2) | 4 (TC02, TC03, TC04, TC11) |
  | Motion step not clamped (overshoot) | 3 |
  | Collision check disabled | 2 |
  | Timeout disabled | 1 |
  | Timestamps doubled | 3 |
  | Animation snaps the robot to the previous sample instead of interpolating (a "jumping" robot) | 2 Python tests (interpolation unit test and animation integration test) |
