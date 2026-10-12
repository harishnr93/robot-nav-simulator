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
| `test_astar` | planner unit tests (TC01–TC11, planner reuse) |
| `test_robot` | motion and simulation unit tests (TC12, TC14, motion side of TC03–TC05) |
| `test_footprint` | robot footprint: disc geometry, inflation, footprint planning and disc collisions |
| `cli_help` | `--help` exits with 0 |
| `cli_plan_obstacles_map` | plans and simulates on `maps/obstacles.csv`, expects `GOAL_REACHED` |
| `cli_plan_maze_map` | plans and simulates on `maps/maze.csv`, expects `GOAL_REACHED` |
| `cli_reference_bfs_matches` | corridor map with `--reference-bfs`, expects `Reference BFS: 68 steps (matches A*)` |
| `cli_reference_bfs_no_path` | unreachable map with `--reference-bfs`, expects `Reference BFS: no path (matches A*)` |
| `cli_footprint_fine_map` | `maps/obstacles_fine.csv`, 0.25 m cells, `--robot-radius 0.2` with `--reference-bfs`: expects 45 steps matching BFS on the inflated map, then `GOAL_REACHED` |
| `cli_footprint_too_wide_for_gaps` | `maps/obstacles.csv` with `--robot-radius 0.3`: expects `NO_PATH`: on every route some passage has no 0.5 m cell center that is 0.3 m from both walls |
| `cli_footprint_start_too_close` | start next to an obstacle with `--robot-radius 0.3`: expects `START_TOO_CLOSE_TO_OBSTACLE` |
| `cli_rejects_negative_radius` | `--robot-radius -0.1`: expects "invalid motion parameters: robot radius" |
| `cli_navigation_timeout` | corridor map with `--max-time 2`, expects `TIMEOUT` |
| `cli_rejects_invalid_motion_parameters` | `--speed -1`, expects "invalid motion parameters" |
| `cli_reports_unreachable_goal` | `tests/data/unreachable_goal.csv`, expects `NO_PATH` and navigation `NOT_RUN` |
| `cli_rejects_occupied_start` | start inside an obstacle, expects `START_OCCUPIED` |
| `cli_rejects_malformed_map` | `tests/data/nonrectangular.csv`, expects a rectangularity error |
| `cli_rejects_missing_map` | nonexistent map, expects "does not exist" |
| `cli_rejects_bad_arguments` | non-integer coordinate, expects a nonzero exit code |
| `python_visualization` | `tests/test_python_visualization.py` (32 tests, about 10 s); only registered when CMake finds a Python with NumPy, Matplotlib and Pillow |

## Python visualization tests

`tests/test_python_visualization.py` uses `unittest` and runs the real `robot_simulator`
binary into temporary directories.

| Group | What is checked |
|---|---|
| Frame timing | frames start and end exactly at the first and last timestamps, are strictly increasing and at most `playback / fps` apart; real-time playback gives `fps` frames per second; zero-duration runs give one frame; the default playback speed targets about 12 s and is never slower than real time |
| Interpolation | the robot is exactly at recorded samples at their timestamps, linear in between, clamped after the end; the traveled trail ends at the robot; the target waypoint switches at the right sample |
| Result validation | non-increasing timestamps and missing columns are rejected |
| Animation integration | the robot in each frame is at the trajectory position for that frame's time, and in an independent check from the motion model, `x(t) = 0.75 + 0.5 t` on the first leg at a time *between* samples; the time label matches; the status reads `NAVIGATING …` and then `GOAL_REACHED` / `TIMEOUT` |
| Blitting | blitted frames are pixel-identical to full renders of the same state (first, middle and last frame) |
| Footprint | the inflated map is read from the simulator (372 inflated cells, obstacles preserved) and a mismatched one is rejected; the animated robot is a disc with the true radius, centred on the trajectory; blitted disc frames equal full renders; static figures and GIF are produced with a footprint; `run_demo.py` forwards `--robot-radius` |
| Export | the GIF is valid and looping with several frames; a failed plan produces `navigation_failed.png`, no GIF, and removes a stale GIF; MP4 without FFmpeg reports an error and still writes the GIF; a trajectory from another run is rejected |
| Analysis | the 95 % Wilson interval is correct (120/120, 10/20); summaries keep failed runs in counts, rates and planning-time statistics while path lengths use successful plans only; single-run metrics are recomputed from the raw CSV files (17.0 m, 34.0 s on the obstacles map) and a tampered `simulation_metrics.csv` is detected |
| Benchmark | benchmark maps are identical for the same seed and differ between seeds, with start and goal cleared; a 12-run benchmark runs end to end and writes every summary file and plot |
| `run_demo.py` | the simulator is called with an argument list (a map name containing spaces and `;` stays one argument); unset options are not passed, so the C++ defaults apply; animation runs only when requested; a missing executable gives exit code 127 |

## Robot footprint tests (`test_footprint.cpp`)

| Test | What is checked |
|---|---|
| Disc geometry | touching (distance = r) is free and overlapping is a collision, at a side neighbor (0.5 m), a diagonal corner (0.7071 m) and the map border |
| Swept disc is exact | a segment whose endpoints are both clear but which passes through an obstacle corner is a collision; a diagonal pass with the minimum distance (0.3536 m) at its midpoint is resolved exactly |
| Inflation rings | 1 m cells: `r = 0.5` inflates nothing (touching only), `r = 0.6` inflates the 4 side neighbors and the 16 border cells, `r = 0.75` everything |
| Monotonic inflation | on 50 random maps a larger radius never unblocks a cell |
| Planned paths are safe | 300 random maps, resolutions 0.25/0.5/1 m, radii 0.05–1.6 cells, start/goal where the disc fits. Every waypoint and every move passes the independent swept-disc check on the real map, and the simulation reaches the goal with 0 collisions (176 plans in the current sample) |
| Statuses and door widths | a 1 m door admits `r = 0.5` but not `r = 0.6`; `START_/GOAL_TOO_CLOSE_TO_OBSTACLE` for free cells where the disc does not fit; real obstacles keep `START_/GOAL_OCCUPIED` |
| Simulator disc collision | a point-robot path through the door, driven with `r = 0.6`, stops before the disc touches the door corners: within one 0.05 m step of the contact point `y = 3 − √(0.6² − 0.5²)` |
| Radius validation | negative, NaN and > 100 m radii are rejected |

**Point-robot equivalence.** With `--robot-radius 0` the new binary was compared with the
previous release on 400 random maps. `path.csv`, `trajectory.csv` and every previously
existing metrics column were identical, and no `inflated_map.csv` was written. The PNG
figures and the GIF produced by the Python scripts were byte-identical too.

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
- Planner reuse: one `AStarPlanner` plans 300 random maps of changing sizes (1–60 cells
  per side, some with blocked start/goal), and every status, cost, expansion count and
  path equals a fresh `planAStar()` call.
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
  | Reused planner skips resetting the previous search's state | 1 (planner reuse test) |
  | Blitted export redraws artists in the wrong z-order | 1 Python test (blitting test) |
  | Obstacle inflation disabled | 3 (footprint tests) |
  | Swept-disc check only looks at the segment endpoints | 1 |
  | Simulator ignores the robot radius | 1 |
  | Touching the obstacle counts as a collision | 5 |
  | Map border ignored by the footprint | 3 |

- **Benchmark-scale optimality check:** every one of the 480 benchmark runs
  ([performance.md](performance.md)) compares A* with the reference BFS through
  `--reference-bfs`. All 480 agreed: 388 equal costs and 92 runs where both found no path.
- **Optimization equivalence (Phase 6):** the optimized planner was compared with the
  previous implementation. A result checksum over 160 micro-benchmark maps is identical,
  and the old and new `robot_simulator` binaries produce byte-identical `path.csv`,
  `trajectory.csv` and non-timing metrics on 400 random maps
  ([performance.md](performance.md#verification)).

## Sanitizers and static analysis

Run in Phase 6 on the final code:

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all"
cmake --build build-asan -j && ctest --test-dir build-asan --output-on-failure
```

| Tool | Result |
|---|---|
| AddressSanitizer + UndefinedBehaviorSanitizer (Debug, asserts on) | all 16 CTest entries pass, no reports |
| `cppcheck` 2.7 (`--enable=warning,performance,portability`) | no findings |
| `clang-tidy` 14 (`bugprone-*`, `performance-*`, `clang-analyzer-*`) | 3 notes about string concatenation in CLI error messages (run only when reporting an error); left as is |
| `ruff` (pyflakes, pycodestyle, bugbear, pyupgrade, simplify) | no findings |
| GCC `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` | no warnings |

