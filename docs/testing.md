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
```

The tests use a small assertion framework in `tests/test_framework.hpp` (`CHECK`,
`CHECK_EQ`, `CHECK_NEAR`, `REQUIRE`) with no external dependencies. Floating-point
values are compared with explicit tolerances.

## CTest entries

| CTest name | What it runs |
|---|---|
| `test_grid_map` | map representation and parsing unit tests |
| `test_astar` | planner unit tests (TC01–TC11) |
| `cli_help` | `--help` exits with 0 |
| `cli_plan_obstacles_map` | plans on `maps/obstacles.csv`, expects `SUCCESS` |
| `cli_plan_maze_map` | plans on `maps/maze.csv`, expects `SUCCESS` |
| `cli_reports_unreachable_goal` | `tests/data/unreachable_goal.csv`, expects `NO_PATH` |
| `cli_rejects_occupied_start` | start inside an obstacle, expects `START_OCCUPIED` |
| `cli_rejects_malformed_map` | `tests/data/nonrectangular.csv`, expects a rectangularity error |
| `cli_rejects_missing_map` | nonexistent map, expects "does not exist" |
| `cli_rejects_bad_arguments` | non-integer coordinate, expects a nonzero exit code |

## Test cases

Phase status: TC12 (robot motion) and TC14 (trajectory consistency) belong to Phase 3
and are not implemented yet. The parts of TC03 and TC04 that involve the robot will
be added then.

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
| TC13 | Malformed map text: ragged rows, values `2`/`a`/`-1`, empty cells, trailing comma, empty input, comments only, bad resolution | controlled error message (with line/column) | `test_grid_map.cpp` |

Additional unit tests:

- Row-major parsing (first line = row 0), whitespace/comment/CRLF tolerance.
- Grid→world conversion at cell centers.
- Bounds and occupancy queries (out of bounds counts as occupied).
- `GridMap` constructor rejects bad dimensions, resolution, cell count and values.
- File loader: missing file, empty file, directory, malformed file, valid file.
- Every shipped map in `maps/` loads as 20×20 and connects its documented start/goal.
- Non-square maps (25×1, 1×17, 60×7).
- `isValidGridPath()` rejects occupied, skipped, diagonal and out-of-bounds steps.

## Validation methods

- **Optimality:** A* is compared against an independent breadth-first search
  (`bfsShortestPathCost`), which is optimal on uniform-cost graphs by construction.
- **Collision-freedom:** every successful plan is checked with `isValidGridPath()`
  (all cells free, consecutive cells 4-adjacent), and path length = cost + 1 waypoints.
- **Determinism:** repeated runs must match exactly, and the tie-break is pinned by an
  exact expected route in TC09.
- **Sensitivity check:** while developing Phase 2, the test suite was run against a
  deliberately broken planner that ignored obstacles; TC02, TC03, TC04 and TC11 failed
  as expected. This confirms the tests can detect a planner that cuts through walls.
