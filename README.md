# robot-nav-simulator

A lightweight 2D autonomous mobile robot navigation simulator built with C++17 and Python,
featuring A* path planning, occupancy-grid mapping, collision detection, trajectory
visualization, and performance analysis. Runs on an ordinary laptop: no robot hardware,
no ROS, no GPU.

> **Development status:** Phases 1–3 (map loading/validation, A* path planning and
> point-robot motion simulation) are complete and tested. Animation and benchmarking are
> being added phase by phase. See [Roadmap](#roadmap). This README only documents
> features that exist today.

| Planned path | Planned path vs. robot trajectory |
|---|---|
| ![Planned A* path on the obstacles map](docs/images/planned_path_obstacles.png) | ![Robot trajectory on the obstacles map](docs/images/trajectory_obstacles.png) |

## Features (implemented)

- Occupancy-grid maps loaded from CSV, with validation of every input (missing/empty
  files, non-rectangular grids, unsupported cell values, invalid dimensions).
- A* path planner written from scratch: 4-connected grid, uniform step cost,
  Manhattan heuristic, `std::priority_queue` with lazy deletion, **deterministic
  tie-breaking**.
- Explicit planner status: `SUCCESS`, `NO_PATH`, `START_OUT_OF_BOUNDS`,
  `GOAL_OUT_OF_BOUNDS`, `START_OCCUPIED`, `GOAL_OCCUPIED`. A failed plan never returns a
  fabricated path.
- Planning metrics: time (steady clock, search only), expanded nodes, path length in
  steps and meters.
- Point-robot motion simulation: constant speed, fixed timestep, step clamped so the
  robot never overshoots a waypoint, time limit against endless runs.
- Motion-time collision checking of every movement segment against the occupancy grid.
- Navigation status: `GOAL_REACHED`, `COLLISION`, `TIMEOUT`, `INVALID_PATH`,
  `INVALID_CONFIG`, or `NOT_RUN` when planning failed.
- CSV export (`path.csv`, `planning_metrics.csv`, `trajectory.csv`,
  `simulation_metrics.csv`) for Python analysis.
- Python map generator: empty, fixed obstacles, corridor, maze and seeded random maps,
  with optional reachability guarantee.
- Python static visualization of the map, planned path, actual robot trajectory and
  robot position over time (including a clear "No valid path" figure on failure).
- 35 C++ test cases plus 10 end-to-end CLI checks, including A* vs. BFS optimality on
  500 random maps and trajectory checks under 5 speed/timestep settings.

## Architecture

```text
 Python  generate_map.py ──► maps/*.csv
                                 │
 C++     robot_simulator:  GridMap loader ──► A* planner ──► motion simulator ──► CSV writer
                                                                                   │
           results/path.csv, planning_metrics.csv, trajectory.csv, simulation_metrics.csv
                                                                                   │
 Python  visualize_map.py ◄────────────────────────────────────────────────────────┘
             └──► results/map_visualization.png, planned_path.png,
                  trajectory.png, robot_position_vs_time.png
```

The C++ planner is the single authoritative planning implementation; Python only
generates maps and visualizes results. Details: [docs/architecture.md](docs/architecture.md).

### Coordinate conventions

| Item | Convention |
|---|---|
| Grid coordinate | `(x, y)`: `x` = column, `y` = row |
| Origin | upper-left cell `(0, 0)` |
| Axes | `+x` right, `+y` **down** (grid and world) |
| Cell → world (m) | `world = (grid + 0.5) * resolution` (cell center) |
| Default resolution | 0.5 m/cell (20 × 20 map = 10 m × 10 m) |

## Requirements

- Ubuntu 24.04 LTS (also verified on 22.04 with GCC 11.4 / CMake 3.22)
- GCC/G++ with C++17, CMake ≥ 3.16
- Python 3.10+ with NumPy, Matplotlib (≥ 3.7), Pillow

## Installation

```bash
sudo apt update
sudo apt install build-essential cmake python3 python3-venv git

git clone https://github.com/harishnr93/robot-nav-simulator.git
cd robot-nav-simulator

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Build and test

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Or use the helper scripts: `scripts/build.sh` and `scripts/test.sh`.

## Usage

### Plan a path and simulate the robot

```bash
./build/robot_simulator --map maps/obstacles.csv --start 1 1 --goal 18 18 --output results/
```

```text
Map:            maps/obstacles.csv (20 x 20 cells, 0.50 m/cell)
Start:          cell (1, 1), world (0.75, 0.75) m
Goal:           cell (18, 18), world (9.25, 9.25) m
Planner status: SUCCESS
Expanded nodes: 38
Planning time:  0.008 ms
Path:           35 waypoints, 34 steps, 17.00 m
Navigation:     GOAL_REACHED
Motion:         340 steps of 0.10 s at 0.50 m/s, travel time 34.00 s, distance 17.00 m
Goal error:     0.0000 m (tolerance 0.0500 m), collisions 0
Results:        results/
```

(The planning time depends on your machine.) A run counts as a navigation success only
if planning succeeds and the robot reaches the goal within tolerance with no collision
and no timeout.

| Option | Meaning | Default |
|---|---|---|
| `--map FILE` | map CSV (required) | |
| `--start X Y` | start cell (required) | |
| `--goal X Y` | goal cell (required) | |
| `--resolution M` | meters per cell | `0.5` |
| `--speed V` | robot speed, m/s | `0.5` |
| `--dt S` | simulation timestep, s | `0.1` |
| `--goal-tolerance M` | allowed final distance to the goal center, m | `0.05` |
| `--max-time S` | simulation time limit, s | `600` |
| `--output DIR` | output directory | `results` |
| `-h`, `--help` | show help | |

Exit codes: `0` goal reached · `1` usage error · `2` map error · `3` invalid
start/goal · `4` no path · `5` output write error · `6` navigation failed (collision or
timeout).

The speed, timestep and tolerance defaults are simulation settings, not measured
hardware specifications.

### Visualize the result

```bash
python3 python/visualize_map.py --map maps/obstacles.csv
# -> results/map_visualization.png, results/planned_path.png,
#    results/trajectory.png, results/robot_position_vs_time.png
```

The trajectory figures are skipped when there is no trajectory (planning failed).

`visualize_map.py` refuses to plot results that do not belong to the given map
(different size, or a path that crosses its obstacles).

### Simulate and visualize in one step

```bash
scripts/run.sh maps/corridor.csv 1 1 18 18
```

### Sample maps

| Map | Suggested start → goal |
|---|---|
| `maps/empty.csv` | `1 1` → `18 18` |
| `maps/obstacles.csv` | `1 1` → `18 18` |
| `maps/corridor.csv` | `1 1` → `18 18` |
| `maps/maze.csv` | `1 1` → `19 19` |

### Generate maps

```bash
python3 python/generate_map.py --type maze --seed 3 --goal 19 19 --output maps/maze.csv
python3 python/generate_map.py --type random --width 40 --height 40 \
    --density 0.3 --seed 7 --ensure-reachable --output maps/random_40.csv
```

Types: `empty`, `obstacles`, `corridor`, `maze`, `random`. Random maps are reproducible
for a given `--seed`. A path is **not** guaranteed unless `--ensure-reachable` is used.

## File formats

**Map CSV**: one row per line, comma-separated `0` (free) / `1` (occupied). The first
line is row `y = 0`. Blank lines and lines starting with `#` are ignored.

**`path.csv`**: `waypoint_index,grid_x,grid_y,world_x,world_y`. Includes start and
goal. Header only when planning fails.

**`planning_metrics.csv`**: one row with
`status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m`.
The two path-length fields are empty when planning fails.

**`trajectory.csv`**: `time,x,y,target_waypoint_index`. One row per timestep, from
`t = 0` (start-cell center) to the final position; `x`, `y` in meters.
`target_waypoint_index` is the waypoint the robot is heading to after that sample (the
last index once the goal is reached). Header only when planning fails.

**`simulation_metrics.csv`**: one row with
`status,success,navigation_success,steps,travel_time_s,distance_traveled_m,final_goal_error_m,collision_count,speed_mps,dt_s,goal_tolerance_m,max_time_s`.
Travel time is the final timestamp minus the initial timestamp. The distance, time and
error fields are empty when the simulation did not run.

## Algorithm summary

A* expands cells in order of `f(n) = g(n) + h(n)`, where `g` is the number of steps from
the start and `h(n) = |x_goal − x_n| + |y_goal − y_n|`. The Manhattan heuristic never
overestimates the remaining cost on a 4-connected grid, so the first time the goal is
expanded its cost is optimal. Ties are broken by lowest `f`, then lowest `h`, then
lowest `y`, then lowest `x`, so results are reproducible.

The robot then follows the waypoints at constant speed. Each timestep it moves
`min(speed × dt, distance to the next waypoint)` toward that waypoint, so it never
overshoots, and every movement segment is checked against the occupancy grid. Full
explanation: [docs/algorithms.md](docs/algorithms.md).

## Testing

See [docs/testing.md](docs/testing.md) for every test case and what it checks.

## Known limitations

- Point robot: no physical footprint, orientation or dynamics, so paths and the
  trajectory may run right next to obstacles. The collision check does not account for
  a robot radius.
- Leftover step distance is not carried over to the next waypoint. When `speed × dt`
  does not divide the cell size, the robot loses part of a timestep at every waypoint,
  so travel time is slightly longer than `path length / speed`.
- 4-connected movement only; paths are optimal in grid steps, not in Euclidean distance.
- Static maps only.

## Roadmap

| Phase | Scope | Status |
|---|---|---|
| 1 | Repository, CMake, map loading & validation, sample maps | ✅ done |
| 2 | A* planner, path export, planner tests | ✅ done |
| 3 | Point-robot motion simulation, collision checks, `trajectory.csv` | ✅ done |
| 4 | Robot animation and GIF export | pending |
| 5 | Benchmarks and performance plots | pending |
| 6 | Publication polish | pending |

Later extensions: 8-connected A*, differential-drive robot, PID control, simulated
LiDAR, EKF localization, dynamic obstacles, ROS 2 integration.

## License

MIT, see [LICENSE](LICENSE).
