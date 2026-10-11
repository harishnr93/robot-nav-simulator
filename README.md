# robot-nav-simulator

A lightweight 2D autonomous mobile robot navigation simulator built with C++17 and Python,
featuring A* path planning, occupancy-grid mapping, collision detection, trajectory
visualization, and performance analysis. Runs on an ordinary laptop: no robot hardware,
no ROS, no GPU.

> **Development status:** Phases 1–5 (map loading/validation, A* path planning,
> point-robot motion simulation, visualization/animation and benchmarking) are complete
> and tested. Publication polish is next. See [Roadmap](#roadmap). This README only
> documents features that exist today.

![Animated robot navigation on the obstacles map](docs/images/navigation_obstacles.gif)

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
- Robot animation (`matplotlib.animation.FuncAnimation`) driven by the recorded
  timestamps, with the planned route, traveled trajectory, simulation clock and
  navigation status. GIF export via Pillow; optional MP4 export when FFmpeg is installed.
- One-command demo launcher (`python/run_demo.py`) that runs the C++ simulator and all
  visualizations.
- Reproducible benchmark (`python/run_benchmark.py`): 6 grid sizes × 4 obstacle densities
  × 20 seeds. Failed runs are kept, and every run is checked against the reference BFS.
  Analysis (`python/analyze_results.py`) produces summary tables and performance plots.
- 35 C++ test cases, 12 end-to-end CLI checks and 26 Python tests, including A* vs. BFS
  optimality on 500 random maps, trajectory checks under 5 speed/timestep settings,
  animation timing checks and analysis checks.

## Architecture

```text
 Python  generate_map.py ──► maps/*.csv
                                 │
 C++     robot_simulator:  GridMap loader ──► A* planner ──► motion simulator ──► CSV writer
                                                                                   │
           results/path.csv, planning_metrics.csv, trajectory.csv, simulation_metrics.csv
                                                                                   │
 Python  visualize_map.py ◄──────────────────────────────────────────────────────┤
             └──► results/map_visualization.png, planned_path.png,                 │
                  trajectory.png, robot_position_vs_time.png                       │
         animate_robot.py ◄────────────────────────────────────────────────────────┘
             └──► results/navigation.gif (and optionally navigation.mp4)

         run_demo.py runs all of the above with one command.
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

Or use the helper scripts: `scripts/build.sh` and `scripts/test.sh`. The Python tests are
registered automatically when `.venv/` exists (or when the Python found by CMake has
NumPy, Matplotlib and Pillow); otherwise CMake prints that they were skipped.

## Quick demo

After installation and building, one command plans, simulates and renders everything:

```bash
python3 python/run_demo.py --save-gif
```

This uses `maps/obstacles.csv` from `(1, 1)` to `(18, 18)` and writes the CSV results,
four PNG figures and `results/navigation.gif` to `results/`. Add `--animate` to also
play the animation in a window.

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
| `--reference-bfs` | also run the reference BFS and check that A* is optimal | off |
| `--output DIR` | output directory | `results` |
| `-h`, `--help` | show help | |

Exit codes: `0` goal reached · `1` usage error · `2` map error · `3` invalid
start/goal · `4` no path · `5` output write error · `6` navigation failed (collision or
timeout) · `7` A* disagrees with the reference BFS (only with `--reference-bfs`).

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

### Animate the robot

```bash
python3 python/animate_robot.py --map maps/obstacles.csv --save-gif   # results/navigation.gif
python3 python/animate_robot.py --map maps/obstacles.csv --show       # play in a window
python3 python/animate_robot.py --map maps/obstacles.csv --save-mp4   # needs FFmpeg
```

| Option | Meaning | Default |
|---|---|---|
| `--results-dir DIR` | where the simulator wrote its CSV files | `results` |
| `--output-dir DIR` | where to save animations | `--results-dir` |
| `--save-gif` / `--save-mp4` / `--show` | outputs (GIF if none is given) | GIF |
| `--fps N` | animation frame rate | `15` |
| `--playback-speed X` | simulation seconds per animation second | about a 12 s animation, at least 1× |
| `--dpi N` | animation resolution | `80` (600 × 680 px) |

Each frame shows one simulation time; the robot is drawn at the trajectory position at
that time (interpolated between the recorded samples), so it moves at the simulated
speed rather than jumping between waypoints. If planning failed, nothing is animated:
`navigation_failed.png` shows the map with a "No valid path" message, and any
`navigation.gif`/`.mp4` from an earlier run is removed.

### Plan, simulate and visualize in one step

```bash
python3 python/run_demo.py --map maps/obstacles.csv --start 1 1 --goal 18 18 --animate --save-gif
python3 python/run_demo.py --map maps/maze.csv --start 1 1 --goal 19 19 --speed 1.0 --save-gif
scripts/run.sh maps/corridor.csv 1 1 18 18      # same as run_demo.py ... --save-gif
```

`run_demo.py` accepts the simulator's options (`--speed`, `--dt`, `--goal-tolerance`,
`--max-time`, `--resolution`, `--output`), the animation options `--animate`,
`--save-gif`, `--save-mp4`, `--fps` and `--playback-speed`, and `--simulator PATH`. It
starts the C++ executable with `subprocess` and an argument list, never through a shell.
Its exit code is the simulator's (for example `4` when there is no path).

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
`status,success,map_width,map_height,resolution,start_x,start_y,goal_x,goal_y,planning_time_ms,expanded_nodes,waypoint_count,path_length_steps,path_length_m,reference_bfs_cost`.
The two path-length fields are empty when planning fails. `reference_bfs_cost` is empty
unless `--reference-bfs` was given, and `-1` when the reference BFS found no path.

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

## Performance

Measured on an AMD Ryzen 7 4800H laptop (Ubuntu 22.04 under WSL2, GCC 11.4, Release
build): 480 random maps from 10 × 10 to 100 × 100 cells with 0–30 % obstacles, 20 seeds
per configuration, start and goal in opposite corners.

| Metric | Result |
|---|---|
| A* optimal (cost equal to reference BFS) | 480 / 480 runs |
| Median planning time, 100 × 100, successful plans | 0.11 ms (0 % obstacles) to 0.18 ms (20 %) |
| Slowest single plan | 1.79 ms |
| Expanded nodes, obstacle-free maps | exactly 2N − 1, the path cells only |
| Navigation success (all sizes) | 100 % / 95 % / 75 % / 53 % at 0 / 10 / 20 / 30 % obstacles |
| Collisions, timeouts | 0, 0 |

| | |
|---|---|
| ![Planning time vs. grid size](docs/images/benchmark/planning_time_vs_grid_size.png) | ![Success rate vs. obstacle density](docs/images/benchmark/success_rate_vs_obstacle_density.png) |

Failures are genuine `NO_PATH` results. 89 of the 92 come from a start or goal corner being sealed
off by its neighbors. For maps of this size, planning time is dominated by setting up the
per-cell search arrays, not by the search itself. Methodology, all plots, the full table
and the measurement limitations are in [docs/performance.md](docs/performance.md).

Reproduce (about 10 s):

```bash
python3 python/run_benchmark.py                        # -> results/benchmark/
python3 python/analyze_results.py run --results-dir results   # metrics of one run
```

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
- GIF export renders every frame with Matplotlib: about 25 s for the default
  ~200-frame animation on a laptop CPU. Lower `--fps` or `--dpi` for faster exports.

## Troubleshooting

| Problem | Fix |
|---|---|
| `simulator not found at .../build/robot_simulator` | build first: `cmake -S . -B build && cmake --build build -j` |
| `ImportError: Matplotlib requires numpy>=...` | use the project environment: `source .venv/bin/activate` (system packages may be outdated) |
| CMake says "Python tests skipped" | create `.venv` and install `requirements.txt`, then re-run `cmake -S . -B build` |
| `--show` / `--animate` opens no window | no display is available (e.g. SSH); use `--save-gif` instead |
| `MP4 export needs FFmpeg` | `sudo apt install ffmpeg`, or use GIF export |
| `results were produced for a ... map` / `path crosses obstacles` | the CSV files in `results/` belong to another map; re-run the simulator with this map |

## Roadmap

| Phase | Scope | Status |
|---|---|---|
| 1 | Repository, CMake, map loading & validation, sample maps | ✅ done |
| 2 | A* planner, path export, planner tests | ✅ done |
| 3 | Point-robot motion simulation, collision checks, `trajectory.csv` | ✅ done |
| 4 | Robot animation, GIF/MP4 export, one-command demo | ✅ done |
| 5 | Benchmarks, performance analysis and plots | ✅ done |
| 6 | Publication polish | pending |

Later extensions: 8-connected A*, differential-drive robot, PID control, simulated
LiDAR, EKF localization, dynamic obstacles, ROS 2 integration.

## License

MIT, see [LICENSE](LICENSE).
