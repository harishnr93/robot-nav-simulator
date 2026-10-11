# Performance

This document describes how the planner and the navigation simulation were benchmarked,
what was measured, and what the measurements show. Every number and figure here comes
from the benchmark run recorded in [`docs/benchmark/`](benchmark/). Nothing was
estimated or synthesized.

## How to reproduce

```bash
cmake -S . -B build && cmake --build build -j     # Release build (the default)
python3 python/run_benchmark.py                    # 480 runs, about 10 s
# -> results/benchmark/benchmark_runs.csv, benchmark_metadata.json,
#    benchmark_summary.csv/.md and the plots below
```

`python3 python/analyze_results.py benchmark` regenerates the summary and plots from an
existing `benchmark_runs.csv`. `python3 python/analyze_results.py run` analyzes a single
simulator run in `results/`.

## Methodology

| Item | Setting |
|---|---|
| Grid sizes N | 10, 20, 40, 60, 80, 100 (maps of N × N cells, 0.5 m per cell) |
| Obstacle densities | 0 %, 10 %, 20 %, 30 % (each cell occupied independently with that probability) |
| Seeds | 20 per (N, density): **480 runs** in total |
| Map seeding | `numpy.random.default_rng([seed, N, round(100·density)])`, so every map is reproducible |
| Start, goal | opposite corners `(0, 0)` → `(N−1, N−1)`; only these two cells are forced free |
| Reachability | **not** guaranteed; unreachable goals are recorded as failures, never discarded |
| Motion | 0.5 m/s, dt 0.1 s, goal tolerance 0.05 m, time limit 5000 s (raised from the CLI default of 600 s so long paths on large maps do not time out) |
| Optimality check | every run uses `--reference-bfs`: the C++ reference BFS cost is compared with the A* cost |
| Execution | one run at a time (no parallelism), each in a fresh `robot_simulator` process |

**What "planning time" measures.** `std::chrono::steady_clock` around `planAStar()`
inside the C++ process: request validation, allocation of the per-cell search arrays,
and the search itself. It excludes process start-up, map loading, the BFS check, motion
simulation and file output. Each run times a single planning call.

**Navigation success** means planning succeeded *and* the robot reached the goal
within tolerance with no collision and no timeout. Every run counts in the success
rates, including failed ones.

### Test machine

| | |
|---|---|
| CPU | AMD Ryzen 7 4800H (16 logical CPUs) |
| OS | Ubuntu 22.04 under WSL2 (Linux 6.6.87.2-microsoft-standard-WSL2) |
| Compiler | GCC 11.4.0, CMake build type Release (`-O3`) |
| Python | 3.10.12, NumPy 2.2.6, Matplotlib 3.10.9 |
| Date | 2026-10-10 |

The data was collected with the Phase 5 code before it was committed, so
`benchmark_metadata.json` records the commit as `e83bcb5-dirty`.

## Results

### Correctness

- **A* was optimal in 480 of 480 runs.** On all 388 reachable maps, its path cost equaled the reference
  BFS cost. On all 92 unreachable maps, both reported that no path exists.
- **0 collisions and 0 timeouts.** Navigation succeeded in exactly the runs where planning
  succeeded (388 of 480).
- For every successful run, the measured travel time equals `path length / speed`
  exactly. That is expected: with 0.05 m steps a 0.5 m cell takes exactly 10 steps (see
  [algorithms.md](algorithms.md#6-waypoint-tracking-point-robot-motion)).

### Planning time

![Planning time vs. grid size](images/benchmark/planning_time_vs_grid_size.png)

Median over successful plans (bars: 25th–75th percentile):

| N | 0 % | 10 % | 20 % | 30 % |
|---|---|---|---|---|
| 10 | 0.0034 ms | 0.0038 ms | 0.0045 ms | 0.0046 ms |
| 20 | 0.0072 ms | 0.0098 ms | 0.0112 ms | 0.0111 ms |
| 40 | 0.0191 ms | 0.0381 ms | 0.0315 ms | 0.0405 ms |
| 60 | 0.0382 ms | 0.0568 ms | 0.0689 ms | 0.0802 ms |
| 80 | 0.0868 ms | 0.1337 ms | 0.1368 ms | 0.1366 ms |
| 100 | 0.1066 ms | 0.1691 ms | 0.1844 ms | 0.1712 ms |

- Planning is fast for these map sizes. The slowest single run was 1.79 ms (N = 100,
  10 % density). All other runs took under 1.4 ms.
- **The fixed per-request set-up dominates.** A* allocates and initializes three arrays
  with one entry per cell on every request (cost so far, parent, closed). Runs
  where the start or goal was sealed in, so the search expanded at most 5 nodes, show this cost
  directly. They still took a median of 0.0011 ms at N = 10 and **0.0975 ms at N = 100**,
  which is about 6–11 ns per cell. At N = 100 that is 91 % of the time of a complete obstacle-free
  plan (0.1066 ms). A natural optimization is to reuse these buffers between requests
  (not implemented).
- Obstacles add 1.5–2.1× to the median time at N ≥ 40, because the search has to explore around them
  (see expanded nodes below).

### Expanded nodes

![Expanded nodes vs. grid size](images/benchmark/expanded_nodes_vs_grid_size.png)

- **Obstacle-free maps: exactly 2N − 1 expansions** (19, 39, 79, …, 199). That is just
  the cells on the final path, thanks to the (f, h, y, x) tie-break (also asserted by
  test TC01).
- With obstacles, the median rises to 1.3–4.1× the 2N − 1 cells of a minimal path, but stays far
  below the N² cells of the map. At N = 100 the median is 622–812 nodes, 6–8 % of the 10 000
  cells.

### Path length

![Path length vs. grid size](images/benchmark/path_length_vs_grid_size.png)

- **92 % of successful plans have the obstacle-free minimum length** `2(N − 1) × 0.5 m`.
  On a 4-connected grid, many different shortest paths exist between opposite corners, so
  random obstacles rarely force a detour.
- Only at 30 % density does the median detour become visible: 2.6 % (N = 40) and 1.7 % (N = 60).
  The largest detour over all 388 plans was 15 %.

### Navigation success rate

![Navigation success rate vs. obstacle density](images/benchmark/success_rate_vs_obstacle_density.png)

Pooled over all six grid sizes (120 maps per density; 95 % Wilson confidence interval):

| Density | Successful runs | Success rate | 95 % CI |
|---|---|---|---|
| 0 % | 120 / 120 | 100.0 % | 96.9–100.0 % |
| 10 % | 114 / 120 | 95.0 % | 89.5–97.7 % |
| 20 % | 90 / 120 | 75.0 % | 66.6–81.9 % |
| 30 % | 64 / 120 | 53.3 % | 44.4–62.0 % |

**Why runs fail.** All 92 failures are genuine `NO_PATH` results. Analyzing the failed
maps shows that **89 of 92 failures are caused by the start or the goal corner being sealed
into a small pocket** (median 2 free cells, at most 59). Only 3 maps were split into two
large regions. A corner cell has only two neighbors, so for density d the chance that both
neighbors of one corner are occupied is already d² (9 % at 30 %). Larger pockets add to
this. This is why the success rate depends mostly on the density and hardly on the grid size: the
per-size lines (20 maps each) scatter around the pooled line within their much larger
sampling uncertainty (about ±20 percentage points at n = 20).

### Robot motion (single run)

![Robot position vs. time](images/benchmark/robot_position_vs_time.png)

`maps/obstacles.csv`, (1, 1) → (18, 18): 34 grid steps, 17.0 m, 340 timesteps,
travel time 34.0 s, final goal error 0.0 m, 0 collisions. The sloped segments have a
constant 0.5 m/s slope. A flat segment in x(t) or y(t) means the robot is moving along the other
axis at that time.

## Full summary (all runs)

Planning-time statistics here include **all** runs, failed searches too. That is why
some minimum times at high density are below the obstacle-free time: a sealed-in start
ends the search after one or two expansions. "nav. success" counts runs that reached the
goal. "A* = BFS" counts runs where A* and the reference BFS agreed.

| N | density | runs | nav. success | median ms | mean ms | min ms | max ms | p95 ms | median expanded | median path (m) | A* = BFS |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 0% | 20 | 20/20 | 0.0034 | 0.0034 | 0.0032 | 0.0038 | 0.0037 | 19 | 9.0 | 20/20 |
| 10 | 10% | 20 | 20/20 | 0.0038 | 0.0042 | 0.0031 | 0.0091 | 0.0051 | 19 | 9.0 | 20/20 |
| 10 | 20% | 20 | 16/20 | 0.0046 | 0.0053 | 0.0015 | 0.0108 | 0.0106 | 25 | 9.0 | 20/20 |
| 10 | 30% | 20 | 7/20 | 0.0052 | 0.0051 | 0.0009 | 0.0097 | 0.0095 | 29 | 9.0 | 20/20 |
| 20 | 0% | 20 | 20/20 | 0.0072 | 0.0073 | 0.0069 | 0.0086 | 0.0082 | 39 | 19.0 | 20/20 |
| 20 | 10% | 20 | 20/20 | 0.0098 | 0.0104 | 0.0081 | 0.0163 | 0.0137 | 52 | 19.0 | 20/20 |
| 20 | 20% | 20 | 13/20 | 0.0112 | 0.0146 | 0.0033 | 0.0398 | 0.0392 | 60 | 19.0 | 20/20 |
| 20 | 30% | 20 | 12/20 | 0.0107 | 0.0116 | 0.0034 | 0.0329 | 0.0256 | 58 | 19.0 | 20/20 |
| 40 | 0% | 20 | 20/20 | 0.0191 | 0.0213 | 0.0178 | 0.0427 | 0.0400 | 79 | 39.0 | 20/20 |
| 40 | 10% | 20 | 19/20 | 0.0362 | 0.0330 | 0.0103 | 0.0440 | 0.0426 | 158 | 39.0 | 20/20 |
| 40 | 20% | 20 | 13/20 | 0.0312 | 0.0570 | 0.0097 | 0.2732 | 0.1860 | 154 | 39.0 | 20/20 |
| 40 | 30% | 20 | 15/20 | 0.0371 | 0.0509 | 0.0093 | 0.1357 | 0.1227 | 224 | 40.0 | 20/20 |
| 60 | 0% | 20 | 20/20 | 0.0382 | 0.0407 | 0.0350 | 0.0693 | 0.0551 | 119 | 59.0 | 20/20 |
| 60 | 10% | 20 | 19/20 | 0.0572 | 0.0888 | 0.0443 | 0.6216 | 0.1254 | 232 | 59.0 | 20/20 |
| 60 | 20% | 20 | 18/20 | 0.0682 | 0.0869 | 0.0276 | 0.4417 | 0.1215 | 284 | 59.0 | 20/20 |
| 60 | 30% | 20 | 11/20 | 0.0846 | 0.1460 | 0.0278 | 0.3540 | 0.3362 | 522 | 60.0 | 20/20 |
| 80 | 0% | 20 | 20/20 | 0.0868 | 0.0914 | 0.0781 | 0.1219 | 0.1192 | 159 | 79.0 | 20/20 |
| 80 | 10% | 20 | 18/20 | 0.1337 | 0.1871 | 0.0652 | 1.1074 | 0.2640 | 416 | 79.0 | 20/20 |
| 80 | 20% | 20 | 15/20 | 0.1377 | 0.2440 | 0.0668 | 0.8985 | 0.8228 | 539 | 79.0 | 20/20 |
| 80 | 30% | 20 | 9/20 | 0.1596 | 0.2763 | 0.0674 | 0.6258 | 0.6170 | 577 | 79.0 | 20/20 |
| 100 | 0% | 20 | 20/20 | 0.1066 | 0.1091 | 0.0996 | 0.1281 | 0.1214 | 199 | 99.0 | 20/20 |
| 100 | 10% | 20 | 18/20 | 0.1691 | 0.2465 | 0.0913 | 1.7904 | 0.3152 | 622 | 99.0 | 20/20 |
| 100 | 20% | 20 | 15/20 | 0.1769 | 0.2897 | 0.0831 | 1.3328 | 1.3128 | 791 | 99.0 | 20/20 |
| 100 | 30% | 20 | 10/20 | 0.1712 | 0.3622 | 0.0929 | 0.9705 | 0.9573 | 713 | 99.5 | 20/20 |

Raw per-run data: [`benchmark/benchmark_runs.csv`](benchmark/benchmark_runs.csv).
Machine and parameters: [`benchmark/benchmark_metadata.json`](benchmark/benchmark_metadata.json).

## Measurement limitations

- **Single cold call per run.** Each process plans once, so the time includes first-touch
  page faults and cold caches. On identical obstacle-free maps the spread was
  0.0032–0.0038 ms (N = 10) and 0.0996–0.1281 ms (N = 100). Medians over 20 runs are
  stable, but individual values can vary by tens of percent.
- **Microsecond scale.** At N = 10 a plan takes about 3–5 µs, close to the overhead of
  reading the clock. Treat small-map timings as orders of magnitude, not exact values.
- **WSL2 virtual machine on a laptop.** CPU frequency scaling, the hypervisor and background
  processes add noise. Absolute times will differ on other machines; the trends
  (growth with N, effect of density) are what carry over.
- **Benchmark design.** Corner start/goal and independent random obstacles make sealed-in
  corners the main failure mode. Other layouts (corridors, rooms, a start/goal
  in open space) would give different success rates.
- **Scope.** 4-connected moves and a point robot. No obstacle inflation, no dynamics.
