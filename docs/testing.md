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

The ROS 2 package has its own 21 tests (gtest and launch_testing), run with
`scripts/build_ros2.sh --test`. They are listed in [ros2.md](ros2.md#tests).

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
| `test_octile` | 8-connected A*: exact costs, corner rule, optimality against exact Dijkstra, diagonal motion |
| `test_diff_drive` | differential drive: unicycle kinematics, P heading controller, turn-then-drive follower |
| `test_heading_control` | P vs. PID study: heading plant, controllers, metrics |
| `test_lidar` | simulated LiDAR: exact ray casting, beam layout, noise, scans along a trajectory |
| `test_ekf` | EKF localization: odometry model, Jacobians, Kalman update, gate, convergence, NEES consistency |
| `test_correlated_errors` | correlated range errors: LiDAR bias process, augmented EKF (bias and wall states), wall-line surface, grazing-beam model |
| `test_obstacle_tracking` | moving obstacles from the LiDAR alone: map subtraction, disc fitting, fragment merging, tracking through reversals, learned turning points, two-point start, negative evidence and coasting, LiDAR perception in the simulator, occlusion awareness |
| `test_dynamic` | moving obstacles and replanning: motion model, exact contact, loading, visibility, waiting, detours, retreats, invariants |
| `test_ghost_cells` | minimum ghost cells: covers vs. inflation, exact solver vs. brute force (count, every optimal set, weighted), overlapping footprints, statuses |
| `test_ghost_perception` | perception loop: beam and ghost model, occupancy update, ghost evidence, recovery strategies on a doorway |
| `test_alignment` | grid alignment: geometry, rasterization rules, placement, exact LiDAR, point evidence, the alignment-robust planner, ground truth, artifact vs. ghost recovery |
| `cli_help` | `--help` exits with 0 |
| `cli_plan_obstacles_map` | plans and simulates on `maps/obstacles.csv`, expects `GOAL_REACHED` |
| `cli_plan_maze_map` | plans and simulates on `maps/maze.csv`, expects `GOAL_REACHED` |
| `cli_reference_bfs_matches` | corridor map with `--reference-bfs`, expects `Reference BFS: 68 steps (matches A*)` |
| `cli_reference_bfs_no_path` | unreachable map with `--reference-bfs`, expects `Reference BFS: no path (matches A*)` |
| `cli_footprint_fine_map` | `maps/obstacles_fine.csv`, 0.25 m cells, `--robot-radius 0.2` with `--reference-bfs`: expects 45 steps matching BFS on the inflated map, then `GOAL_REACHED` |
| `cli_footprint_too_wide_for_gaps` | `maps/obstacles.csv` with `--robot-radius 0.3`: expects `NO_PATH`: on every route some passage has no 0.5 m cell center that is 0.3 m from both walls |
| `cli_footprint_start_too_close` | start next to an obstacle with `--robot-radius 0.3`: expects `START_TOO_CLOSE_TO_OBSTACLE` |
| `cli_rejects_negative_radius` | `--robot-radius -0.1`: expects "invalid motion parameters: robot radius" |
| `cli_octile_obstacles_map` | `maps/obstacles.csv` with `--connectivity 8 --reference-bfs`: expects 26 steps (8 diagonal), 14.66 m, the exact Dijkstra cost 29.313708 cells matching A*, then `GOAL_REACHED` |
| `cli_octile_maze_has_no_diagonals` | `maps/maze.csv` with `--connectivity 8`: expects 60 steps with 0 diagonals (every turn is at a wall, so the corner rule forbids all diagonals) |
| `cli_octile_with_footprint` | fine map, `--robot-radius 0.2 --connectivity 8 --reference-bfs`: expects a match with the reference, then `GOAL_REACHED` |
| `cli_rejects_bad_connectivity` | `--connectivity 6`: expects "--connectivity must be 4 or 8" |
| `cli_diff_drive_obstacles_map` | `maps/obstacles.csv` with `--robot-model diff-drive`: expects `GOAL_REACHED`, travel time 38.50 s and 3 turns in place taking 4.50 s |
| `cli_diff_drive_footprint_octile` | fine map with footprint, 8-connectivity, differential drive and `--initial-heading 90`: expects `GOAL_REACHED` with 0 collisions |
| `cli_rejects_bad_robot_model` | `--robot-model tank`: expects "--robot-model must be point or diff-drive" |
| `cli_lidar_scans` | obstacles map, differential drive, `--lidar-beams 72`: expects 386 scans and a closest return of 0.250 m |
| `cli_rejects_bad_lidar_settings` | `--lidar-fov 400`: expects "invalid LiDAR parameters" |
| `cli_ekf_localization` | obstacles map, differential drive, 72 beams, `--ekf`: expects EKF position error RMS 0.004 m and dead reckoning 0.533 m |
| `cli_ekf_requires_diff_drive` | `--ekf` with the point robot: expects a clear error |
| `cli_rejects_bad_ekf_settings` | `--ekf-range-sigma 0`: expects "invalid EKF parameters" |
| `cli_dynamic_replanning` | obstacles map with `maps/obstacles_dynamic.csv`: expects `GOAL_REACHED` and 3 replans |
| `cli_rejects_bad_dynamic_obstacles` | a route through a wall (`tests/data/bad_dynamic_obstacles.csv`): expects the line and the reason |
| `cli_rejects_bad_replanning_settings` | `--sensing-range 0`: expects "invalid replanning parameters" |
| `ghost_experiment_example` | the documented example map: expects a 2-cell explanation with 8 optimal alternatives |
| `alignment_experiment_example` | the documented example: the same 0.75 m door is passable at one sub-cell offset and NO_PATH at another |
| `heading_experiment_p_vs_pid` | runs the P vs. PID experiment: expects P to never settle under the disturbance and PID to settle |
| `heading_experiment_rejects_bad_gain` | `--kp -1`: expects a non-zero exit code |
| `cli_rejects_overshooting_heading_gain` | `--heading-gain 20` (gain × dt = 2): expects "invalid motion parameters: heading gain" |
| `cli_navigation_timeout` | corridor map with `--max-time 2`, expects `TIMEOUT` |
| `cli_rejects_invalid_motion_parameters` | `--speed -1`, expects "invalid motion parameters" |
| `cli_reports_unreachable_goal` | `tests/data/unreachable_goal.csv`, expects `NO_PATH` and navigation `NOT_RUN` |
| `cli_rejects_occupied_start` | start inside an obstacle, expects `START_OCCUPIED` |
| `cli_rejects_malformed_map` | `tests/data/nonrectangular.csv`, expects a rectangularity error |
| `cli_rejects_missing_map` | nonexistent map, expects "does not exist" |
| `cli_rejects_bad_arguments` | non-integer coordinate, expects a nonzero exit code |
| `python_visualization` | `tests/test_python_visualization.py` (65 tests, about 35 s); only registered when CMake finds a Python with NumPy, Matplotlib and Pillow |

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
| LiDAR | scans load as (scans × beams), one per trajectory sample, with heading-relative beams; no-return beams carry the maximum range; scans of another run and malformed beam orders are rejected; ray segments start at the robot and have the measured lengths; the animation draws the latest scan, and its blitted frames equal full renders; `lidar_scan.png` is written only with LiDAR data; `run_demo.py` forwards the LiDAR options |
| Localization | `localization.csv` loads with symmetric, positive-definite covariances and matches the trajectory (rejected otherwise); the EKF error stays below 5 cm and dead reckoning drifts more than 10 times as far; heading errors are wrapped; the 3σ ellipse of a known covariance has the right axes and angle; the figures exist only with `--ekf`, and a stale `localization.csv` is removed; the animation shows the latest dead-reckoning pose and estimate, and its blitted frames equal full renders; `run_demo.py` forwards `--ekf` and the odometry options |
| Alignment experiment plots | `plot_alignment_experiment.py` renders the three figures from the committed `docs/alignment/` data; the documented doorway result holds in the data (the point planner always passes from 0.67 m, any overlap passes 0.85 m for under 60 % of alignments); missing input is reported |
| Ghost experiment plots | `plot_ghost_experiment.py` renders the three figures from the committed `docs/ghost/` data; outcome counts add up per strategy and match the documented headline; missing input is reported |
| Dynamic obstacles | the three output files load and match the trajectory (rejected otherwise); obstacle positions follow their routes; replanned paths run from their start cell to the goal; the robot kept a positive gap and waited at least once; the current plan switches at each successful replan; `replanning.png` exists only with obstacles, and stale dynamic files are removed; the animation draws the obstacles at their interpolated positions, faded when unseen, and its blitted frames equal full renders; `run_demo.py` forwards the options |
| Heading control plots | the experiment's metrics show P's steady-state errors `−d/Kp` and `w/Kp` and PID's ≈ 0, and "never" settling for P under the disturbance; all five figures are written; run files have a uniform 10 ms time base, wrapped errors and commands within the limit; missing and malformed inputs are reported |
| Differential drive | heading columns are loaded (45 turn samples for 3 turns, no translation while turning); point trajectories have no heading; heading interpolation takes the short way across ±180°; the animated heading line points along θ and the status reads TURNING / NAVIGATING; blitted frames with the heading line equal full renders; the position figure gains a θ(t) panel; `run_demo.py` passes string options unquoted |
| Connectivity | an 8-connected plan is titled "8-connected A* path: 26 steps (8 diagonal)" and its drawn path contains the 8 diagonal moves; the 4-connected title is unchanged; an 8-connected benchmark marks every run optimal; `compare` writes its table and plot, measures the 29.3 % obstacle-free saving, and rejects benchmarks made on different maps; `run_demo.py` forwards `--connectivity` |
| Footprint | the inflated map is read from the simulator (372 inflated cells, obstacles preserved) and a mismatched one is rejected; the animated robot is a disc with the true radius, centred on the trajectory; blitted disc frames equal full renders; static figures and GIF are produced with a footprint; `run_demo.py` forwards `--robot-radius` |
| Export | the GIF is valid and looping with several frames; a failed plan produces `navigation_failed.png`, no GIF, and removes a stale GIF; MP4 without FFmpeg reports an error and still writes the GIF; a trajectory from another run is rejected |
| Analysis | the 95 % Wilson interval is correct (120/120, 10/20); summaries keep failed runs in counts, rates and planning-time statistics while path lengths use successful plans only; single-run metrics are recomputed from the raw CSV files (17.0 m, 34.0 s on the obstacles map) and a tampered `simulation_metrics.csv` is detected |
| Benchmark | benchmark maps are identical for the same seed and differ between seeds, with start and goal cleared; a 12-run benchmark runs end to end and writes every summary file and plot |
| `run_demo.py` | the simulator is called with an argument list (a map name containing spaces and `;` stays one argument); unset options are not passed, so the C++ defaults apply; animation runs only when requested; a missing executable gives exit code 127 |

## LiDAR tests (`test_lidar.cpp`)

| Test | What is checked |
|---|---|
| Exact distances | to an obstacle face (3.5 m), the map border (1.5 m and 2.5 m), along a 30° beam (3.5 / cos 30°); no return beyond the range; a hit exactly at the range limit counts |
| Corners | a 45° beam hits the diagonal obstacle at the shared corner (1.5·√2 m); a beam grazing an obstacle corner registers the touch |
| Sensor inside an obstacle | range 0 |
| Exact vs. reference | 2400 random rays on 60 random maps (0.25 and 0.5 m cells): the traversal equals an independent slab-intersection reference within 1e-9 m, and is never later than 1 mm sampling (which can miss thin slivers; it did on one ray) |
| Beam layout | 8 beams over 360° from −180° with one beam straight ahead and none duplicated at +180°; 5 beams over 90° reach both ends of the field of view; a single beam points straight ahead |
| Scans along a trajectory | one scan per sample with the sample's time and position; the forward beam follows the heading; exact ranges at the start (4.25 m ahead, 0.75 m behind) and at the end |
| Noise | 8000 noisy ranges have mean ≈ 0 and standard deviation ≈ σ (within 3 mm); same seed gives the same scans, another seed different ones; huge noise is clamped to [0, max range] |
| Validation | beams, field of view, range and noise limits |

**LiDAR-off equivalence.** Without `--lidar-beams`, the new and previous binaries
produce identical output files and console summaries on 400 random maps (mixing
footprints, 8-connectivity, the differential drive and `--reference-bfs`), and a stale
`lidar_scans.csv` is removed. The Python figures and GIFs of three scenarios are
byte-identical.

## EKF localization tests (`test_ekf.cpp`)

| Test | What is checked |
|---|---|
| Odometry increments reproduce the trajectory | "turn by Δφ, then drive Δs" from the true increments rebuilds the simulated differential-drive trajectory to 1e-9 m; every step is a turn or a drive; the commanded increments equal the true ones |
| Odometry variances | the formulas `k_dist²·Δs` and `k_turn²·Δφ + k_drift²·Δs`, zero without motion |
| Prediction Jacobians | `P' = F P Fᵀ + V M Vᵀ` equals the result with `F` and `V` from central finite differences of the motion model |
| Range Jacobian | the analytic `∂r/∂(x, y, θ)` matches finite differences of the ray caster at 500 random poses and beams (vertical and horizontal faces) |
| Single-beam update | the information-form update equals the textbook `K = P Hᵀ / (H P Hᵀ + R)` update to 1e-12 |
| Innovation gate | an outlier 1 m off is rejected and leaves the estimate unchanged; without the gate it is fused; beams without a return are ignored |
| Convergence | from 0.3 m and 6° off, repeated exact scans bring the estimate within 1 mm and 1 mrad of the true pose |
| Along a trajectory | one sample per trajectory sample; same seed, same odometry with or without the LiDAR; without the LiDAR the estimate equals dead reckoning and the uncertainty never shrinks; with it, the error stays under 3 cm while dead reckoning drifts over 10 cm; determinism and seeds |
| Noise-free odometry | dead reckoning follows the truth to 1e-9 |
| Consistency | mean NEES over 200 prediction-only runs and 40 runs with a noisy 36-beam LiDAR is within [2.5, 3.5] and [2.4, 3.8] (measured 3.21 and 3.28 with the default grazing model) |
| Heading wrap | driving west (heading ±180°), where the estimate flips between +π and −π, errors and NEES stay small |
| Initial error | over 2000 seeds the initial estimate's error has zero mean and the stated standard deviation; dead reckoning starts from the same pose |
| Validation | noise, range sigma, gate and initial-uncertainty limits |

**Monte Carlo check of the covariance.** Beyond the mean NEES, the per-axis variance
of the actual errors over 200 prediction-only runs was compared with the filter's
covariance at several times (ratios 1.0–1.2). This found a real modelling error: with
turn noise and drift both on, the heading variance was 35 % too large (mean NEES 2.4),
because the process noise was evaluated at the measured increments. See
[algorithms.md](algorithms.md#13-ekf-localization---ekf).

**EKF-off equivalence.** Without `--ekf`, the new and previous binaries produce
identical output files and console summaries on 400 random maps (about half with the
LiDAR on, which also covers the ray caster's new hit-face output), and the Python
figures and GIFs of three scenarios are byte-identical.

## Moving-obstacle perception tests (`test_obstacle_tracking.cpp`)

| Test | What is checked |
|---|---|
| Static world | 100 scans with 2 cm noise from random free cells of the obstacles map: map subtraction leaves no detection |
| Disc detection | discs of radius 0.2, 0.3, 0.45 m 1.4 m away (360 beams): one detection each, center and radius within 3 cm; with 72 beams the center is still within 8 cm |
| Disc fit on an arc | points on the near side of a circle give its center and radius to 1e-6 |
| Fragments merged | a post in front of a disc splits its returns into two clusters: one detection, center within 6 cm |
| Patrol through reversals | 50 s of an obstacle patrolling a 2.5 m lane: one track throughout; 1.5 s after every reversal the velocity is within 0.15 m/s; both turning points learned (lane length within 15 cm); predictions 10 s ahead stay between them and are within 25 cm of the truth over the next 2 s |
| Unwatched reversal | an obstacle turning round out of sight (a corridor seen through a gap): no turning point is learned |
| Two-point start | a 2 m/s obstacle: on its second detection the track is confirmed with the velocity within 0.25 m/s |
| Negative evidence and coasting | an obstacle vanishing in plain view is dropped within 4 scans; one walking behind a wall is kept (coasting) |
| Simulator with LiDAR perception | the sample scenario (`obstacles_dynamic.csv`), point and differential drive: goal reached, clearance > 0, all three obstacles tracked, deterministic |
| Occlusion awareness | an L-shaped corridor with nothing in it: the occlusion-aware robot holds at the blind corner (more wait steps) and still reaches the goal |

The decision rules in `test_dynamic.cpp` are tested with oracle perception (ideal
sightings). Its random-scenario guard allows 5 collisions in 120: over 1000 scenarios
of that generator the oracle collides in 2.3 % (and LiDAR tracking in 0.7 %).

## Correlated-error tests (`test_correlated_errors.cpp`)

| Test | What is checked |
|---|---|
| Bias process | a constant bias is the same in every scan; over 4000 seeds its std is the configured 3 cm (±1.5 mm); a Gauss–Markov bias has the stationary std and the lag-one correlation `exp(−dt/τ)` (40 000 scans) |
| Bias shifts every beam alike | with noise off, every beam with a return of a scan is shifted by the same amount; misses keep `max_range`; the shift drifts between scans |
| Without a bias the scans are unchanged | ranges equal the original model (one generator, white noise per beam with a return, clamped) bit for bit |
| Augmented EKF without extra states | equals the classic batch filter along a whole run: estimates to 1e-9, covariance to 1e-12, identical used/rejected counts |
| Bias state | 16 runs with a 3 cm bias: independent beams' mean NEES > 20 (measured ~810), bias state within [2.4, 4.5] (measured 3.76), bias estimated to 0.2 mm |
| Wall states on a rasterized world | 6 continuous worlds rasterized at 0.1 m: independent beams' mean NEES > 100 (measured ~2260), wall states on the line surface < 8 (measured 4.8), smaller position RMS |
| Validation | the new bias, map-error and incidence limits; LiDAR bias limits |
| Boundary chains | an 8 × 6 map with a 2 × 3 block and a diagonal pair: 3 chains with 28, 10 and 8 faces (the diagonal pair is one obstacle) |
| Axis-aligned walls are exact | on the obstacles map, 2000+ random beams: the line surface's ranges and normals equal the grid's to 1e-9 |
| Slanted walls are recovered | walls at 3°, 6°, 17°, 33° rasterized at 0.1 m: line range RMS below 6 mm and below 0.3× the grid's; along the wall the line surface's `∂r/∂y` never jumps, the grid's jumps by > 0.5 |
| Surface Jacobians | both surfaces' analytic Jacobians match finite differences of their own ray casts (600+ beams, incidence below 73°) |
| Range Hessian | the closed-form second derivatives match finite differences of the Jacobian, up to 83° incidence |
| Grazing model | 12 runs on the exact world: the model keeps more beams than the 78.46° cutoff with mean NEES in [2.4, 5]; an 89° cutoff without modeling is > 3× more overconfident |
| Grazing near a wall end | a 70° grazing beam hitting 0.1 m from a wall's end is skipped (slide 2σ tan α = 0.28 m); the same beam mid-wall is fused |

## Grid-alignment tests (`test_alignment.cpp`)

| Test | What is checked |
|---|---|
| Geometry | containment (boundary included), distances, overlap needs a positive area (edge and corner touches do not count), clipped areas (incl. a rotated square), ray-polygon and ray-circle hits, segment-polygon distance |
| Rasterization | cell-aligned obstacles: all rules agree; the center rule samples the cell center; the majority rule needs more than half the area (30 % no, 60 % yes); any overlap contains the other rules on 200 random rotated rectangles |
| Placement | the grid holds the whole scene for 100 random rotations and offsets; the transform is rigid |
| Exact LiDAR | hit distance to a wall face, an in-sector ghost hit rate (0.6 over 4000 beams), out-of-sector beams always pass, the starting cell counts as observed, `isReal` |
| Point evidence | a beam passing 3 mm away and continuing contradicts a hit; one that stops just past it (grazing) does not; an isolated hit is filtered out |
| Point planner | segments join consecutive hits; chain ends carry a margin of range x beam spacing; the margin closes a gap a corner may hide (and without it the gap would pass); unobserved space is not used |
| Alignment invariance | over 16 random alignments per width, the point planner's answer is the same for every alignment outside its tolerance band, while any overlap varies on a 0.75 m door and never passes a door narrower than 2 r |
| Ground truth | the two-threshold fine lattice: 0.62 m passable, 0.58 m blocked, 0.60 m borderline |
| Artifact vs. ghost | grid NO_PATH on a narrow passable door is recovered with no sensing (artifact); a reflection in a wide door is removed by re-sensing (not an artifact, beams > 0, never unsafe); a door narrower than 2 r is never opened |

**Refactor check.** The perception loop now runs against a `SensorWorld` interface.
Problem 1's tests pass unchanged, and its 300-case recovery CSV is identical apart from
timing.

## Ghost-cell tests (`test_ghost_cells.cpp`, `test_ghost_perception.cpp`)

| Test | What is checked |
|---|---|
| Covers vs. inflation | on 150 random maps (radius 0 to 0.75 m, two resolutions), a cell is blocked by `inflateObstacles` iff border-blocked or its cover is not inside the removed set, both as they are and after random removals |
| Minimum count vs. brute force | 1500 random maps (point and disc robot, 4- and 8-connected, two resolutions): the exact minimum equals brute force up to 3 cells; the explanation opens a valid path, which is the one returned |
| Every optimal explanation | on 306 maps, the listed optimal sets equal brute force's |
| Weighted and unit optimum | small maps, exhaustive search over every subset (removals of any size): equal optimum with random and unit weights |
| Overlapping footprints | a single-lane corridor where removing either of two adjacent obstacles frees nothing: the answer is the pair |
| Statuses | path exists; impossible (map border, out of bounds); weights pick the cheap cell; a cost limit stops with a lower bound; the start itself as the ghost |
| Beam trace and ghost source | passes and hit cell, range limit, map edge; an in-sector ghost hits at its probability (0.7 measured over 4000 beams), out of sector it is passed |
| Sensor noise | miss rate and spurious early returns at their expected rates |
| Occupancy update | log-odds steps, hit and pass counts, hit bearings, clamp, unknown as free |
| Ghost evidence | passes outweigh hits as specified by the two-hypothesis ratio |
| Doorway ghost | a ghost seen only from inside its sector: the most likely explanation recovers by re-sensing a few cells from another side, never clearing a real cell; clearing the map brings the ghost back (gives up); re-sensing everything spends over 5x the beams |
| Doorway real obstacle | a real obstacle in the same place: no re-sensing strategy clears it (all give up); deleting without looking is unsafe |

**Original experiment reproduced.** On the same 400 maps as the brute-force study that
proposed the problem, the exact solver gives identical counts at every density (and
average single-cell options), in 8.6 s instead of 1 min 44 s.

## Dynamic-obstacle tests (`test_dynamic.cpp`)

| Test | What is checked |
|---|---|
| Ping-pong motion | positions along an L-shaped route, round the corner, at the end, back and periodic; corner and reversal times |
| Exact closest approach | 300 random cases against 20 000-point sampling: never larger, and within 0.6 mm |
| Contact between samples | an obstacle 0.5 m away at both ends of a step that passes straight through the robot is detected |
| Loading | a valid file; missing pairs, non-numbers, out-of-map and fractional cells, routes through walls, bad radius and speed |
| Visibility | range measured to the obstacle's edge; a wall hides an obstacle behind it |
| No obstacles | the dynamic loop gives exactly the static trajectory (both robot models, 4- and 8-connected) |
| Crossing obstacle | the robot waits and crosses behind an obstacle sweeping the whole corridor; a blind robot collides with it |
| Persistent blockage | a slow obstacle on the straight route: the first replan comes after the patience, the detour avoids the blocked cells, and with no patience it detours sooner |
| Retreat | head-on with a late sighting: the robot backs off and gets through |
| No way around | an obstacle patrolling across a doorway's exit: plans fail while it is blocked, the robot waits, then passes |
| Differential drive backs up | it reverses without turning (heading unchanged), and the EKF's odometry model still rebuilds the trajectory exactly |
| Random invariants | 120 random scenarios: the robot stays in free cells, collision-free runs never come within contact (exact check over every step) and report that gap, no replan fails because of the goal, at least 108 reach the goal |
| Validation | sensing range, margin and horizon limits; a scenario without a planning map |

**Stress test and held-out results.** Each rule of the replanning method was added
after analysing failures on 300 random scenarios. The success rates reported in the
README were measured on 500 different scenarios (another seed). See
[algorithms.md](algorithms.md#14-dynamic-obstacles-and-replanning---dynamic-obstacles-file).

**Static equivalence.** Without `--dynamic-obstacles`, the new and previous binaries
produce identical output files and console summaries on 400 random maps (mixing
footprints, 8-connectivity, the differential drive, the LiDAR and the EKF).

## Heading-control tests (`test_heading_control.cpp`)

| Test | What is checked |
|---|---|
| Plant is exact | the closed-form step matches a 500 000-step Euler integration (1e-5) and splitting a step in two (1e-12) |
| Plant without lag; saturation | `θ += (u + d)·dt` for τ = 0; commands are clamped to ±1.5 rad/s |
| P overshoot vs. theory | for Kp = 2, 4, 8 (ζ = 0.71, 0.50, 0.35) the simulated overshoot of a small, unsaturated step matches `exp(−ζπ/√(1−ζ²))` within 0.1 percentage points |
| Steady-state errors | P under a 0.3 rad/s disturbance settles at exactly `−d/Kp` (1e-4) and never enters the 2 % band; on a 0.5 rad/s ramp it lags by `w/Kp`; PID drives both to below 1e-4 rad |
| No derivative kick | the first PID command after a reference step is exactly `Kp·e + Ki·e·dt`; a moving heading is damped by `−Kd·dθ/dt` |
| Anti-windup | 100 saturated periods leave the integral at 0; unsaturated periods integrate normally |
| PID with Ki = Kd = 0 | identical to P, sample by sample |
| Command limit | P and PID with aggressive gains never exceed ±1.5 rad/s |
| Metrics | overshoot, rise time, settling time, steady-state error, IAE, effort, command variation and saturated share on a hand-made response with known values |

## Differential-drive tests (`test_diff_drive.cpp`)

| Test | What is checked |
|---|---|
| Angle wrapping | results in (−π, π], including ±π, multiples of 2π and large angles |
| Unicycle integration | straight line, pure rotation, and a quarter circle `(0,0,0) → (1,1,π/2)`; two half steps equal one full step to 1e-12; agreement with a 200 000-step Euler integration |
| Heading controller | saturation at ±1.5 rad/s, the proportional region, the exact settling step; for 1000 random errors and gains with `k·dt` from 0.1 to 1: never crosses the target, monotone, finishes |
| Straight path | no turn; trajectory bit-identical to the point robot; full speed through collinear waypoints |
| Turn times | facing backwards: 25 steps (2.5 s) to turn 180°, travel time 9.0 + 2.5 s; one 90° corner: 15 steps (1.5 s), final heading 90° |
| Follows the plan exactly | 200 random maps, 4- and 8-connectivity, random initial heading: dropping the turn-in-place samples leaves exactly the point robot's positions; travel time = point time + turning time; `v ≤ v_max`, `\|ω\| ≤ ω_max`, never turning and driving in the same step, heading change per step ≤ `ω_max·dt`, heading equals the direction of motion while driving |
| Footprint and diagonals | 200 random maps with radii up to 1.2 cells, 4- and 8-connectivity: `GOAL_REACHED` with 0 collisions |
| Timeout while turning | a 180° turn with a 1 s limit stops after 10 steps without moving |
| Validation | `k·dt > 1`, zero turn rate, too large a tolerance and a NaN heading are rejected; the point robot ignores these settings |

**Point-robot equivalence.** The point robot is still the default. Run with the same
inputs, the new and previous binaries produce identical `path.csv`, `trajectory.csv`,
planning metrics (except timing) and every previously existing `simulation_metrics.csv`
column on 400 random maps, mixing `--reference-bfs`, footprints and 8-connectivity. For
three scenarios (4- and 8-connected obstacles map, fine map with footprint), the PNG
figures and the GIF are byte-identical.

## 8-connected planning tests (`test_octile.cpp`)

| Test | What is checked |
|---|---|
| Exact cost comparison | `3 > 2√2`, `7 < 5√2`, the near-tie `99 > 70√2` (99² − 2·70² = 1), values near 10⁸, and agreement with `long double` arithmetic on 20 000 random pairs |
| Octile distance | `(0,0)→(5,2)` = 3 straight + 2 diagonal; symmetric; zero; pure diagonal; pure straight |
| Empty map | `(0,0)→(9,4)` costs exactly the octile distance (5 + 4√2); the path is valid; A* expands only the path cells |
| No corner cutting | both orthogonal cells blocked: `NO_PATH`; one blocked: two straight moves instead of the diagonal; both free: the diagonal |
| Path validation | diagonals accepted with 8- and rejected with 4-connectivity; corner cuts, jumps and moves into obstacles rejected; `gridPathCost` counts moves |
| Tie-breaking | `(0,0)→(2,1)` has two optimal routes; the documented `(f, h, y, x)` order picks "diagonal first"; repeated plans are identical |
| A* vs. exact Dijkstra | 500 random maps (1–45 cells per side, 0–45 % obstacles): A* cost exactly equals the reference; `NO_PATH` exactly when the reference fails; same success as 4-connectivity on every map; never longer than the 4-connected path |
| Planner reuse | one `AStarPlanner` alternating 4- and 8-connectivity on 200 maps equals fresh plans |
| Diagonal motion | 5 diagonal legs on an empty map reach the goal (15 steps per 0.707 m leg); the same path is `INVALID_PATH` for a 4-connected simulation |
| Corner cutting in the simulator | a hand-made path squeezing between two obstacle cells is a `COLLISION`; the robot stops within one step before the shared corner (7 steps, 0.35 m of 0.354 m) |
| Footprint + diagonals | 300 random maps, resolutions 0.25/0.5/1 m, radii 0–1.5 cells: every move of every 8-connected plan passes the exact swept-disc test (or the touch test for point robots), and the simulation reaches the goal with 0 collisions (216 plans, 621 diagonal moves) |

**4-connected equivalence.** 4-connected planning is still the default. Run with the
same inputs, the new and previous binaries produce identical `path.csv`,
`trajectory.csv`, `simulation_metrics.csv` and pre-existing `planning_metrics.csv`
columns (except timing) on 400 random maps, including runs with `--reference-bfs` and
with a footprint. The Python PNGs and GIF for the obstacles demo are byte-identical, and
the micro-benchmark result checksum is unchanged.

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
  | 8-connected planner ignores the corner rule | 3 (octile tests) |
  | Manhattan instead of octile heuristic (overestimates) | 3 |
  | Diagonal moves cost 1 | 4 |
  | Inexact cost comparison (√2 approximated) | 3 |
  | Simulator's corner-touch check removed | 1 |
  | Differential drive moves without turning first | 3 (differential-drive tests) |
  | No angular-speed saturation | 3 |
  | Sign error in the arc integration | 1 |
  | No exact settling step in the heading controller | 2 |
  | PID without anti-windup | 1 (heading-control tests) |
  | PID derivative on the error (derivative kick) | 3 |
  | Plant ignores the motor lag in the heading | 2 |
  | Integral term dropped | 2 |
  | Settling time measured at the first entry into the band | 2 |
  | LiDAR: a corner touch is not counted (diagonal cell only) | 1 (LiDAR tests) |
  | LiDAR: wrong first boundary for positive directions | 4 |
  | LiDAR: range limit treated as exclusive | 1 |
  | LiDAR: noise not clamped | 1 |
  | LiDAR: duplicate beam at ±180° | 2 |
  | EKF: wrong sign of the range Jacobian's heading term | 5 (EKF tests) |
  | EKF: wrong sign in the prediction Jacobian F | 2 |
  | EKF: odometry noise not added in the prediction | 4 |
  | EKF: prior information dropped in the update | 1 |
  | EKF: innovation gate disabled | 5 |
  | EKF: odometry noise std = variance (missing square root) | 1 |
  | EKF: drive before turning in the odometry model | 1 |
  | EKF: turn variance uses the distance | 1 |
  | EKF: grazing beams not skipped | 2 |
  | EKF: no initial error drawn | 1 |
  | EKF: heading error not wrapped in the NEES | 1 |
  | Dynamic: closest approach not clamped to the step | 7 (dynamic tests) |
  | Dynamic: contact test ignores reversals within a step | 1 |
  | Dynamic: route not folded back (obstacle jumps to the start) | 4 |
  | Dynamic: visibility ignores walls | 1 |
  | Dynamic: no velocity estimate | 2 |
  | Dynamic: robot's own cell not unblocked before replanning | 1 |
  | Dynamic: no retreat | 3 |
  | Dynamic: look-ahead gate off | 2 |
  | Dynamic: patience ignored | 2 |
  | Dynamic: a waiting robot not checked for contact | 1 |
  | Dynamic: no backing up (differential drive) | 1 |
  | Dynamic: goal counted as blocked | 1 |
  | Dynamic: evasive step off | 1 |
  | Ghost: dominance reversed (subsets pruned) | 3 (ghost tests) |
  | Ghost: bound overestimates (cost + H) | 3 |
  | Ghost: diagonal side cells ignored | 3 |
  | Ghost: map border ignored | 5 |
  | Ghost: touching counts as blocking | 1 |
  | Ghost: stops after the first optimal set | 1 |
  | Ghost: weights ignored | 2 |
  | Ghost evidence: passes ignored | 2 |
  | Ghost re-sensing ignores bearings (nearest viewpoint) | 1 |
  | Ghost: re-sensed cell not decided by evidence | 1 |
  | Ghost source ignores direction | 2 |
  | Ghost: passes not counted | 2 |
  | Alignment: touching counts as overlap | 2 (alignment tests) |
  | Alignment: center rule samples the cell corner | 1 |
  | Alignment: majority threshold a quarter | 1 |
  | Alignment: the beam's starting cell not observed | 1 |
  | Alignment: ghost sector ignored | 2 |
  | Alignment: contradiction without continuing past the hit | 1 |
  | Alignment: no outlier filter | 1 |
  | Alignment: no corner margin | 1 |
  | Alignment: unobserved space ignored | 1 |
  | Alignment: hits joined at any distance | 2 |
  | Alignment: ground-truth thresholds swapped | 1 |
  | Alignment: artifact flag set after re-sensing | 1 |

- **Benchmark-scale optimality check:** every one of the 480 benchmark runs
  ([performance.md](performance.md)) compares A* with the reference BFS through
  `--reference-bfs`. All 480 agreed: 388 equal costs and 92 runs where both found no path.
- **Optimization equivalence (Phase 6):** the optimized planner was compared with the
  previous implementation. A result checksum over 160 micro-benchmark maps is identical,
  and the old and new `robot_simulator` binaries produce byte-identical `path.csv`,
  `trajectory.csv` and non-timing metrics on 400 random maps
  ([performance.md](performance.md#verification)).

## Sanitizers and static analysis

Run in Phase 6, and again after each extension (latest: grid alignment):

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all"
cmake --build build-asan -j && ctest --test-dir build-asan --output-on-failure
```

| Tool | Result |
|---|---|
| AddressSanitizer + UndefinedBehaviorSanitizer (Debug, asserts on) | all 49 C++ and CLI CTest entries pass, no reports |
| `cppcheck` 2.7 (`--enable=warning,performance,portability`) | no findings |
| `clang-tidy` 14 (`bugprone-*`, `performance-*`, `clang-analyzer-*`) | 3 notes about string concatenation in CLI error messages (run only when reporting an error); left as is |
| `ruff` (pyflakes, pycodestyle, bugbear, pyupgrade, simplify) | no findings |
| GCC `-Wall -Wextra -Wpedantic -Wshadow -Wconversion` | no warnings |

