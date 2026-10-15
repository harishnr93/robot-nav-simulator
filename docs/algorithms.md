# Algorithms

## 1. Occupancy grids

**Why:** a robot needs a model of where it may and may not go. An occupancy grid
divides the environment into equal square cells and marks each as free (`0`) or
occupied (`1`). It is the simplest map representation that path planners can search
directly.

**Data:** a `width × height` array (row-major) plus a resolution in meters per cell.
With the default 20 × 20 grid at 0.5 m/cell, the environment is 10 m × 10 m.

**Grid ↔ world:** a cell represents the square region it covers; its world position is
its center: `world = (grid + 0.5) * resolution`. Example: cell `(3, 1)` at 0.5 m/cell →
`(1.75 m, 0.75 m)`.

## 2. Graph view of the grid

Each free cell is a graph node. In the 4-connected model, a cell is connected to its
up, right, down and left neighbors if they are inside the map and free. Every edge
costs 1 (one cell = `resolution` meters).

```text
          (x, y-1)
             │
 (x-1, y) ─ (x, y) ─ (x+1, y)
             │
          (x, y+1)
```

## 3. A* search

**Why:** we need the *shortest* collision-free route, and we want to find it without
exploring the whole map. Breadth-first search also finds shortest paths on
uniform-cost grids, but it explores in all directions equally. A* uses a heuristic to
focus the search toward the goal.

**How it works:**

```text
g(n) = cost of the best known path from start to n   (steps)
h(n) = |x_goal − x_n| + |y_goal − y_n|               (Manhattan distance)
f(n) = g(n) + h(n)                                    (estimated total cost)

open  ← {start}, g(start) = 0
while open is not empty:
    n ← entry in open with the lowest f  (ties: see below)
    if n was already expanded: skip it   (stale duplicate)
    mark n expanded
    if n == goal: rebuild the path by following parents; return SUCCESS
    for each free neighbor m of n that is not expanded:
        if g(n) + 1 < g(m):
            g(m) ← g(n) + 1, parent(m) ← n, push m with f(m)
return NO_PATH
```

**Why the Manhattan heuristic is correct here:** on a 4-connected grid each move
changes `x` or `y` by exactly one, so you need *at least* `|dx| + |dy|` moves to reach
the goal. The heuristic therefore never overestimates (admissible). It also changes by
at most 1 per move, which is the move's cost (consistent). With a consistent heuristic,
the first time A* expands a node its `g` value is already optimal. So when the goal is
expanded, the path is shortest.

**Implementation details (`src/astar.cpp`, class `AStarPlanner`):**

| Concern | Choice |
|---|---|
| Open set | two `std::priority_queue` min-heaps, one per f value (see below) |
| Decrease-key | not supported by `std::priority_queue`; cells are pushed again when a cheaper `g` is found, and stale entries are skipped when popped (`closed` check) |
| Per-cell state | one byte: *visited*, *closed*, and the direction to the parent (3 bits: 4 or 8 directions) |
| Costs `g` | `int` array, written only for cells the search reaches (never bulk-initialized) |
| Path reconstruction | follow the stored parent directions back from the goal |
| Repeated planning | an `AStarPlanner` keeps its buffers and resets only the cells the previous search touched; `planAStar()` is the one-shot wrapper |
| Expanded-node count | number of cells popped *and* expanded, including the goal |
| Timing | `std::chrono::steady_clock` around validation, buffer set-up and search (no file I/O) |

**Why two heaps suffice.** Every move adds 1 to `g` and changes `h` by exactly ±1, so
`f = g + h` changes by 0 or +2 along every edge. With a consistent heuristic, `f` never
decreases along the search, so if `F` is the smallest `f` in the open set, every entry
has `f = F` or `f = F + 2`. The planner keeps one min-heap for each. When the `F` heap
runs empty, the heaps swap and `F` grows by 2. Inside a heap, entries are 64-bit keys
`h << 32 | index`, where `index = y · width + x`. Comparing two keys is then a single
integer comparison that implements the `(h, y, x)` tie-break, and the overall pop order is
exactly `(f, h, y, x)`. A debug-build `assert` checks the two-bucket invariant on every
push.

**Deterministic tie-breaking.** Many grid paths have equal cost. The open set orders
entries by:

1. lowest `f`;
2. then lowest `h` (prefer cells closer to the goal; this reduces expansions);
3. then lowest `y`, then lowest `x` (a total order, so the result never depends on
   insertion order).

A cell's parent is only replaced by a *strictly* cheaper one. Together these rules
make the output a pure function of the map and the request. On an empty map the
`h` tie-break makes A* expand only the cells on its final path (tested in TC01).

**Special cases:**

| Case | Result |
|---|---|
| start == goal | `SUCCESS`, path = `[start]`, cost 0 |
| start/goal outside the map | `START_OUT_OF_BOUNDS` / `GOAL_OUT_OF_BOUNDS` |
| start/goal on an obstacle | `START_OCCUPIED` / `GOAL_OCCUPIED` |
| goal not connected to start | `NO_PATH`, empty path |

**Complexity:** with `V` free cells, each cell is expanded at most once and pushed at
most 4 times, giving `O(V log V)` time. Memory is one byte per map cell plus 4 bytes per
reached cell and the heap entries. The per-request set-up is `O(cells touched by the
previous search)` with a reused `AStarPlanner`, and one byte per cell for a fresh one.
Measured effect: [performance.md](performance.md#optimizations).

## 4. Reference BFS (validation only)

`bfsShortestPathCost()` runs a plain breadth-first search and returns only the shortest
step count. On a uniform-cost graph BFS is optimal by construction, so it is an
independent reference for checking A*'s optimality (test TC11, 500 random maps). It is
never used for normal planning.

## 5. Path checks and length

- `isValidGridPath()` checks that every waypoint is a free, in-bounds cell and that
  consecutive waypoints are 4-neighbors (exactly one step apart). This is the grid-level
  collision check for a planned route.
- `pathLengthMeters()` sums the Euclidean distances between consecutive cell centers.
  For 4-connected paths this equals `steps × resolution`.

## 6. Waypoint tracking (point-robot motion)

**Why:** a plan is a list of cells; a robot moves continuously through time. The motion
simulator turns the path into a time-stamped trajectory, which is what a real robot's
controller would have to produce and what the visualization and metrics are built on.

**Data in:** the planned grid path, the map, and `MotionConfig`
(defaults: speed 0.5 m/s, `dt` 0.1 s, goal tolerance 0.05 m, time limit 600 s; these are
simulation settings, not hardware measurements).
**Data out:** a trajectory sampled every `dt` seconds plus a navigation status.

**How it works** (`stepToward()` in `src/robot.cpp`, `simulatePath()` in
`src/simulator.cpp`):

```text
waypoints  = cell centers of the path (meters)
position   = waypoints[0];  t = 0;  record(t, position)
target     = 1
while target < number of waypoints:
    if steps == floor(max_time / dt): TIMEOUT
    direction     = waypoints[target] - position
    distance      = |direction|
    step_distance = min(speed * dt, distance)            # clamp: never overshoot
    next          = position + direction / distance * step_distance
                    (exactly waypoints[target] when the clamp applies)
    if the segment position -> next is not collision-free: COLLISION (do not move)
    position = next;  steps += 1;  t = steps * dt;  record(t, position)
    if position == waypoints[target]: target += 1
GOAL_REACHED if |position - goal| <= goal_tolerance
```

Design decisions:

| Decision | Reason |
|---|---|
| Clamp the step to the remaining distance | prevents overshooting a waypoint, which on a grid could cut into an obstacle |
| Snap exactly onto a reached waypoint | floating-point error cannot accumulate from one waypoint to the next |
| `t = steps × dt` instead of `t += dt` | no accumulated rounding drift in timestamps |
| Leftover step distance is not carried over | each timestep moves toward exactly one waypoint, so the motion stays on the planned axis-aligned segments; the cost is a little lost time per waypoint |
| Zero-distance waypoints are skipped without using a timestep | safe handling of `start == goal`; no division by zero |
| Hard step limit from `max_time` | the loop always terminates |

**Worked example.** Default settings move `0.5 × 0.1 = 0.05 m` per step, and a cell is
0.5 m, so the robot needs exactly 10 steps per cell. The 34-step path on
`maps/obstacles.csv` therefore takes 340 steps = 34.0 s and covers 17.0 m. With
`--speed 0.7 --dt 0.13` a step is 0.091 m: 5 full steps cover 0.455 m and a 6th, clamped
step covers the last 0.045 m, so each cell takes 6 steps (0.78 s instead of the ideal
0.714 s).

**How to interpret the trajectory:** `trajectory.csv` lists `time,x,y,target_waypoint_index`.
For a successful run, consecutive rows are exactly `dt` apart, the distance between
rows is at most `speed × dt`, the first row is the start-cell center, and the last row
is within the goal tolerance of the goal-cell center.

## 7. Collision checking

**Why:** the planner only promises a path through free cells. The simulator checks
independently, while moving, that the robot really stays in free space. This catches
bad input paths, and with a robot footprint it verifies the disc against the real map
(section 8).

**How it works (point robot, `--robot-radius 0`):** before executing a step from `a` to
`b`, `isSegmentCollisionFree()`
samples the segment at intervals of at most `resolution / 20` (both endpoints included)
and converts every sample to a cell with `worldToGrid()` (`floor(world / resolution)`).
Each cell must be inside the map and free. If not, the step is not executed: the robot
stays at its last safe pose, `collision_count` becomes 1, and the run ends as
`COLLISION`. The starting position is checked the same way.

For the planner's paths (axis-aligned moves between centers of adjacent free cells)
this check is exact: such a segment only touches the two cells it connects, and
each cell spans 20 samples. The structural path check before the simulation only
rejects malformed paths (empty, out of bounds, or not 4-connected). Occupied cells are
deliberately left to the motion-time check, so a path through an obstacle is reported
as a collision at the obstacle boundary.

For a robot with a radius, the disc check of section 8 replaces this point check.

## 8. Robot footprint and obstacle inflation

**Why:** a real robot has a size. A point-robot path may graze an obstacle or pass
through a gap narrower than the robot. Modelling the robot as a disc of radius `r`
(`--robot-radius`) keeps the robot's whole body clear of obstacles.

**Collision rule.** Occupied cells are the full squares they cover, and the area outside
the map counts as an obstacle. The disc collides when it **overlaps** one of them:

```text
collision  ⇔  distance(center, nearest occupied square or map exterior) < r
```

Touching (`distance = r`) is allowed. So a robot whose diameter equals a gap's width
still fits through it; the unit tests check a 1 m door with `r = 0.5` (fits) and
`r = 0.6` (does not).

**How planning uses it (configuration-space inflation).** Instead of searching over
disc positions, A* plans for the robot's *center* on an **inflated map**:
`inflateObstacles()` blocks every free cell whose center is closer than `r` to an
obstacle or to the map border. A* then runs unchanged on that map (`planWithFootprint()`).
With 0.25 m cells, the rings around an obstacle are blocked at these radii:

| Ring around an obstacle cell | Center distance | Blocked when |
|---|---|---|
| side neighbors | 0.125 m | r > 0.125 m |
| diagonal neighbors | 0.177 m | r > 0.177 m |
| second ring, side | 0.375 m | r > 0.375 m |

So the inflation grows in steps as `r` increases (372 cells for `0.177 < r ≤ 0.375` on
`maps/obstacles_fine.csv`).

**Why the inflated path is safe while moving.** A* only checks the cell *centers* on the
path. The robot also passes between them, so the segment from one center to the next must
be clear too. For a 4-connected move between the centers of two adjacent free cells of the
inflated map, it is. Take a horizontal move from column `i` to `i + 1`:

- The distance to an obstacle square is `sqrt(dx² + dy²)`, where `dy` is constant along
  the move.
- `dx` is the distance to the square's x-interval `[k·res, (k+1)·res]`. If that interval
  overlaps the segment's x-range `[(i+½)·res, (i+1½)·res]`, it contains one of the two
  endpoints, so `dx = 0` at that endpoint.
- If it doesn't overlap, `dx` is smallest at the nearer endpoint.
- Either way, the smallest distance along the move occurs at an endpoint, and both
  endpoints are at least `r` from every obstacle. Distances to the map border change
  linearly along the move, so the same holds for them.

The test suite checks this on 300 random maps with random radii and resolutions, using
the independent swept-disc check below.

**How the simulator checks it.** Every step is checked against the **real** map (not the
inflated one) by `sweptDiscOverlapsObstacle()`, an exact test with no sampling:

- the distance between the step's segment and an obstacle square is 0 if they intersect
  (a slab test);
- otherwise it is attained at a segment endpoint or a square corner, because the closest
  points of two disjoint convex shapes include a vertex of one of them.

Only cells within the swept disc's bounding box are examined. With `r = 0` the
simulator keeps using the point check of section 7, so point-robot results are
unchanged.

**Statuses.** A start or goal that is free in the real map but blocked after inflation
is reported as `START_TOO_CLOSE_TO_OBSTACLE` / `GOAL_TOO_CLOSE_TO_OBSTACLE`.
`START_OCCUPIED` / `GOAL_OCCUPIED` remain for cells that are actually occupied.

**Conservative on coarse grids.** Inflation tests only cell centers. On a coarse grid, a
passage can be wide enough for the robot even though no cell center inside it is far
enough from both walls.

- Example: `maps/obstacles.csv` (0.5 m cells) has 1.0 m wide passages beside the map
  border. A robot with `r = 0.3 m` (0.6 m wide) physically fits, but both cell centers
  in the passage are within 0.3 m of a wall, so the planner reports `NO_PATH`.
- The same layout at 0.25 m cells (`maps/obstacles_fine.csv`) does find these routes.

Use a resolution that is small compared with `r`. Inflation never produces an unsafe
path, only possibly a missed one.

**Cost.** Inflation examines the cells within `r` of each free cell:
`O(cells × (2r / res + 1)²)`. It runs once per CLI call, before planning, and is timed
separately (`inflation_time_ms`): 0.05–0.14 ms for `maps/obstacles_fine.csv` (r = 0.12–0.4 m).

## 9. 8-connected A* (`--connectivity 8`)

**Why:** with only straight moves, the robot zig-zags along a "staircase" whenever the
goal is not straight ahead, and its paths are up to 41 % longer than the straight line
between cells. Allowing the four diagonal moves gives shorter and more natural paths.

**Model.**

| Move | Cost (cells) | Allowed when |
|---|---|---|
| up / down / left / right | 1 | the target cell is free |
| diagonal | √2 | the target cell **and both orthogonal cells it passes** are free |

The second rule is **no corner cutting**: a diagonal move between two cells that touch
only at a corner would scrape the obstacles there, so it is forbidden if either of
the two cells beside the move is occupied. One consequence: a diagonal can always be
replaced by two straight moves through a free cell, so 8-connectivity reaches exactly
the same cells as 4-connectivity. Planning succeeds or fails on the same maps; only
the paths get shorter.

**Heuristic: octile distance.** With `dx = |x_goal − x|` and `dy = |y_goal − y|`:

```text
h = (max(dx, dy) − min(dx, dy)) · 1 + min(dx, dy) · √2
```

Take `min(dx, dy)` diagonal steps, then the rest straight. On an obstacle-free
8-connected grid this is the exact shortest path length, so it never overestimates
(admissible). It is also a distance metric whose value for any single move equals
that move's cost, so `h(u) ≤ cost(u, v) + h(v)` (consistent). Hence, as in section 3,
the first time A* expands the goal, its cost is optimal.

**Exact costs, no floating point.** Every path cost and every heuristic value has the
form `a + b·√2`, with integers `a` (straight moves) and `b` (diagonal moves). The planner
stores costs as such integer pairs (`GridCost`) and compares them exactly
(`compareGridCost`):

```text
sign(a1 + b1·√2 − a2 − b2·√2) = sign(Δa + Δb·√2)
  both Δ ≥ 0 → positive, both ≤ 0 → negative (zero only if both are 0, since √2 is irrational),
  mixed signs → compare Δa² with 2·Δb²     (integers only)
```

This matters for correctness:

- Floating-point sums of √2 accumulate rounding error, so two equal-cost paths could
  compare as unequal (or the reverse). That would break the deterministic tie-breaking
  and make an exact optimality check impossible.
- With exact pairs, the tie-break is again `(f, h, y, x)`.
- `--reference-bfs` in 8-connected mode checks A* against an independent, deliberately
  simple Dijkstra search for **exactly** the same cost.
- The integer squares stay within 64 bits for any map up to the 10 000-cell size limit.

**Implementation.** `AStarPlanner` keeps the specialised two-heap open set for
4-connectivity (section 3, unchanged and byte-identical in its results). For
8-connectivity, `f` can take many values, so the open set is a single
`std::priority_queue` of exact `(f, h, index)` entries. The per-cell state byte stores
the parent direction in 3 bits (8 directions), and a second cost array holds the
diagonal part of `g`. Both arrays are reused between plans.

**Footprint safety with diagonals.** With a robot footprint, A* plans on the inflated map
(section 8), and the corner rule is applied there too: the two orthogonal cells must be
inflated-free. Then the whole disc stays clear during a diagonal move. The argument
extends the one in section 8:

- Shift coordinates so the move goes from the center of cell `(0, 0)` to the center of
  `(1, 1)` (unit cells). The segment lies inside the 2 × 2 block `[0, 2]²`, and all four
  cells of that block are inflated-free: the two endpoints, plus the two orthogonal cells
  required by the corner rule. So every obstacle square `Q` lies outside the block.
- Say `Q` is to the right of the block (`x ≥ 2`); the other sides are symmetric. For a
  point `P = (s, s)` on the segment, the x-distance to `Q` is at least the x-distance
  from the right-hand centers (`x = 1.5`).
- For the y-distance, `Q`'s y-interval `[l, l+1]` either contains one of the centers'
  y-values `0.5` or `1.5`, which gives distance 0 there, or lies entirely above or below
  them. In that case the y-distance is smallest at the nearer of the two.
- So some cell center of the block is no farther from `Q` than `P` is. All four centers
  are at least `r` from every obstacle, hence so is `P`.
- Map borders are linear along the move, so checking the endpoints suffices for them.

For a point robot (`r = 0`), the same argument shows a diagonal move never even touches
an obstacle. The test suite checks this on 300 random maps (621 diagonal moves) with the
exact swept-disc test, and with the touch test for point robots.

**Simulator.** In 8-connected mode the simulator also accepts diagonal waypoint steps.
For a point robot it adds a **touch check**: a step may not touch an occupied cell, which
is exactly what a corner-cutting diagonal does. A corner-cutting path is therefore
reported as `COLLISION`, with the robot stopped just before the corner. A disc robot is
covered by the swept-disc test. 4-connected simulations are unchanged.

**Travel time.** A diagonal leg is `√2 · resolution` long. That is 0.707 m with the
default 0.5 m cells, and does not divide evenly into 0.05 m steps. Since leftover step
distance is not carried over (section 6), each diagonal leg takes 15 steps (1.5 s)
instead of the ideal 0.707 / 0.5 = 1.414 s. On the obstacles map, 18 straight legs × 1 s
plus 8 diagonal legs × 1.5 s give 30.0 s for the 14.66 m path (ideal: 29.3 s).

**Measured effect** (480 maps, [performance.md](performance.md#4--vs-8-connectivity)):

- Paths are 23 % shorter overall, by median: from 29.3 % (the obstacle-free optimum
  `1 − √2/2`) at 0 % obstacles down to 12 % at 30 % obstacles.
- Planning is 2–8× slower (median per grid size), because the search has more neighbors,
  more expansions with obstacles, and a general priority queue.

## 10. Differential-drive robot (`--robot-model diff-drive`)

**Why:** most real mobile robots have two driven wheels. Such a robot cannot move
sideways. It moves along its heading and changes heading by turning, so a sharp corner
in the path costs time. The point robot (section 6) ignores this; the
differential-drive model captures it.

**Model.** State `(x, y, θ)`, inputs `v` (linear velocity) and `ω` (angular velocity):

```text
dx/dt = v · cos θ      dy/dt = v · sin θ      dθ/dt = ω
```

`θ = 0` faces `+x`, and `θ` grows toward `+y`, which points down the map like everywhere
in this project. So `θ = 90°` faces down the map. The limits are `0 ≤ v ≤ --speed` and
`|ω| ≤ --max-angular-speed` (default 1.5 rad/s). With `v` and `ω` constant over a
timestep, the equations have an exact solution: a straight line, or an arc of radius
`v/ω`. `integrateUnicycle()` uses it, so no integration error accumulates. A test checks
it against a 200 000-step Euler integration and against splitting a step in two.

**Heading controller (P).** `headingControl()`:

```text
ω = clamp(k · e, −ω_max, +ω_max)       e = heading error, wrapped to (−π, π]
once |e| ≤ tolerance (0.01 rad):  ω = e / dt    (one final step lands exactly on target)
```

- Far from the target heading, the controller is saturated at the angular speed limit.
- Close to it, the proportional term dominates. Each step multiplies the error by
  `1 − k·dt`, so the error decays geometrically.
- `k·dt ≤ 1` is enforced, so the heading never overshoots (default `k = 5 /s`, `k·dt = 0.5`).
- The last step cancels the remaining error exactly, using an `|ω|` well within the limit.
- The θ(t) panel of `robot_position_vs_time.png` shows the result: a straight ramp at
  1.5 rad/s, then the exponential approach.

**Waypoint follower: turn, then drive.** Each timestep:

1. compute the direction to the next waypoint and the heading error `e`;
2. if `|e| > 10⁻⁶ rad`: **turn in place**, with `v = 0` and `ω` from the controller;
3. otherwise **drive straight** to the waypoint, with `ω = 0` and `v ≤ v_max`, clamped so
   the robot never overshoots the waypoint (exactly like the point robot).

Collinear waypoints need no turn, so the robot passes them at full speed. Time is
counted the same way as for the point robot, so turning time simply adds to the travel
time. A test checks that removing the turn-in-place samples leaves exactly the point
robot's trajectory (bit-for-bit), and that the travel time equals the point robot's
plus the turning time.

**Why turn-then-drive, rather than a smoother controller such as pure pursuit?**

- A controller that turns while driving cuts the inside of corners, or swings wide on
  the outside. Either way, the robot's center leaves the planned path.
- The planner's guarantees (footprint clearance, section 8; no corner cutting, section 9)
  only cover the planned segments between cell centers. A wall in this project's maps is
  often just half a cell (0.25 m) from the path.
- Turn-then-drive keeps the center exactly on those segments, so every guarantee still
  holds; the property tests confirm 0 collisions with footprints and diagonals.
- A rotating disc covers no new area, so turning in place needs no collision check.
- The cost is a stop at every corner. Smoother tracking would need a clearance-aware
  path smoother first.

**Example: one 90° corner** (defaults: 1.5 rad/s, `k = 5 /s`, `dt = 0.1 s`):

- 9 saturated steps at 1.5 rad/s take the error from 1.571 to 0.221 rad;
- 5 proportional steps halve it each time, down to 0.0069 rad (below the 0.01 rad tolerance);
- 1 settling step cancels the rest.

That is 15 steps, so the corner takes 1.5 s. A 180° turn takes 2.5 s.

**Measured cost of turning** (default settings; turning time from the simulator's
summary):

| Map | Moves | Path | Point robot | Differential drive | Turns | Turning time |
|---|---|---|---|---|---|---|
| obstacles.csv | 4 | 17.00 m | 34.0 s | 38.5 s | 3 | 4.5 s |
| obstacles.csv | 8 | 14.66 m | 30.0 s | 38.5 s | 8 | 8.5 s |
| corridor.csv | 4 | 34.00 m | 68.0 s | 75.5 s | 5 | 7.5 s |
| corridor.csv | 8 | 31.36 m | 63.5 s | 74.0 s | 9 | 10.5 s |
| maze.csv | 4 or 8 | 30.00 m | 60.0 s | 81.0 s | 14 | 21.0 s |
| obstacles_fine.csv, r = 0.2 m | 4 | 11.25 m | 22.5 s | 36.0 s | 9 | 13.5 s |
| obstacles_fine.csv, r = 0.2 m | 8 | 9.79 m | 20.5 s | 30.0 s | 9 | 9.5 s |

For a differential-drive robot, path *length* is not the whole story:

- On the obstacles map, the shorter 8-connected path has 5 more turns and ends up exactly
  as fast as the 4-connected one.
- On the fine map, it is still clearly faster.

A planner that charges for turns (a heading-aware search, or a turn penalty in the
cost) would optimise travel time directly. That is a possible extension.

## 11. P vs. PID heading control (`heading_experiment`)

**Why:** section 10 uses an ideal actuator, whose turn rate follows the command
instantly. With such an actuator a P controller already lands the heading exactly, and
PID has nothing to add. Real motors respond with a lag, and wheels slip. This study
adds both and compares the two classic controllers on the robot's heading.

**Plant** (`HeadingPlant`):

```text
dθ/dt = ω + d          heading driven by the actual turn rate ω plus a disturbance d
dω/dt = (u − ω) / τ    the motor follows the command u with time constant τ (first-order lag)
|u| ≤ ω_max
```

Defaults: τ = 0.25 s, ω_max = 1.5 rad/s, control period dt = 0.01 s; d = 0.3 rad/s where
a disturbance is used. The command is held constant over each control period, and the
plant uses the exact solution for that case:

```text
ω(t) = u + (ω₀ − u)·e^(−t/τ)
θ(dt) = θ₀ + (u + d)·dt + (ω₀ − u)·τ·(1 − e^(−dt/τ))
```

So results do not depend on an integration step. Tests check it against a 500 000-step
Euler integration and against splitting a step in two.

**Controllers** (`HeadingController`), with error `e = wrap(θ_ref − θ)`:

```text
P:    u = Kp·e
PID:  u = Kp·e + Ki·∫e dt − Kd·dθ/dt        both clamped to ±ω_max
```

- **Derivative on the measurement:** the derivative uses the measured heading rate,
  not the rate of change of the error. When the reference jumps, the error jumps but
  the heading does not, so there is no "derivative kick" (a spike in `u`). A test
  checks the first command after a step.
- **Anti-windup:** while the output is saturated and the error pushes further in the
  same direction, the integral is frozen (conditional integration). Without this, the
  integral would build up during long saturated turns and cause a large overshoot.

**Validating the simulation against theory.** With P control and no saturation, the
closed loop is `τs² + s + Kp`: a damped second-order system with
`ζ = 1/(2√(Kp·τ))` and overshoot `exp(−ζπ/√(1−ζ²))`. The simulated overshoot matches
it:

| Kp (1/s) | ζ | theory | simulated (dt = 1 ms, 0.1 rad step) |
|---|---|---|---|
| 2 | 0.707 | 4.32 % | 4.35 % |
| 4 | 0.500 | 16.30 % | 16.38 % |
| 8 | 0.354 | 30.50 % | 30.67 % |

The steady-state errors also match theory:

- **P under a disturbance `d`:** the heading settles where `Kp·e = −d`, so `e = −d/Kp`.
  That is −0.075 rad (−4.30°) for d = 0.3 rad/s and Kp = 4.
- **P on a ramp at rate `w`:** the heading lags by `w/Kp` = 0.125 rad (7.16°).
- **PID:** the integral term removes both errors.

**Choosing the gains.** Both controllers use the same `Kp = 4` (ζ = 0.5), so the
comparison isolates what the I and D terms add. The PID gains `Ki = 2, Kd = 0.4` come
from a grid over `Ki ∈ [0.5, 4]` and `Kd ∈ [0, 1.2]`. `heading_experiment --sweep`
reprints that grid. The chosen setting is a balanced point:

- it overshoots less than P;
- it settles within the 2 % band in all step tests, including under the disturbance;
- it tracks a ramp well.

Larger `Kd` lowers the overshoot further but makes disturbance rejection slower.
Larger `Ki` speeds up disturbance rejection but raises the overshoot. No setting wins
every metric.

**Metrics** (`computeHeadingMetrics`):

| Metric | Definition |
|---|---|
| overshoot | largest excursion beyond the target, in % of the step |
| rise time | 10 % → 90 % of the step |
| settling time | the time after which the error stays within ±2 % of the step (1.8° for 90°) |
| steady-state error | mean error over the last 1 s |
| tracking error | IAE `∫\|e\| dt` and RMS error |
| control effort | `∫\|u\| dt` (total commanded rotation), `∫u² dt` (energy-like), `Σ\|Δu\|` (command variation: how jerky the command is) |

**Results** (`./build/heading_experiment`; data in
[`heading_control/heading_metrics.csv`](heading_control/heading_metrics.csv)):

| Scenario | Controller | Overshoot | Settling | Steady-state error | IAE (rad·s) | RMS error (rad) | Effort ∫\|u\| (rad) | Σ\|Δu\| (rad/s) |
|---|---|---|---|---|---|---|---|---|
| 90° step | P | 7.29 % | 2.08 s | 0.00° | 1.254 | 0.371 | 2.07 | 2.61 |
| 90° step | PID | **5.38 %** | 2.52 s | −0.02° | 1.295 | 0.371 | **1.84** | **1.87** |
| 90° step + disturbance | P | 13.44 % | never | −4.30° | 1.695 | 0.343 | 4.38 | 3.11 |
| 90° step + disturbance | PID | **10.17 %** | **3.55 s** | **−0.05°** | **1.221** | 0.337 | 4.25 | **2.30** |
| ramp, 0.5 rad/s | P | – | – | 7.16° | 1.250 | 0.126 | 5.00 | 0.87 |
| ramp, 0.5 rad/s | PID | – | – | **0.04°** | **0.349** | **0.059** | 5.12 | 0.79 |
| planned-path profile | P | – | – | 0.00° | **4.242** | 0.306 | 10.50 | 29.63 |
| planned-path profile | PID | – | – | 0.06° | 4.490 | 0.308 | **8.92** | **24.78** |

The planned-path profile takes the 8-connected A* path on `maps/obstacles.csv` (1, 1) →
(18, 18). Each straight run's direction is held for the time it takes to drive at
0.5 m/s, so the reference is a series of 45° and 90° steps over 31 s.

![P vs. PID metrics](images/heading_metrics.png)

**Interpretation.**

- **Disturbances and ramps are where PID clearly wins.**
  - P cannot remove a constant disturbance; it settles 4.3° off target and never enters
    the 2 % band. PID settles in 3.55 s with a 0.05° residual.
  - On a ramp (following an arc), P lags by a constant 7.2°. PID removes the lag: tracking
    error 3.6× lower in IAE and 2.1× lower in RMS.
- **On a plain step, PID is gentler but slower to settle.**
  - The derivative term damps the motion, giving less overshoot (5.4 % vs. 7.3 %), 11 %
    less control effort and 28 % less command variation.
  - The integral term then slowly removes the last part of the error, so PID settles
    0.4 s later than P (2.52 s vs. 2.08 s).
- **On the path profile (frequent steps, no disturbance), P has slightly lower tracking
  error** (IAE 4.24 vs. 4.49). The integral term only helps against persistent errors,
  and there are none here. PID still uses 15 % less control effort and produces a smoother
  command.
- **Both controllers saturate at the start of large turns** (about 10 % of the time in
  the step runs). There, both simply turn at the limit and behave identically (see
  `heading_step.png`). The differences appear only in the final approach.

**Recommendation.** For the navigation follower (section 10), a heading without
disturbance and stops between turns, the simple P controller is adequate. On a real
robot with motor lag and wheel slip, use PID (or at least PI). It is the only one of
the two that holds the heading against a persistent disturbance and that can track a
curving path without a constant lag.

**Scope.** This is a controlled study of the heading loop alone. The navigation
simulator keeps the ideal actuator of section 10 (`--robot-model diff-drive`), so its
paths, collision guarantees and earlier results are unchanged.

## 12. Simulated 2D LiDAR (`--lidar-beams N`)

**Why:** a LiDAR is a mobile robot's main exteroceptive sensor. It measures the distance
to the nearest obstacle in many directions. Simulating one gives later extensions
(localization, dynamic obstacles) realistic measurements to work with. It also shows,
from the robot's own point of view, how close it gets to obstacles along its route.

**Sensor model.**

- Beams are evenly spaced over the field of view (`--lidar-fov`, default 360°),
  centred on the robot's heading.
  - The point robot has no heading, so its beams use the world frame.
  - A full circle has no duplicate beam at ±180°.
- Each beam reports the distance to the first obstacle within `--lidar-range`
  (default 5 m), or **no return** if there is none (stored as the maximum range with
  `hit = 0`).
- The area outside the map counts as a wall, as in the planner and footprint checks.
- One scan is taken at every trajectory sample. The LiDAR only observes; it never
  changes the motion.
- Optional zero-mean Gaussian noise (`--lidar-noise`, seeded with `--lidar-seed`) is
  added to the returns and clamped to `[0, max range]`.

**Ray casting: exact grid traversal** (`castRay`, after Amanatides & Woo, 1987).

- Instead of sampling points along the beam, the ray is followed from one cell boundary
  to the next. Each step crosses into the neighbouring cell through the nearer of the
  next vertical and the next horizontal boundary:

  ```text
  t_max_x, t_max_y   = distance along the ray to the next vertical / horizontal boundary
  t_delta_x, t_delta_y = distance between consecutive boundaries (res / |cos|, res / |sin|)
  repeat: step into the next cell across the nearer boundary; if it is occupied (or off
          the map), the range is that boundary's t; advance that t_max by its t_delta
  ```

- Every cell the ray passes through is visited exactly once, in order. The range is the
  exact distance to the obstacle face, with no sampling error.
- The cost is proportional to the number of cells crossed, not to the range. Measured:
  about 75 ns per ray, so one run of 386 scans × 72 beams takes 2.2 ms.

**Why exact matters, with a real example.** An early version of the test compared the
traversal with brute-force sampling every 1 mm. On one random ray the two disagreed:

- The ray enters an occupied cell at 0.15025 m but crosses into a free cell 0.66 mm later.
- Sampling every 1 mm stepped over that sliver and reported the obstacle 0.8 m away.
- The traversal cannot skip a cell, so the test now uses an independent exact
  reference: a slab intersection of the ray with every occupied cell and with the map
  border. The traversal matches it to 1e-9 m on 2400 random rays. Sampling is only
  checked one way: it may report an obstacle later, never earlier.

**Corners.** A beam can pass exactly through a grid corner; this happens for every 45°
beam from a cell centre. When it does, the ray touches the two cells beside the corner
and the one diagonally beyond it. If any of them is occupied, that is a hit at the
corner. This matches the rule that touching an obstacle counts, used by the
corner-cutting check (section 9).

**Output.**

- `lidar_scans.csv` has one row per beam and scan: `time, beam, angle, range, hit`,
  where the angle is in the world frame.
- The console summary reports the share of beams that returned and the **closest
  return** over the run. That is the robot's minimum clearance as its sensor sees it:
  0.25 m on the default maps, where paths run along cell centres next to walls.
- `visualize_map.py` saves `lidar_scan.png`: the scan with the closest return, drawn on
  the map and as a polar range plot.
- The animation draws each frame's latest scan.

**Limitations.**

- No beam divergence, intensity, reflectivity or multipath: each beam is an ideal ray.
- The noise is additive Gaussian on the range only, with no dropouts or angular error.
- The scan rate is tied to the simulation timestep.

## 13. EKF localization (`--ekf`)

**Why:** a real robot does not know its pose. Wheel odometry tells it how far it drove
and turned, but every small error is integrated, so the pose estimate drifts without
bound (dead reckoning). An Extended Kalman Filter (EKF) fuses that odometry with
measurements of the known map, here the LiDAR ranges of section 12, and keeps both an
estimate and an honest measure of its uncertainty (a covariance).

**State:** the pose `(x, y, θ)` and its 3 × 3 covariance `P`. The filter runs
alongside the simulation; the robot is still controlled with its true pose, so turning
the EKF on never changes the motion.

**Odometry model.** Between two samples, the turn-then-drive robot first turns by `Δφ`,
then drives `Δs` straight ahead (it never does both at once, and the residual turn of at
most 1e-6 rad absorbed when driving starts is applied first):

```text
θ' = θ + Δφ,    x' = x + Δs cos θ',    y' = y + Δs sin θ'
```

From the true increments this reproduces the simulated trajectory to 1e-9 m (tested).
The simulated odometry reports the increments with Gaussian errors whose variance grows
with the motion, as encoder errors accumulate per metre and per radian:

```text
var(Δs) = k_dist² |Δs|                      --odom-noise-dist  (default 0.05 m/√m)
var(Δφ) = k_turn² |Δφ| + k_drift² |Δs|      --odom-noise-turn  (0.05 rad/√rad)
                                            --odom-noise-drift (0.02 rad/√m)
```

`k_drift` is heading drift while driving straight (e.g. unequal wheel radii). With the
defaults, dead reckoning ends 0.27–2.4 m off after the 17 m demo route (obstacles map,
4-connected), depending on the seed (`--odom-seed`; seeds 0–5).

**Prediction.** The estimate is moved with the measured increments, and the covariance
is propagated through the motion model's Jacobians:

```text
P' = F P Fᵀ + V M Vᵀ,     F = ∂(x', y', θ')/∂(x, y, θ) = [1 0 −Δs sin θ'; 0 1 Δs cos θ'; 0 0 1]
                          V = ∂(x', y', θ')/∂(Δs, Δφ)  = [cos θ' −Δs sin θ'; sin θ' Δs cos θ'; 0 1]
                          M = diag(var(Δs), var(Δφ))
```

**A pitfall found by Monte Carlo testing.** The first version evaluated `M` at the
*measured* increments, as is common. Over 200 runs the filter's heading variance was
35 % larger than the actual spread of its errors (mean NEES 2.4 instead of 3). Each
noise source alone was fine; only turn noise and drift together showed it. The cause:
while driving straight, the measured turn is pure drift noise, a few milliradians, and
`k_turn² |Δφ|` turned that noise into turn variance that was never there. `M` is now
evaluated at the *commanded* increments (`v dt`, `ω dt`), which the controller knows
exactly, and the mean NEES is 3.2.

**Correction with the LiDAR.** For every beam with a return, the expected range `r̂` is
cast from the estimated pose on the known map (section 12). The ray caster also reports
which kind of cell face it hit. On a vertical face `x = X` the range is
`r = (X − x) / cos φ`, with `φ = θ + beam offset`, and on a horizontal face
`r = (Y − y) / sin φ`. That gives the exact measurement Jacobian:

```text
vertical face:    H = [ −1/cos φ,   0,          r tan φ ]
horizontal face:  H = [  0,        −1/sin φ,   −r / tan φ ]
```

(The tests check it against finite differences of the ray caster at 500 random poses.)
Beams are skipped where there is nothing to linearize: a predicted corner hit, and beams
without a measured or predicted return. Grazing beams (where `1/cos φ` explodes) are
kept and weighted by their own linearization variance, by default (`--ekf-grazing
model`, section 17); the classic rule dropped every beam beyond 78.46°
(`--ekf-grazing cutoff`). A beam whose innovation `ν = z − r̂` is larger than `--ekf-gate`
(default 3) standard deviations `√(H P Hᵀ + R)` is rejected as an outlier. Typically the
real beam passed an obstacle edge that the predicted one hit, or the other way round.
About 0.25 % of the beams are rejected on the demo route.

With the classic model, the accepted beams are fused in one batch update in
**information form**:

```text
P⁺ = (P⁻¹ + Σ Hᵢᵀ Hᵢ / R)⁻¹,     x⁺ = x + P⁺ Σ Hᵢᵀ νᵢ / R,     R = --ekf-range-sigma² (0.05 m)
```

This is the standard Kalman update of all the beams stacked (tested against
`K = P Hᵀ / (H P Hᵀ + R)` for one beam), but it only inverts 3 × 3 matrices, however
many beams there are. The other models (section 17) fuse the beams one at a time, all
linearized at the prior; without extra states that equals the batch update to 1e-9.

**Starting pose.** The filter, and dead reckoning, start from the true start pose plus
an error drawn from the initial uncertainty (1 cm, 0.01 rad). Starting exactly at the
truth while claiming 1 cm of uncertainty would make the first samples look
over-cautious and bias the consistency check below.

**Consistency (NEES).** If the covariance is honest, the normalized estimation error
squared `NEES = eᵀ P⁻¹ e` (`e` = true − estimated pose, heading wrapped) averages 3,
the state dimension. Below 3 the filter is too cautious, above 3 overconfident. Measured
on the obstacles route:

| Setting | Mean NEES |
|---|---|
| Prediction only, 200 runs | 3.19 |
| 72 beams, 5 cm range noise, `--ekf-range-sigma 0.05`, 100 runs | 3.16 |
| 72 beams, exact ranges, default sigma 0.05 m (one run) | 0.67 |

The last line is cautious on purpose: the default assumes 5 cm of range noise, while
the default LiDAR is exact. Set `--ekf-range-sigma` to the actual noise for a consistent
filter.

**Results** (obstacles map, 72 beams, defaults, one run): position error RMS 3 mm and
at most 1.4 cm, against 0.53 m RMS and 1.14 m at the end for dead reckoning; heading
RMS 0.05° against 4.8°. Over odometry seeds 0–5, the EKF stays at 3 mm RMS (at most
2 cm) while dead reckoning ranges from 0.16 to 0.94 m RMS. (With the classic 78.46°
cutoff: 4 mm and 0.06°.)

**Cost.** About 0.1 µs per beam (a ray cast plus a rank-one information update): 7.5 µs
per timestep with 72 beams, 2.9 ms for the whole run.

**Output.**

- `localization.csv`: per sample, the true, dead-reckoning and EKF poses, the
  covariance, the beams fused and rejected, and the NEES.
- The console summary compares EKF and dead-reckoning errors and reports the mean NEES.
- `visualize_map.py` saves `localization.png` (the three trajectories on the map, with
  3σ ellipses when they are large enough to see) and `localization_errors.png` (position
  error on a log scale, and the x, y and heading errors inside the EKF's ±3σ bounds).
- The animation shows the dead-reckoning trail and pose, and the EKF estimate with its
  3σ ellipse.

**Limitations.**

- The map is known and static. This is localization, not SLAM.
- By default, range measurements of one scan are treated as independent, which is
  right for this simulator's own world (the map is the truth, and the LiDAR noise is
  white). Correlated errors (a LiDAR bias, or a map that is a rasterization of the real
  walls) are modeled by the measurement models of section 17.
- Requires the differential drive (`--robot-model diff-drive`): the odometry model
  measures turns and distances driven.
- The estimate does not feed back into the control. Closing that loop is part of the
  "realistic actuator" future improvement.

## 14. Dynamic obstacles and replanning (`--dynamic-obstacles FILE`)

**Why:** real environments contain people and other robots. A path planned on a static
map gets blocked, and the robot must notice that, find another way or wait, and never
run into anything.

**Obstacle model.**

- Each obstacle is a disc that moves at constant speed back and forth along a route
  through cell centres (one line per obstacle in the file:
  `radius_m, speed_mps, x0, y0, x1, y1, ...`).
- Its position is a closed-form function of time (arc length `speed · t` folded back
  and forth along the route).
- The route must stay in free cells. Obstacles do not avoid the robot.

**Exact contact test.** Within a timestep the robot centre moves in a straight line, as
recorded in the trajectory and drawn by the animation. The obstacle centre is piecewise
linear: it can turn a corner or reverse mid-step. The step is split at those times, so
both move linearly in each piece, and the closest approach of two linear motions has a
closed form. Contact means a distance below obstacle radius + robot radius. The test
also catches an obstacle that passes through the robot between two samples (tested
with an obstacle that is 0.5 m away at both step ends). Against 20 000-point sampling,
the exact minimum is never larger and always within 0.6 mm (300 random cases).

**What the robot knows.** The robot does not know the routes.

- It sees an obstacle within `--sensing-range` (3 m, measured to the obstacle's edge)
  when the line to its centre crosses no static obstacle (the ray caster of section 12).
- From two consecutive sightings it estimates the obstacle's velocity.
- It blocks every planning cell whose centre lies within obstacle radius + robot radius
  + `--replan-margin` (0.25 m) of the obstacle's predicted sweep. The sweep runs from
  the current position to where the obstacle will be after `--prediction-horizon` (2 s)
  at the estimated velocity.

**Decision rules, every step.**

1. **Blocked path → wait, then detour.** If a blocked cell lies on the rest of the
   path, the robot keeps following the free part and waits on the last free waypoint.
   It replans a detour only if the path is still blocked after `--replan-patience`
   (6 s). It replans at once if its own cell, or the cell it is entering, becomes
   blocked.
2. **Replanning** uses one `AStarPlanner` object for the whole run, so every replan
   reuses its buffers (Phase 6). It plans from the current cell, or from the waypoint
   being driven to. If that waypoint is blocked, the robot **retreats**: it plans from
   the waypoint it came from and drives back to it first. The goal cell is never
   counted as blocked, because it cannot be avoided, only timed.
3. **Failed replan.** If there is no path yet, the robot keeps following the old path
   while its next waypoint is free, backs off if the one ahead is blocked, and
   otherwise waits and retries.
4. **Look-ahead gate.** Before driving, the robot predicts its own motion along the
   path for the horizon, and the seen obstacles at their estimated velocities. If that
   comes within the margin of an obstacle and standing still is safer, it waits. This
   is what lets it cross *behind* a moving obstacle instead of in front of it.
5. **Differential drive.** In dynamic runs it drives *backwards* to a waypoint behind it
   (retreats included) instead of turning around. If turning in place, which is
   standing still, is predicted to end in contact, it takes an **evasive step** forward
   or backward. It picks the option that delays the contact longest, then the one with
   the larger gap.

**How the rules were found.** I stress-tested each version on 300 random scenarios
(three obstacles patrolling random lanes of the obstacles map) and analysed every
failure. Each failure class led to one rule:

| Failure seen | Cause | Rule added |
|---|---|---|
| Head-on collision after stopping mid-cell | waiting where the obstacle was heading | 2 (retreat) |
| Planning straight into an obstacle's path | only its current position was blocked | velocity estimate + predicted sweep |
| Robot shuttling beside a patrolling obstacle until the timeout | each reversal of the obstacle flipped the best detour | 1 (patience) and 4 (look-ahead gate) |
| Frozen in an obstacle's lane after a failed replan | "no path" made it stop | 3 (keep going) |
| Hit while turning around (differential drive) | a 180° turn takes about 3 s | 5 (back up; evasive step) |
| Every plan failing while an obstacle swept the goal | goal cell counted as blocked | 2 (goal never blocked) |
| Evasive step moving towards the obstacle | contact was unavoidable either way, and the tie went to the larger gap | 5 (later contact first) |

Two tuning findings: patience alone did not help until the look-ahead gate existed
(2 s was worse than none), and results plateau between 6 and 8 s of patience.

**Results** on 500 *new* random scenarios (a different seed from the 300 used for
tuning), 3 moving obstacles each, 600 s limit:

| Robot | Reaches the goal | Collisions | Timeouts (safe waiting) |
|---|---|---|---|
| Point, blind (does not react) | 355 (71 %) | 145 | 0 |
| Point, replanning | 491 (98.2 %) | 0 | 9 |
| Differential drive, blind | 357 (71 %) | 143 | 0 |
| Differential drive, replanning | 494 (98.8 %) | 2 | 4 |

The remaining failures are hard cases for any purely reactive planner:
- obstacles closing in from two sides in a corner;
- an obstacle patrolling the lane next to the goal;
- a short patrol across the only passage, which makes the robot wait until the
  timeout.

Avoiding these needs knowledge of where obstacles will be: space-time planning, or
learned routes.

**Example** (`maps/obstacles_dynamic.csv`, point robot): it waits 9.5 s in total for
the corridor-crossing and corner obstacles, replans 3 times (0.009 ms of planning in
total), reaches the goal in 60.7 s instead of 34 s, and its closest gap to an obstacle
is 0.49 m.

**Cost.** About 2.5 µs per step with three obstacles (visibility, blocking, gate), plus
a few µs per replan on this map. Static runs are untouched: without
`--dynamic-obstacles` every output is byte-identical to before (400 random maps).

**Limitations.**

- Constant-velocity prediction: the robot cannot anticipate an obstacle's reversal.
- Reactive, not optimal: detours are shortest paths on the current blocking, not
  fastest routes through space and time.
- Obstacles never yield, and they ignore each other.
- The differential drive's evasive step only moves along its heading.

## 15. Why NO_PATH? Minimum ghost cells and targeted re-sensing (`ghost_experiment`)

**Problem.** When the disc-robot planner returns `NO_PATH`, find the smallest set of
occupied cells that, if they were sensor ghosts (false obstacles), would make the goal
reachable. Then re-sense only those cells. This root-causes a planning failure to its
minimal set of culprits, and turns "the map says no" into "look again at these cells".

**Why it is hard.** For a point robot it is a shortest path in which entering an
occupied cell costs 1. For a disc robot each occupied cell blocks an identical
rounded square of neighbouring cells, and these footprints overlap. Removing one cell
can free nothing, because its neighbours still block the same cells (tested: a
single-lane corridor where two adjacent obstacles must both go). This is *minimum
constraint removal* (MCR; Hauser 2014), which is NP-hard in general. The lattice
version, with identical footprints exactly as obstacle inflation produces them,
appears not to have been studied directly; its complexity is open.

**Formulation.** Each cell `c` has a *cover* `C(c)`: the occupied cells that block it
on the inflated map (the same rule as `inflateObstacles`, distance below the radius
minus 1e-9). Map-border blocks can never be removed. Removing a set `R` frees exactly
the cells whose cover lies in `R`, so the goal is reachable after the removal iff some
path's covers all lie in `R`. The minimum explanation is therefore the path minimizing
the weight of the union of its covers:

```text
minimize  w(C(p0) ∪ C(p1) ∪ ... ∪ C(pn))   over grid paths start = p0 ... pn = goal
```

For an 8-connected diagonal move, the two side cells' covers join the union (no
corner cutting). With `w = 1` the result is the fewest cells; with per-cell costs, the
most likely explanation.

**Exact algorithm** (`findGhostCells`). A best-first search over labels
`(cell, S)`, where `S` is the union of covers so far.

- **Ordering:** labels are ordered by `f = max(w(S), H(cell))`. `H` is the
  bottleneck (minimax) cover weight from the cell to the goal, computed once by a
  Dijkstra-like pass. Every path from the cell must cross a cover at least that heavy,
  and the union contains it, so `f` never overestimates. It is consistent, so the
  first goal label popped is optimal.
- **Dominance:** a label is dropped if another label at the same cell has a subset
  `S' ⊆ S`. A 64-bit signature rejects most subset tests in one instruction.
- **Ties:** broken goal-directed (nearest to the goal first), which cut the labels by
  2–5× on dense maps.
- **All optimal explanations:** after the first optimum the search continues across
  the plateau `f = optimum`, listing every inclusion-minimal optimal set.
- **Limits:** a cost bound and a label limit stop runaway searches, reporting the best
  proven lower bound.

**Verification.**

- **Covers:** they reproduce `inflateObstacles` cell for cell on 150 maps, both as
  they are and after random removals.
- **Minimum count:** it equals brute force (`planWithFootprint` on every subset, up to
  3 cells) on 1500 random maps. These cover the point and disc robots, 4- and
  8-connectivity, and two resolutions.
- **All optimal sets:** the listed sets equal brute force's on 306 maps.
- **Weighted and unit optima:** both equal an exhaustive search over *every* subset
  (removals of any size) on small maps.
- **Mutation testing:** all 7 deliberate solver bugs are caught (section "Mutation
  checks" in [testing.md](testing.md)).

The original brute-force experiment (40×40 cells at 0.25 m, 0.3 m robot, 100 seeds
per density, the same seeds) is reproduced exactly at all four densities.

- **Same counts:** at 6 % density, 31 failures, of which 23 need 1 cell and 8 need 2,
  with 4.0 single-cell options on average.
- **Faster:** 8.6 s in total, listing every optimal explanation, against 1 min 44 s
  for the brute force.
- **More information:** where the brute force could only report "3 or more" (8 maps
  at 8 %, 59 at 10 %), the exact search also gives the size.

**Scaling (empirical).** 100 seeds per setting, NO_PATH maps only:

| Map | Density | Radius | Explanation (median / max cells) | Solver ms (median / max) |
|---|---|---|---|---|
| 40×40 | 4 % | 0.3 m | 1 / 1 | 1.6 / 4.0 |
| 40×40 | 6 % | 0.3 m | 1 / 2 | 1.4 / 4.9 |
| 40×40 | 8 % | 0.3 m | 1 / 4 | 1.3 / 8.6 |
| 40×40 | 10 % | 0.3 m | 3 / 6 | 1.8 / 2163 |
| 40×40 | 12 % | 0.3 m | 5 / 8 | 3.6 / 355 |
| 40×40 | 15 % | 0.3 m | 8 / 13 (2 hit the 2 M-label limit) | 46 / 61 753 |
| 40×40 | 6 % | 0.6 m | 6 / 10 | 3.4 / 87 |
| 120×120 | 6 % | 0.3 m | 1 / 2 | 105 / 234 |

Where brute force can still check (explanations of 1–2 cells), it takes 7.7 ms per
map against 1.4 ms. It grows as (occupied cells)^k, while the exact search stays in
milliseconds up to 5–6 cells. Beyond that the cost rises steeply with the size of the
explanation, as an NP-hard problem leads one to expect; the slowest case took about
62 s. Larger maps cost more mainly through the wide plateau of equally cheap labels.
`docs/images/ghost_scaling.png` plots all of this.

### Closing the perception loop

**World and sensor.** The true map plus *ghost sources*: free cells that return a
false hit to beams arriving within an angular sector (a reflection, a glass pane).

- **Hits and passes:** an in-sector beam hits a ghost with probability q = 0.8, and an
  out-of-sector beam passes through. Ghosts are persistent and view-dependent:
  scanning again from the same places brings them back.
- **Beams:** LiDAR beams are traced cell by cell. A real obstacle misses 2 % of beams,
  and 1 % of beams return a spurious short range.
- **Map:** a log-odds occupancy grid (+0.85 for a hit, −0.4 for a pass). Unknown space
  counts as free.

**Ghost evidence.** Plain log-odds treat a pass through a cell as weak evidence. But a
beam can only pass through a ghost, never through a solid obstacle, so for the
question "real or ghost?" one pass outweighs many hits. Two hypotheses give the
removal cost of a believed-occupied cell:

```text
log P(real)/P(ghost) = prior (3) + hits · log((1 − miss)/q) + passes · log(ε)
                     ≈ 3 + 0.2 · hits − 3.9 · passes          (ε = 0.02: seeing through a real cell)
```

**Recovery strategies.** These are compared from the same belief and the same noise:

| Strategy | What it does |
|---|---|
| **Most likely** (evidence weights) | explain with the minimum-weight ghost set, re-sense those cells, decide them by the same evidence, replan |
| Most likely (log-odds weights) | the same with the plain occupancy log-odds as costs, and log-odds deciding |
| Fewest cells | minimum number of cells, re-sense, replan |
| Re-sense every obstacle | look again at every believed-occupied cell |
| Clear map, re-scan | reset the map and re-scan from every viewpoint (ROS costmap-clearing recovery) |
| Delete explanation unseen | remove the most likely explanation without looking |

**Active re-sensing.** The robot may drive to any cell it believes reachable. For each
suspect cell it picks the viewpoint, in range and in line of sight, whose bearing
differs most from the bearings that produced the hits there, preferring short drives.
It then aims 5 beams at the cell.

**Results.** 300 NO_PATH cases with 3 ghost sources next to the true route, 10 rounds
at most:

| Strategy | Thorough mapping: recovered | median beams | Sparse mapping: recovered / unsafe | real cells wrongly cleared |
|---|---|---|---|---|
| **Most likely** (evidence) | **300 (100 %)** | **5** | 259 / 40 | **0** |
| Most likely (log-odds) | 251 | 10 | 189 / 35 | 0 |
| Fewest cells | 62 | 5 | 54 / 13 | 0 |
| Re-sense every obstacle | 222 | 645 | 187 / 13 | 0 |
| Clear map, re-scan | 149 | 6840 | 115 / 34 | 36 |
| Delete explanation unseen | 300 | 0 | 243 / 57 | 19 |

- **Thorough mapping:** viewpoints every 2 m. The most likely explanation recovered
  every case, usually by re-sensing one cell with 5 beams after a 6 m drive.
- **Sensitivity:** in variations (240° ghost sectors, 6 ghosts, 6 % density;
  `docs/ghost/`) it again recovered 100/100 each time. Log-odds weighting dropped to
  30–83 % and clearing the map to 18–48 %.
- **Sparse mapping:** viewpoints every 4 m. Unseen obstacles count as free, so every
  strategy can produce unsafe paths through obstacles never observed; that is a
  mapping limitation, not a recovery one. Re-sensing never cleared a real obstacle.
  Deleting without looking cleared 19, and clearing the map 36.
- **Fewest cells does badly:** with equal costs it keeps choosing well-observed real
  cells. Confidence is what makes the explanation useful.

**What failed first, and why.** Each of these was found by tracing failed runs:

| Observation | Cause | Fix |
|---|---|---|
| Re-sensing the right cell never cleared it | only viewpoints the robot had visited, all inside the ghost's sector | active re-sensing from any reachable cell, at the most different bearing (hit bearings are stored per cell) |
| The same confirmed real cell chosen round after round | log-odds clamped at ±6: 60 hits looked as uncertain as 8 | clamp raised to ±30 |
| A ghost seen through 10 times stayed "occupied" | plain log-odds count a pass as only half a hit | the two-hypothesis evidence, for both the costs and the decision after re-sensing |
| Most strategies "unsafe" | unknown space treated as free after sparse mapping | thorough-mapping scenario for the main result; sparse kept as the stress case |

**Limitations.**

- **Simulated sensor:** the ghost model (sector, hit probability) is an idealized
  view-dependent artifact. Ghosts that look solid from every reachable direction
  cannot be told apart by any sensor, and the robot then gives up, which is correct.
- **Simplified driving:** the drive distance is a straight-line estimate, and the
  robot's re-sensing moves are not simulated with the motion model.
- **Independence:** cells are assumed independent (an explanation's probability is
  the product).
- **Complexity:** the solver is exact and exponential in the worst case. The
  polynomial-time question for the lattice version stays open (not attempted).

## 16. Grid alignment: when the same doorway is passable or not (`alignment_experiment`)

**Problem.** An occupancy grid is laid over a continuous world at some rotation and
sub-cell offset. The planner then inflates obstacles at cell centres (section 8). Both
steps quantize:

- obstacles are rounded to whole cells;
- configuration space is sampled at cell centres.

So the same door can come out passable or blocked depending only on where the grid
happened to start. The README listed cell-centre inflation as a known limitation. This
section measures the effect, bounds it, removes it, and shows what it means for
problem 1: some "ghosts" are not sensor errors but discretization artifacts.

**Setup.**

- **World:** continuous, made of convex polygons (`geometry2d.hpp`). A 4 m room has a
  dividing wall 0.2 m thick with one doorway of width w.
- **Robot and grid:** radius r = 0.3 m (the door is passable iff w ≥ 0.6 m); cells of
  0.1 m.
- **Alignments:** the grid is placed at a random rotation and sub-cell offset
  (`placeGrid`).
- **Grid rules compared:**
  - *any overlap* (a cell is occupied if it shares any area with an obstacle; the
    conservative costmap rule);
  - *cell centre inside*;
  - *majority coverage*;
  - a *LiDAR log-odds map* built from exact scans (720 beams, 6 viewpoints).

**The effect** (`docs/images/alignment_doorway.png`; 60 random alignments for each
width from 0.50 to 1.10 m):

| Method | Widths where the alignment decides | Band | Passable when blocked | Blocked when passable |
|---|---|---|---|---|
| Grid, any overlap | 0.75 – 0.99 m | 25 cm | 0 | 1719 of 3060 |
| Grid, cell centre inside | 0.61 – 0.85 m | 25 cm | 0 | 939 |
| Grid, majority coverage | 0.61 – 0.84 m | 24 cm | 0 | 867 |
| Grid, LiDAR log-odds | 0.70 – 0.91 m | 22 cm | 0 | 1389 |
| **LiDAR points + 2.5 cm lattice** | **0.64 – 0.66 m** | **3 cm** | **0** | 362 |
| Exact geometry + 2.5 cm lattice (reference) | 0.62 – 0.63 m | 2 cm | 0 | 205 |

With any overlap, a door 14 cm wider than the robot is never passable. One 25 cm wider
passes for only 48 % of the alignments, and one 30 cm wider for 43 %. Two axis-aligned
grids 2.6 cm apart already disagree on a 0.75 m door
(`docs/images/alignment_example.png`). Even the less conservative rules give 45–62 %
for doors 15–20 cm wider than the robot.

**Bounds** (cell size s, robot radius r, any overlap):

- **Never unsafe.** The occupied cells contain every obstacle, so a cell centre's
  distance to them is at most its true clearance. A cell the grid calls free is truly
  free. Measured: no unsafe answer in 3660 doorway and 1200 clutter cases.
- **Always passable once w ≥ 2r + 3√2·s** (1.02 m here). Occupied cells reach at most
  s√2 beyond an obstacle, so a centre with true clearance ≥ r + s√2 is free. In the
  doorway those centres fill a strip of width w − 2r − 2√2·s. Any strip at least √2·s
  wide (more precisely s(|cos θ| + |sin θ|)) contains a 4-connected chain of cell
  centres across it. Measured: passable for every alignment from 1.00 m.
- **On an axis-aligned grid,** the opening must hold n ≥ 2⌈r/s − ½⌉ + 1 = 7 whole
  free cells, for a centre at least r from both sides. So 0.70 m is the least width
  that can pass, and 0.80 m passes at any offset. Between those, the offset decides
  (the example figure: 0.75 m).

The centre and majority rules can also mark partly covered cells free. In principle
that could make a grid pass a door narrower than 2r; it never happened in these runs.

**The fix: plan on the LiDAR's points, not the grid** (`planOnClearance`). The hit
points are continuous measurements of the surfaces, independent of the grid.

1. **Join consecutive hits.** Consecutive hits of one scan closer than 2r are joined
   into segments. A disc cannot pass between points closer than 2r anyway, so joining
   never closes a usable passage.
2. **Plan on a fine lattice.** The lattice spacing is h = s/4. A node is free when it
   is at least r from every segment and from the map border.
3. **Margin at chain ends.** Where a chain of joined hits ends, the true surface may
   continue unseen up to the next beam (a corner between two beams). The end point
   gets a margin of range × beam spacing.
4. **Unobserved space.** The disc must also stay at least r from every cell the LiDAR
   never observed (no beam passed through or hit it), because faces no viewpoint saw
   may hide obstacles. Obstacle interiors are never observed, but they lie behind
   their observed faces, so this never blocks more than the true geometry.

The answer depends on the alignment only through h, not through s. The tolerance band
is [2r, 2r + 2·R·Δθ + √2·h], where R is the range and Δθ the beam spacing. Measured: 3
cm, against 22–25 cm for every grid rule.

**What failed first.** Each fix was found by checking against the true geometry:

| Observation | Cause | Fix |
|---|---|---|
| Point plans through obstacles the LiDAR never saw (true clearance 5 cm) | unseen box faces leave no points | keep r from unobserved cells |
| Point plans 6–8 mm too close to box corners | a corner between two beams | margin of range × beam spacing at chain ends |
| The point planner never left its start | a beam's own starting cell was never marked as observed | count the origin cell as passed |
| A wide door "blocked" by a single point near the robot | a spurious return, never re-crossed within 5 mm | outlier filter: at least 2 hits within 5 cm |

**Random clutter** (100 rooms with 6 rotated boxes, 12 alignments each, ground truth
from a 5 mm lattice with two-sided thresholds):

| Method | Worlds whose answer flips with alignment | Wrong answers | Passable when blocked |
|---|---|---|---|
| Grid, any overlap | 29 / 100 | 392 / 1200 | 0 |
| Grid, cell centre inside | 22 | 187 | 0 |
| Grid, majority coverage | 22 | 172 | 0 |
| Grid, LiDAR log-odds | 23 | 284 | 0 |
| **LiDAR points + lattice** | **5** | 61 (all cautious) | **0** |
| Exact geometry + lattice | 2 | 15 | 0 |

The point planner's remaining errors are cautious "blocked" answers caused by space
the 5 viewpoints never saw: a limit of mapping coverage, not of alignment.

### Artifact or ghost? (connecting to problem 1)

Problem 1's ghost evidence counts beams that pass through a **cell**. With continuous
geometry, cells on an obstacle's edge are partly free: beams pass through their free
part as well as hit the wall. Cell-level evidence can therefore call a real wall cell
a ghost and "recover" through the wall.

The point map uses **point-level evidence** instead (`PointMap`):

- **Contradiction:** a hit is contradicted only by a beam that passes within 5 mm of
  that hit and continues at least 20 cm beyond it. Two contradictions drop it.
- **Why the radius is tiny:** beams graze the corners of door jambs, and a wider
  radius would erode the corners, making doors look wider than they are.
- **Spurious returns:** beams almost never cross a random isolated hit that precisely,
  so these are removed by the outlier filter instead.

**Point-aware recovery** (`recoverWithPoints`) when the grid says NO_PATH:

1. Plan on the points. If a path exists, the failure was a **discretization
   artifact**: no sensing is needed.
2. Otherwise the measurements themselves block the way. Explain with the minimum ghost
   set (section 15) on the cells that hold points, weighted by their evidence.
3. Re-sense those points from the most different bearing, with two beams aimed exactly
   at each suspect hit (up to 8 per cell), and repeat.

Grid NO_PATH cases from noisy LiDAR maps (720 beams, misses and spurious returns):

| Kind (60 grid NO_PATH cases each) | Pipeline | Recovered | Unsafe | Gave up | Called an artifact | Median beams |
|---|---|---|---|---|---|---|
| Discretization artifact (door 1–15 cm wider than 2r) | grid-only (problem 1) | 6 | **47** | 7 | – | 17.5 |
| | **point-aware** | 36 | **0** | 24 | 36 | **0** |
| Sensor ghost (strong reflection in a door 30–60 cm wider) | grid-only | 60 | 0 | 0 | – | 5 |
| | **point-aware** | **60** | **0** | 0 | 1 | 47 |
| Real block (door 1–15 cm narrower than 2r) | grid-only | 0 | **25** | 35 | – | – |
| | **point-aware** | 0 | **0** | **60** | 0 | – |

- **Grid-only recovery is unsafe** on 47 of 60 artifacts and 25 of 60 real blocks. It
  re-senses the edge cells of the door jambs, sees beams pass through their free part,
  calls them ghosts, and plans through the wall.
- **Point-aware recovery is never unsafe.** It called 36 artifacts with no sensing,
  cleared every ghost, and refused every real block.
- **Where it gave up on artifacts:** only doors inside its tolerance band. Below 0.66 m
  it recovered 3 of 27; from 0.66 m it recovered all 33. Those are cautious answers,
  not errors.
- **Aiming at the hits:** the first version re-sensed each cell's centroid with
  jittered beams and cleared only 39 of 60 ghosts. Aiming two beams exactly at each
  suspect hit cleared all 60.

**Limitations.**

- **Planning with points needs the hits themselves** (a point cloud or scans). A grid
  built from CAD geometry needs the polygons instead; that is what the
  exact-geometry planner shows.
- **Thin real obstacles** must return at least 2 hits to count (the outlier filter).
- **Removing a reflection costs more beams** with point-level evidence (a median of 47
  against 5): each beam only contradicts the hits it passes within 5 mm of, so beams
  are aimed at the hits one by one. That is the price of never eroding real corners.
- **Recall depends on coverage.** The unobserved-space rule makes the planner wait for
  observations rather than guess, so it is cautious where mapping was sparse.

## 17. Correlated range errors, sub-cell walls and grazing beams (`perception_experiment`)

**Why:** the EKF of section 13 treats the beams of a scan as independent. That is true
in this simulator's own world, where the map is the truth and the LiDAR noise is white.
It is false for two common errors that many beams share:

- **A range bias** common to a whole scan (calibration, temperature).
- **A map that is a rasterization of the real walls.** A wall that the map put 4 cm
  behind its true position shortens every beam that hits it, in every scan.

Fusing 60 beams that share one error as if they were independent shrinks the
covariance as if 60 errors had averaged out. The filter becomes confident, wrong, and
deaf to later corrections.

**Measuring it.** `perception_experiment ekf` runs random routes (differential drive,
72 beams, 2 cm white noise, 40 routes per row) and measures the NEES (3 = honest, as in
section 13) and the position RMS. It uses two worlds:

- **grid:** the map is the truth, plus an optional LiDAR bias (`--lidar-bias`, a
  Gauss–Markov process: constant, or drifting with `--lidar-bias-time`).
- **continuous:** the obstacles map's blocks as polygons. Each block is shifted by up
  to 0.45 cells and rotated by up to 6°; the outer walls are shifted and tilted by up
  to 1°. The LiDAR sees the polygons. The robot's map is their rasterization (majority
  rule), as good as a grid of that size can be.

**Fix 1: shared errors as states.** The exact treatment of coloured noise is to put
the shared error into the state (`AugmentedEkf`):

- **Bias state** `b` (`--ekf-bias-sigma`, `--ekf-bias-time`): every beam reads
  `h(pose) + b`.
- **Wall states** (`--ekf-map-sigma`, `--ekf-map-tilt`): each wall the beams hit gets
  an offset `a` and a tilt `t`. Its true surface lies `a + t (s − s₀)` behind the
  mapped one at position `s` along it, so a beam at incidence `α` reads
  `h + (a + t (s − s₀)) / cos α`. On the line surface (fix 2), a wall is one fitted
  segment. On the grid, hits within 10° and half a cell of each other belong to one
  wall.

The structure was measured first (`perception_experiment maperror`). Fitting one offset
and tilt per wall leaves 8 mm of the 35 mm RMS map error at 0.1 m cells, and 2.6 cm of
14 cm at 0.5 m; about 0.5 % of beams are gross (> 0.5 m). A first version used a
Gaussian process with distance-based correlation along surfaces. It correlated walls
that merely faced the same way and missed that one wall's error is shared along its
whole length, and its NEES stayed at about 180.

With these states the remaining noise is white, and the beams are fused one at a time,
all linearized at the prior. Without extra states this equals the information-form
batch update to 1e-9 (tested).

**Fix 2: sub-cell walls (`--ekf-surface lines`).** A rasterized slanted wall is a
staircase. On the grid, every step has its own axis-aligned normal, so the range
derivative jumps at every cell edge (`∂r/∂y` alternates between `−1/sin φ` and 0). A
small pose change moves the hit to another step, where the range jumps too. The line
surface recovers the walls from the grid:

1. **Trace:** trace every obstacle boundary as a closed chain of cell faces, and merge
   collinear faces into the exact axis-aligned outline.
2. **Detect staircases:** a straight wall rasterized off-axis turns alternately left
   and right, with one-face risers and treads whose lengths differ by at most one. A
   real corner repeats a turn; a longer run in the riser direction is a jog or a
   corner side and stays exact.
3. **Fit:** each staircase gets one line, a total-least-squares fit through its face
   midpoints. It is rejected if any midpoint lies more than 0.75 cells off.
   Neighbouring lines are intersected near their shared vertex; elsewhere a short
   connector joins them.

| 0.1 m cells, beams at 20° to the normal | Range RMS vs. truth | Largest jump of `∂r/∂y` along the wall |
|---|---|---|
| Grid, 3° / 6° / 17° / 33° walls | 31 / 30 / 29 / 29 mm | 1.06 |
| Lines, same walls | 0.08 / 3.7 / 0.5 / 1.1 mm | 0 |

Axis-aligned maps come out exactly as the grid (tested on the obstacles map: identical
ranges and normals). A C1 bicubic interpolation of the occupancy was tried first and
dropped. Interpolation keeps the staircase (normals spread by 14° on a 6° wall, some by
up to 68°), and straightening it would need a kernel as wide as a step, about 10 cells.

**Results** (mean NEES, its median over runs, the share of samples within the 95 %
bound, and position RMS):

| Model | Bias 3 cm (grid) | Rasterized 0.1 m, rotated | 0.1 m, axis-aligned | 0.05 m, rotated | 0.25 m, rotated |
|---|---|---|---|---|---|
| Independent beams (classic) | 741 / 237 / 18 % / 2.4 cm | 3295 / 2790 / 0 % / 3.3 cm | 4099 / 3780 / 0 % / 3.7 cm | 906 / 782 / 5 % / 1.8 cm | 21627 / 18099 / 0 % / 9.8 cm |
| Noise inflated to cover the error | 143 / 72 / 29 % / 2.1 cm | 927 / 840 / 3 % / 3.0 cm | 1225 / 873 / 0 % / 3.4 cm | 585 / 476 / 8 % / 1.8 cm | 1420 / 1375 / 0 % / 7.5 cm |
| … and every 6th beam only | 27 / 12 / 48 % / 2.2 cm | 125 / 98 / 9 % / 3.1 cm | 171 / 139 / 5 % / 3.4 cm | 82 / 74 / 23 % / 1.9 cm | 201 / 194 / 4 % / 7.4 cm |
| Bias state | **3.95 / 3.96 / 90 % / 0.4 cm** | – | – | – | – |
| Wall states, grid surface | – | 20 / 9.9 / 48 % / 3.5 cm | 5.1 / 4.0 / 82 % / 2.0 cm | 23 / 13 / 40 % / 1.9 cm | 38 / 11 / 46 % / 9.2 cm |
| **Wall states, line surface** | – | **6.9 / 4.4 / 76 % / 2.3 cm** | **4.3 / 3.1 / 89 % / 2.0 cm** | **7.0 / 5.7 / 70 % / 1.2 cm** | **19 / 8.6 / 56 % / 6.1 cm** |

- The usual workarounds (inflating `R`, subsampling beams) reduce the overconfidence
  10–100×, but it stays 10–400× too large. The error is shared, not just large, so no
  per-beam variance can describe it.
- The bias state is honest and six times more accurate: it estimates the bias to
  0.2 mm.
- Wall states on the line surface bring rasterized maps from NEES 900–22000 to 4–19,
  and halve the position error at 0.05–0.1 m cells.
- Residual states per cell-sized patch (`--ekf-map-residual`) help little and cost
  20× more.
- The wall-state prior was swept (`--sweep`): an offset std of `res/√12` and a tilt
  std of 0.02 rad are best. Larger priors are *worse* on both counts. When every wall
  is free to move, the pose is less constrained, and the EKF's linearization at
  changing estimates (the known EKF-SLAM inconsistency) grows.

**Fix 3: grazing beams (`--ekf-grazing model`, now the default).** The classic rule
dropped every beam beyond 78.46° incidence. `perception_experiment grazing` swept that
cutoff from 45° to 89° and compared it with a model that keeps every beam, weighted
by what it is worth:

- **Linearization variance** `½ tr(G P G P)`, where `G` is the range Hessian of a flat
  surface (in closed form, tested against finite differences). This is the exact
  variance of the second-order term for a Gaussian pose error, and it grows without
  bound towards 90°. Only `|cos α| < 0.02` is skipped, for numerical safety.
- **Map error along the normal**, magnified by `1/cos α`: the wall states, plus what
  they leave (0.3 of the offset std, measured).
- **Sliding off the end of a wall:** at grazing incidence a normal error moves the hit
  along the wall by `(normal error) × tan α`. When that slide (2σ) could carry the
  hit past the end of the wall, the real beam may hit something else, so it is
  skipped. This is a topological risk, not a variance.

| | Grid world (exact map) | Rasterized, 0.1 m | Rasterized, 0.05 m |
|---|---|---|---|
| Cutoff 45° | 3.21 / 0.46 cm / 0.19° | 4.47 / 2.25 cm | 4.57 / 1.34 cm |
| Cutoff 60° | 3.19 / 0.42 cm / 0.11° | 4.80 / 2.17 cm | 4.71 / 1.18 cm |
| Cutoff 78.46° (classic) | 4.14 / 0.39 cm / 0.06° | 6.91 / 2.29 cm | 6.99 / 1.21 cm |
| Cutoff 89° | 19.2 / 0.39 cm / 0.08° | 12.1 / 3.25 cm | 10.6 / 1.53 cm |
| **Model** | **3.76 / 0.38 cm / 0.05°** | **5.01 / 2.18 cm** | **4.53 / 1.09 cm** |

(Mean NEES / position RMS / heading RMS.)

- A fixed cutoff trades consistency against accuracy. Low cutoffs are honest but
  throw away the beams that fix heading (0.19° heading RMS at 45°). High cutoffs keep
  them but become overconfident.
- The model gives the best accuracy on every map and is consistent on the exact one.
  On rasterized maps it is about as consistent as the best cutoff, with better
  accuracy. On the CLI's demo route it reaches 3 mm and 0.05° (4 mm and 0.06° with the
  classic cutoff).

**Cost.** The augmented filter costs `O(n²)` per beam for `n` states: 0.04 ms per
72-beam scan with wall states (about 30 states), and about 1 ms with residual patches.

**Limitations.**

- The remaining overconfidence on rasterized maps (NEES 4–7 at 0.05–0.1 m cells, 19
  at 0.25 m) is the EKF's own linearization inconsistency once map states are
  estimated. Coarse maps also change shape, not just position: blocks one or two cells
  wide gain or lose a cell, which no Gaussian error model covers. At 0.5 m cells (the
  CLI's default maps) every model stays far from honest, so localize on a finer map.
- Wall states are dropped least-recently-used beyond `max_map_states` (200). A wall
  seen again after that starts from its prior, so its correlation with earlier scans
  is forgotten.
- The line surface fits straight walls only. Curved walls stay staircases, or become
  short segments when their steps are regular.

## 18. Moving obstacles from the LiDAR alone: tracking, reversals, occlusion (`--obstacle-perception`)

**Why:** section 14's robot did not perceive moving obstacles. It was handed their true
positions whenever they were within 3 m and the line to their centres crossed no wall
(an "oracle"). It took their velocity from two consecutive sightings and forgot any
obstacle the moment it went out of sight. This section replaces that shortcut with the
robot's own LiDAR. It also fixes two weaknesses the shortcut hid:

- **Reversals:** the robot predicted every obstacle straight on, so it could not
  anticipate a reversal. Each reversal flipped the predicted sweep, and with it the
  best detour, which caused back-and-forth livelocks.
- **Forgetting:** an obstacle stopped existing the moment a wall hid it.

**Detection by map subtraction** (`detectMovingObstacles`). Each step the robot scans
(360 beams over a full circle by default, range = sensing range, 2 cm noise in the
experiments). Since the map is known, so is every beam's expected range.

- **Foreign returns:** a return more than 10 cm shorter than expected hit something
  the map does not contain.
- **Clusters:** consecutive foreign returns closer than 0.3 m form a cluster.
- **Merge:** clusters whose fitted discs overlap are merged (a wall in front of part of
  an obstacle splits it).
- **Minimum size:** clusters need at least 2 points, which drops single stray returns.
- **Disc fit:** each cluster is fitted by Gauss–Newton to `|p − c| = R`. The robot sees
  only the near side, so the centre lies behind the hits. The radius is fitted when 3+
  points span 60°+ of the disc; otherwise it is the 0.3 m prior.
- **Tested:** on 100 scans of the static map, map subtraction finds nothing. Discs are
  located to 3 cm, with their radius to 3 cm.

**Tracking** (`ObstacleTracker`). Each track is a constant-velocity Kalman filter
(0.5 m/s² white acceleration). Each scan:

| Step | Rule | Why (the failure that led to it) |
|---|---|---|
| Association | Greedy by Mahalanobis distance within a χ² gate (99.9 %), and 1 m. Confirmed tracks pick before tentative ones. | A stray one-point fragment at a disc's edge started a tentative track that stole the next detection, so the real track starved and never saw the reversal. |
| Birth | Only from detections that do not overlap a confirmed track's disc. | Same: fragments of a tracked obstacle spawned duplicates. |
| Two-point start | On a track's second detection, velocity = difference of the two positions, with its covariance. | Starting from velocity 0 ± 1 m/s, a 2 m/s obstacle's estimate pointed the wrong way for the first 0.3 s. |
| Maneuvers | A normalized innovation above the 99 % gate (one scan), or the last three together above the 6-dof 99 % gate, resets the velocity uncertainty. | A slow obstacle turning round shifts each innovation only a little, but always the same way. |
| Negative evidence | An undetected track is checked against the scan: if a beam passed through where its disc should be (and the map does not hide it there), that is a miss. 3 misses in a row delete it; a tentative track goes after 1. | – |
| Coasting | If the disc is hidden (by a wall or another return) or out of range, the track keeps being predicted, with growing uncertainty, for up to 10 s. | The oracle forgot an obstacle the moment it went behind a wall. |

**Predictions that survive reversals** (`predictPath`, item 7). Once a track has moved
along a straight line (positions within 12 cm RMS of their principal axis), every
reversal of its velocity along the axis records a turning point. The turning point is
the farthest position reached before the reversal. The predictor then moves the
obstacle along the axis and reflects it at the known ends; without a known end in the
direction of travel it extrapolates. A turning point is learned only from a reversal
the LiDAR actually watched, with detections less than 0.3 s apart through the turn. An
obstacle that turns while hidden and reappears says nothing about where it turned;
learning from it gave turning points far too short. Every predicted position carries
its standard deviation. The look-ahead gate inflates the obstacle by 2σ (at most
0.1 m); the planning map is not inflated.

Prediction accuracy (`perception_experiment prediction`; a robot watching one obstacle
patrol a random lane for 60 s, 300 lanes, after a 10 s warm-up):

| | RMS error 1 s / 2 s ahead | 2 s ahead > 0.5 m off | When a reversal falls within the 2 s: RMS, > 0.5 m off |
|---|---|---|---|
| Constant velocity, 0.2–0.5 m/s | 0.48 / 0.87 m | 47 % | 0.98 m, 73 % |
| **Learned reversals**, 0.2–0.5 m/s | **0.37 / 0.51 m** | **9 %** | **0.38 m, 7 %** |
| Constant velocity, 0.6–1.0 m/s | 1.27 / 2.23 m | 86 % | 2.26 m, 96 % |
| **Learned reversals**, 0.6–1.0 m/s | **0.93 / 1.17 m** | **17 %** | **0.97 m, 15 %** |

**Decision rules changed along the way** (they apply to both perceptions):

- **Choosing the retreat.** A robot inside a fast obstacle's predicted sweep used to
  back up to its previous waypoint by rule, even when that waypoint lay in the middle
  of the obstacle's lane. When backing up is itself predicted to end in contact, it now
  heads for whichever end of its segment keeps the larger predicted gap. Over 1000
  scenarios of the test generator this cut collisions from 26 to 23 (oracle) and from
  28 to 7 (LiDAR). On blind junctions with 2 m/s cross traffic it took LiDAR tracking
  from 64 % to 92 %.
- **Soft margin.** An obstacle patrolling the lane next to the goal (0.23 m away) never
  leaves; with the full 0.25 m margin the robot waited for it until the timeout. After
  waiting out the patience (6 s), the gate accepts any contact-free motion.

**Results** (`perception_experiment dynamic`, the obstacles map, start (1, 1), goal
(18, 18), 3 obstacles on random lanes of 2–8 cells, 600 s limit; reversals = times the
robot's direction of motion flips by more than 150°, i.e. shuttling):

| 500 scenarios, 0.2–0.5 m/s | Reached | Collisions | Timeouts | Travel vs. oracle | Reversals per run |
|---|---|---|---|---|---|
| Point, blind | 64.0 % | 180 | 0 | – | – |
| Point, oracle (original) | 97.2 % | 3 | 11 | – | 4.1 |
| Point, LiDAR, constant velocity | 97.8 % | 5 | 6 | +5.7 s | 4.7 |
| **Point, LiDAR, learned reversals** | **98.8 %** | **3** | **3** | +4.2 s | **2.0** |
| Differential drive, oracle | 94.8 % | 6 | 20 | – | 5.4 |
| Differential drive, LiDAR, constant velocity | 97.6 % | 8 | 4 | +4.5 s | 3.6 |
| **Differential drive, LiDAR, learned reversals** | **98.6 %** | **3** | **4** | +2.3 s | **2.0** |

| 300 scenarios, 0.6–1.0 m/s | Reached | Collisions | Timeouts | Reversals per run |
|---|---|---|---|---|
| Point, oracle | 85.7 % | 8 | 35 | 16.5 |
| Point, LiDAR, constant velocity | 89.7 % | 7 | 24 | 12.6 |
| **Point, LiDAR, learned reversals** | **94.7 %** | 8 | **8** | **5.1** |
| Differential drive, oracle | 82.7 % | 9 | 43 | 14.3 |
| **Differential drive, LiDAR, learned reversals** | **94.7 %** | **4** | **12** | **4.7** |

- The robot that perceives obstacles through its own LiDAR does better than the one
  handed their true positions. That is because tracks remember (coasting) and predict
  reversals, not because they are more precise. Learned reversals cut the shuttling by
  a factor of 2–3 and, with fast obstacles, the timeouts by a factor of 4.
- **Ablations:** without uncertainty inflation, collisions rise from 3 to 5–6 at
  normal speeds. Without coasting, collisions rise from 3 to 5–6 at normal speeds and
  from 8 to 10 with fast obstacles.
- **Cost:** about 0.13 ms per step with 360 beams (two ray casts per beam, detection,
  tracking). Travel is 2–4 s longer than with the oracle, because tracks keep blocking
  where an obstacle may still be.

**Occluded space (item 8).** Two kinds of hidden obstacle:

1. **Seen, then hidden:** coasting keeps the obstacle, as above. This is the measured
   gain.
2. **Never seen** (`--occlusion-aware`): free space hidden behind walls may hold an
   obstacle moving at up to 1 m/s. Before each step the robot looks from where the
   step would take it, and holds if something emerging from the space still hidden
   from there could reach it within 0.5 s. It creeps round blind corners, seeing more
   with every step. Space beyond the sensing range does not count, since obstacles
   coming from there are seen in time. After the patience it commits for 1.5 m.

   It was measured on blind junctions (`perception_experiment junction`): a corridor
   crossed by three side corridors that walls hide until the robot is at the crossing,
   with cross traffic on long lanes:

   | 200 runs | Cross traffic 0.6–1.2 m/s: reached / collisions | 1.5–2.5 m/s |
   |---|---|---|
   | Oracle | 100 % / 0 | 100 % / 0 |
   | LiDAR tracking | 100 % / 0 (diff-drive 99 % / 2) | 92 % / 16 |
   | + occlusion-aware | 70 % / 24, 37 timeouts | 0.5 % / 3, 196 timeouts |

   It is off by default because it makes things worse. No collision in any benchmark
   involved an obstacle in view for less than a second ("seen < 1 s" stays 0): the
   robot already sees down a side corridor before it enters it. Holding at a blind
   corner often means holding where cross traffic passes, and waiting gains no
   information, because what is hidden stays hidden. The remaining junction collisions
   at 2 m/s (8 %) come from obstacles faster than 3 m of sensing range allows to
   react to (the oracle avoids them only through its zero latency). They need longer
   range or speed limits, not occlusion reasoning.

**Limitations.**

- Learned turning points assume straight patrol lanes. Routes with corners fall back
  to constant velocity.
- Obstacles are discs. Two obstacles closer than the cluster distance are one cluster.
- The LiDAR used for obstacles is separate from the EKF's (section 13), which still
  sees only the static map, and the robot is still steered by its true pose.

## Limitations

- The footprint is a disc (no rectangular or oriented robots). Inflation is evaluated at
  cell centers, so on coarse grids it may reject passages the robot could physically
  use (section 8).
- The point robot has no orientation and changes direction instantly. The
  differential-drive model (section 10) adds heading and turn-rate limits, but no
  acceleration limits or dynamics: velocities change instantly. It stops to turn at every
  corner, and the planner minimises path length, not travel time including turns.
- Paths are optimal on the grid graph (4- or 8-connected), not in continuous space: they
  turn only at cell centers and in 45° steps. Any-angle planning (e.g. Theta*) or path
  smoothing could shorten them further.
