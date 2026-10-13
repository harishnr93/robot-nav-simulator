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
Beams are skipped where this linearization is poor: a predicted corner hit, a grazing
beam (incidence above about 78°, where `1/cos φ` explodes) and beams without a measured
or predicted return. A beam whose innovation `ν = z − r̂` is larger than `--ekf-gate`
(default 3) standard deviations `√(H P Hᵀ + R)` is rejected as an outlier. Typically the
real beam passed an obstacle edge that the predicted one hit, or the other way round.
About 0.25 % of the beams are rejected on the demo route.

The accepted beams are fused in one batch update in **information form**:

```text
P⁺ = (P⁻¹ + Σ Hᵢᵀ Hᵢ / R)⁻¹,     x⁺ = x + P⁺ Σ Hᵢᵀ νᵢ / R,     R = --ekf-range-sigma² (0.05 m)
```

This is the standard Kalman update of all the beams stacked (tested against
`K = P Hᵀ / (H P Hᵀ + R)` for one beam), but it only inverts 3 × 3 matrices, however
many beams there are.

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
| 72 beams, exact ranges, default sigma 0.05 m (one run) | 0.49 |

The last line is cautious on purpose: the default assumes 5 cm of range noise, while
the default LiDAR is exact. Set `--ekf-range-sigma` to the actual noise for a consistent
filter.

**Results** (obstacles map, 72 beams, defaults, one run): position error RMS 4 mm and
at most 1.5 cm, against 0.53 m RMS and 1.14 m at the end for dead reckoning; heading
RMS 0.06° against 4.8°. Over odometry seeds 0–5, the EKF stays at 3–4 mm RMS (at most
1.7 cm) while dead reckoning ranges from 0.16 to 0.94 m RMS.

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
- Range measurements of one scan are treated as independent. Map discretization makes
  neighbouring beams' errors correlated, which a real implementation would model or
  avoid by subsampling beams.
- Requires the differential drive (`--robot-model diff-drive`): the odometry model
  measures turns and distances driven.
- The estimate does not feed back into the control. Closing that loop is part of the
  "realistic actuator" future improvement.

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
