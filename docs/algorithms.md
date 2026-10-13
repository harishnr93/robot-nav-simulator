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

## Limitations

- The footprint is a disc (no rectangular or oriented robots). Inflation is evaluated at
  cell centers, so on coarse grids it may reject passages the robot could physically
  use (section 8).
- The robot has no orientation, acceleration limits or dynamics: it changes direction
  instantly at waypoints (a differential-drive model is a planned extension).
- Paths are optimal on the grid graph (4- or 8-connected), not in continuous space: they
  turn only at cell centers and in 45° steps. Any-angle planning (e.g. Theta*) or path
  smoothing could shorten them further.
