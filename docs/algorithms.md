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

**Implementation details (`src/astar.cpp`):**

| Concern | Choice |
|---|---|
| Open set | `std::priority_queue` (min-heap via a custom comparator) |
| Decrease-key | not supported by `std::priority_queue`; cells are pushed again when a cheaper `g` is found, and stale entries are skipped when popped (`closed` check) |
| Costs / parents | `std::vector<int>` and `std::vector<size_t>` indexed by cell |
| Expanded set | `std::vector<bool> closed` |
| Expanded-node count | number of cells popped *and* expanded, including the goal |
| Timing | `std::chrono::steady_clock` around validation + search only (no file I/O) |

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
most 4 times, giving `O(V log V)` time and `O(V)` memory.

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
bad input paths now, and it is the hook where a finite robot footprint will be checked
in later extensions.

**How it works:** before executing a step from `a` to `b`, `isSegmentCollisionFree()`
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

**Limitation:** the robot is a point. A real robot with radius `r` would need every
point within `r` of its center to be free (obstacle inflation), which this check does
not do.

## Limitations

- The robot is treated as a point. Paths may pass right next to obstacles; a real robot
  would need obstacle inflation by its radius (planned extension).
- The robot has no orientation, acceleration limits or dynamics: it changes direction
  instantly at waypoints (a differential-drive model is a planned extension).
- Only 4-connected moves. Paths are optimal in grid steps but are not the shortest
  Euclidean route (8-connected A* with an octile heuristic is a planned extension).
