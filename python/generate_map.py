#!/usr/bin/env python3
"""Generate occupancy-grid map files for the robot navigation simulator.

Map file format (shared with the C++ loader):
    * one grid row per line, cells separated by commas;
    * 0 = free, 1 = occupied;
    * the first line is row y = 0 (top of the map), x = column index.

Map types:
    empty      no obstacles
    obstacles  fixed rectangular obstacles (layout scaled from a 20 x 20 design)
    corridor   serpentine corridor: wall rows with single-cell gaps
    maze       perfect maze (recursive backtracker), reproducible with --seed
    random     each cell occupied with probability --density, reproducible with --seed

Examples:
    python3 python/generate_map.py --type obstacles --output maps/obstacles.csv
    python3 python/generate_map.py --type random --width 40 --height 40 \\
        --density 0.3 --seed 7 --ensure-reachable --output maps/random_40.csv
"""

from __future__ import annotations

import argparse
import sys
from collections import deque
from pathlib import Path

import numpy as np

FREE = 0
OCCUPIED = 1
MAP_TYPES = ("empty", "obstacles", "corridor", "maze", "random")

# Obstacle rectangles for the "obstacles" map, designed on a 20 x 20 grid and
# scaled proportionally to other sizes. Each entry is (x0, y0, x1, y1) with
# x1/y1 exclusive.
REFERENCE_SIZE = 20
FIXED_OBSTACLES = (
    (4, 3, 6, 15),
    (9, 6, 16, 8),
    (12, 10, 14, 18),
    (6, 15, 10, 17),
    (16, 2, 19, 4),
    (15, 12, 18, 14),
)

# Wall rows of the corridor map as fractions of the height.
CORRIDOR_WALL_FRACTIONS = (0.25, 0.5, 0.75)
MIN_CORRIDOR_WIDTH = 3
MIN_CORRIDOR_HEIGHT = 7


class MapGenerationError(Exception):
    """Raised when a map cannot be generated with the requested parameters."""


def generate_empty(width: int, height: int) -> np.ndarray:
    return np.zeros((height, width), dtype=np.uint8)


def _scale(value: int, size: int) -> int:
    return int(round(value * size / REFERENCE_SIZE))


def generate_obstacles(width: int, height: int) -> np.ndarray:
    grid = generate_empty(width, height)
    for x0, y0, x1, y1 in FIXED_OBSTACLES:
        grid[_scale(y0, height):_scale(y1, height),
             _scale(x0, width):_scale(x1, width)] = OCCUPIED
    return grid


def generate_corridor(width: int, height: int) -> np.ndarray:
    if width < MIN_CORRIDOR_WIDTH or height < MIN_CORRIDOR_HEIGHT:
        raise MapGenerationError(
            f"corridor maps need at least {MIN_CORRIDOR_WIDTH} x "
            f"{MIN_CORRIDOR_HEIGHT} cells")
    grid = generate_empty(width, height)
    for i, fraction in enumerate(CORRIDOR_WALL_FRACTIONS):
        row = int(round(height * fraction))
        grid[row, :] = OCCUPIED
        gap_x = width - 2 if i % 2 == 0 else 1  # alternate the gap side
        grid[row, gap_x] = FREE
    return grid


def generate_maze(width: int, height: int, rng: np.random.Generator) -> np.ndarray:
    """Perfect maze: maze cells sit at odd (x, y); walls fill the rest."""
    cells_x = width // 2
    cells_y = height // 2
    if cells_x < 1 or cells_y < 1:
        raise MapGenerationError("maze maps need at least 2 x 2 cells")

    grid = np.ones((height, width), dtype=np.uint8)
    visited = np.zeros((cells_y, cells_x), dtype=bool)
    stack = [(0, 0)]
    visited[0, 0] = True
    grid[1, 1] = FREE

    while stack:
        cx, cy = stack[-1]
        candidates = [
            (cx + dx, cy + dy)
            for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0))
            if 0 <= cx + dx < cells_x and 0 <= cy + dy < cells_y
            and not visited[cy + dy, cx + dx]
        ]
        if not candidates:
            stack.pop()
            continue
        nx, ny = candidates[rng.integers(len(candidates))]
        grid[cy + ny + 1, cx + nx + 1] = FREE  # wall between the two maze cells
        grid[2 * ny + 1, 2 * nx + 1] = FREE
        visited[ny, nx] = True
        stack.append((nx, ny))
    return grid


def generate_random(width: int, height: int, density: float,
                    rng: np.random.Generator) -> np.ndarray:
    return (rng.random((height, width)) < density).astype(np.uint8)


def is_reachable(grid: np.ndarray, start: tuple[int, int],
                 goal: tuple[int, int]) -> bool:
    """Four-connected flood fill. Only answers 'is there a path?'.

    This is a connectivity check for map generation; the authoritative path
    planner is the C++ A* implementation.
    """
    height, width = grid.shape
    if grid[start[1], start[0]] != FREE or grid[goal[1], goal[0]] != FREE:
        return False
    seen = np.zeros_like(grid, dtype=bool)
    seen[start[1], start[0]] = True
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            nx, ny = x + dx, y + dy
            if (0 <= nx < width and 0 <= ny < height and not seen[ny, nx]
                    and grid[ny, nx] == FREE):
                seen[ny, nx] = True
                queue.append((nx, ny))
    return False


def build_map(args: argparse.Namespace, start: tuple[int, int],
              goal: tuple[int, int]) -> np.ndarray:
    rng = np.random.default_rng(args.seed)
    if args.type == "empty":
        return generate_empty(args.width, args.height)
    if args.type == "obstacles":
        return generate_obstacles(args.width, args.height)
    if args.type == "corridor":
        return generate_corridor(args.width, args.height)
    if args.type == "maze":
        return generate_maze(args.width, args.height, rng)

    # Random maps: a path is not guaranteed. With --ensure-reachable, retry
    # (continuing the same seeded random stream) until start and goal connect.
    attempts = args.max_attempts if args.ensure_reachable else 1
    for _ in range(attempts):
        grid = generate_random(args.width, args.height, args.density, rng)
        if not args.ensure_reachable:
            return grid
        grid[start[1], start[0]] = FREE
        grid[goal[1], goal[0]] = FREE
        if is_reachable(grid, start, goal):
            return grid
    raise MapGenerationError(
        f"no reachable random map found in {args.max_attempts} attempts "
        f"(density {args.density}); lower --density or raise --max-attempts")


def write_map(grid: np.ndarray, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(output, grid, fmt="%d", delimiter=",")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an occupancy-grid map CSV (0 = free, 1 = occupied).")
    parser.add_argument("--type", required=True, choices=MAP_TYPES,
                        help="map layout to generate")
    parser.add_argument("--width", type=int, default=20, help="columns (default 20)")
    parser.add_argument("--height", type=int, default=20, help="rows (default 20)")
    parser.add_argument("--density", type=float, default=0.2,
                        help="obstacle probability for random maps, 0..1 (default 0.2)")
    parser.add_argument("--seed", type=int, default=0,
                        help="random seed for maze/random maps (default 0)")
    parser.add_argument("--output", type=Path, required=True, help="output CSV path")
    parser.add_argument("--start", type=int, nargs=2, metavar=("X", "Y"),
                        help="start cell used for the reachability report "
                             "(default 1 1)")
    parser.add_argument("--goal", type=int, nargs=2, metavar=("X", "Y"),
                        help="goal cell used for the reachability report "
                             "(default WIDTH-2 HEIGHT-2)")
    parser.add_argument("--ensure-reachable", action="store_true",
                        help="fail unless the goal is reachable from the start; "
                             "random maps are regenerated until it is")
    parser.add_argument("--max-attempts", type=int, default=100,
                        help="random-map attempts with --ensure-reachable (default 100)")
    args = parser.parse_args(argv)

    if args.width < 1 or args.height < 1:
        parser.error("--width and --height must be positive")
    if not 0.0 <= args.density <= 1.0:
        parser.error("--density must be between 0 and 1")
    if args.max_attempts < 1:
        parser.error("--max-attempts must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    start = tuple(args.start) if args.start else (min(1, args.width - 1),
                                                  min(1, args.height - 1))
    goal = tuple(args.goal) if args.goal else (max(args.width - 2, 0),
                                               max(args.height - 2, 0))
    for name, (x, y) in (("start", start), ("goal", goal)):
        if not (0 <= x < args.width and 0 <= y < args.height):
            print(f"Error: {name} {(x, y)} is outside the "
                  f"{args.width} x {args.height} map", file=sys.stderr)
            return 1

    try:
        grid = build_map(args, start, goal)
    except MapGenerationError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    reachable = is_reachable(grid, start, goal)
    if args.ensure_reachable and not reachable:
        print(f"Error: goal {goal} is not reachable from start {start} on the "
              f"'{args.type}' map", file=sys.stderr)
        return 1

    write_map(grid, args.output)
    occupied = int(grid.sum())
    print(f"Wrote {args.output} ({args.width} x {args.height}, {occupied} occupied "
          f"cells, {100.0 * occupied / grid.size:.1f}% density)")
    print(f"Start {start} -> goal {goal}: "
          f"{'reachable' if reachable else 'NOT reachable'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
