#!/usr/bin/env python3
"""Plot the grid-alignment experiments written by build/alignment_experiment.

Reads (from --input, default docs/alignment/):
    example_meta.csv, example_cells.csv, example_polygons.csv, example_path.csv
    alignment_doorway.csv, alignment_classify.csv

Writes (to --output-dir, default docs/images/):
    alignment_example.png    the same door, two sub-cell offsets: passable / NO_PATH
    alignment_doorway.png    share of alignments that find the door passable, by width
    alignment_classify.png   grid NO_PATH: grid-only recovery vs. point-aware recovery

Example:
    ./build/alignment_experiment example --output docs/alignment
    ./build/alignment_experiment doorway --trials 60 --output docs/alignment
    ./build/alignment_experiment classify --trials 60 --output docs/alignment
    python3 python/plot_alignment_experiment.py
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from matplotlib.patches import Polygon as PolygonPatch

REPO_ROOT = Path(__file__).resolve().parent.parent
FIGURE_DPI = 120
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#dddddd"
FREE_COLOR = "#f2f2f2"
OCCUPIED_COLOR = "#3c3c3c"
INFLATED_COLOR = "#e7c9a9"
PATH_COLOR = "#1f77b4"
START_COLOR = "#2ca02c"
GOAL_COLOR = "#d62728"
# Methods in a fixed categorical order (grid rules first, then the robust planners).
METHODS = ("any_overlap", "center_inside", "majority", "lidar_grid", "lidar_points",
           "exact_lattice")
METHOD_LABELS = {
    "any_overlap": "Grid: any overlap",
    "center_inside": "Grid: cell center inside",
    "majority": "Grid: majority coverage",
    "lidar_grid": "Grid: LiDAR log-odds map",
    "lidar_points": "LiDAR points + fine lattice (fix)",
    "exact_lattice": "Exact geometry + fine lattice (reference)",
}
METHOD_COLORS = dict(zip(METHODS, ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4",
                                   "#008300"), strict=True))
OUTCOME_COLORS = {"RECOVERED": "#0ca30c", "UNSAFE": "#d03b3b", "GAVE_UP": "#b4b2a9"}
OUTCOME_LABELS = {"RECOVERED": "recovered (path free in reality)",
                  "UNSAFE": "unsafe (path through a real obstacle)",
                  "GAVE_UP": "gave up"}


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(f"'{path}' not found; run build/alignment_experiment first")
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def style(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(colors=TEXT_SECONDARY)


def plot_example(input_dir: Path, output: Path) -> None:
    """Two panels: the same doorway rasterized with two sub-cell offsets."""
    meta = read_rows(input_dir / "example_meta.csv")
    cells = read_rows(input_dir / "example_cells.csv")
    polygons = read_rows(input_dir / "example_polygons.csv")
    paths = read_rows(input_dir / "example_path.csv")
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 6.6), layout="constrained")
    for ax, m in zip(axes, meta, strict=True):
        k = m["example"]
        res = float(m["resolution"])
        mine = [r for r in cells if r["example"] == k]
        n = max(int(r["x"]) for r in mine) + 1
        grid = np.zeros((n, n), dtype=int)
        for r in mine:
            value = 2 if r["occupied"] == "1" else 1 if r["inflated"] == "1" else 0
            grid[int(r["y"]), int(r["x"])] = value
        ax.imshow(grid, cmap=ListedColormap([FREE_COLOR, INFLATED_COLOR, OCCUPIED_COLOR]),
                  vmin=0, vmax=2, interpolation="nearest", origin="upper",
                  extent=(0, n * res, n * res, 0))
        # Cell lines and the true geometry on top.
        ax.set_xticks(np.arange(0, n * res + 1e-9, res), minor=True)
        ax.set_yticks(np.arange(0, n * res + 1e-9, res), minor=True)
        ax.grid(which="minor", color="#c8c8c8", linewidth=0.4)
        groups = defaultdict(list)
        for r in polygons:
            if r["example"] == k:
                groups[r["polygon"]].append((float(r["x"]), float(r["y"])))
        for i, vertices in enumerate(groups.values()):
            ax.add_patch(PolygonPatch(vertices, closed=True, fill=False, edgecolor=TEXT_PRIMARY,
                                      linewidth=1.6, zorder=3,
                                      label="True walls" if i == 0 else None))
        path = np.array([[(int(r["x"]) + 0.5) * res, (int(r["y"]) + 0.5) * res]
                         for r in paths if r["example"] == k])
        if len(path):
            ax.plot(path[:, 0], path[:, 1], color=PATH_COLOR, linewidth=2.2, zorder=4,
                    label="Planned path")
        start = (float(m["start_x"]), float(m["start_y"]))
        goal = (float(m["goal_x"]), float(m["goal_y"]))
        ax.plot(*start, marker="o", markersize=10, color=START_COLOR, markeredgecolor="white",
                linestyle="none", zorder=5, label="Start")
        ax.plot(*goal, marker="*", markersize=15, color=GOAL_COLOR, markeredgecolor="white",
                linestyle="none", zorder=5, label="Goal")
        # Zoom on the doorway.
        cx = (start[0] + goal[0]) / 2
        cy = (start[1] + goal[1]) / 2
        ax.set_xlim(cx - 1.1, cx + 1.1)
        ax.set_ylim(cy + 1.1, cy - 1.1)
        ax.set_aspect("equal")
        ax.tick_params(which="minor", length=0)
        ax.set_xlabel("x (m, grid frame)")
        ax.set_ylabel("y (m, grid frame)")
        passable = m["passable"] == "1"
        ax.set_title(f"grid shifted by ({100 * float(m['subcell_x']):.1f}, "
                     f"{100 * float(m['subcell_y']):.1f}) cm: "
                     + ("passable" if passable else "NO_PATH"),
                     loc="left", fontsize=11, color=TEXT_PRIMARY)
    handles, _ = axes[0].get_legend_handles_labels()
    handles += [Patch(facecolor=OCCUPIED_COLOR, label="Occupied cell (any overlap)"),
                Patch(facecolor=INFLATED_COLOR, label="Too close for the robot (inflated)")]
    fig.legend(handles=handles, loc="outside lower center", ncol=6, frameon=False, fontsize=9)
    m = meta[0]
    fig.suptitle(f"The same {100 * float(m['door_width']):.0f} cm door for a "
                 f"{100 * 2 * float(m['radius']):.0f} cm robot on a {100 * float(m['resolution']):.0f} cm "
                 f"grid: a few centimetres of grid offset decide whether it is passable", fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def plot_doorway(input_dir: Path, output: Path) -> None:
    """Share of random alignments that find the door passable, per method and width."""
    rows = read_rows(input_dir / "alignment_doorway.csv")
    counts: dict[str, dict[float, list[int]]] = defaultdict(lambda: defaultdict(list))
    for r in rows:
        counts[r["method"]][float(r["door_width"])].append(int(r["passable"]))
    fig, ax = plt.subplots(figsize=(11, 6.6), layout="constrained")
    two_r = None
    for r in rows:
        if r["truth"] == "1":
            w = float(r["door_width"])
            two_r = w if two_r is None else min(two_r, w)
    ax.axvspan(0.45, two_r, color="#f0efec", zorder=0)
    top = ax.get_xaxis_transform()  # x in data, y in axes fraction
    ax.text(two_r - 0.005, 0.97, "truly blocked (door < 2 r)", ha="right", va="top", fontsize=9,
            color=TEXT_SECONDARY, transform=top)
    ax.text(two_r + 0.005, 0.97, "truly passable", ha="left", va="top", fontsize=9,
            color=TEXT_SECONDARY, transform=top)
    for method in METHODS:
        widths = sorted(counts[method])
        share = [100 * np.mean(counts[method][w]) for w in widths]
        if method == "exact_lattice":
            # Dashed on top: it nearly coincides with the LiDAR-point planner.
            ax.plot(widths, share, color=METHOD_COLORS[method], linewidth=1.6, linestyle="--",
                    label=METHOD_LABELS[method], zorder=4)
        else:
            robust = method == "lidar_points"
            ax.plot(widths, share, color=METHOD_COLORS[method], linewidth=4.0 if robust else 2.0,
                    label=METHOD_LABELS[method], zorder=3 if robust else 2)
    ax.set_xlim(0.48, max(max(v) for v in counts.values()) + 0.02)
    ax.set_ylim(-3, 108)
    ax.set_xlabel("door width (m), robot diameter 0.6 m, cells 0.1 m")
    ax.set_ylabel("random grid alignments that find a path (%)")
    ax.grid(color=GRID_COLOR, linewidth=0.6)
    style(ax)
    fig.legend(*ax.get_legend_handles_labels(), loc="outside lower center", ncol=3,
               frameon=False, fontsize=9)
    trials = len(counts["any_overlap"][min(counts["any_overlap"])])
    fig.suptitle(f"Where the grid's rotation and offset decide: between 0 % and 100 % "
                 f"({trials} random alignments per width)", fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def plot_classify(input_dir: Path, output: Path) -> None:
    rows = read_rows(input_dir / "alignment_classify.csv")
    kinds = (("artifact", "Discretization artifact\n(door 1-15 cm wider than 2 r)"),
             ("ghost", "Sensor ghost\n(reflection in a wide door)"),
             ("real", "Real block\n(door 1-15 cm narrower than 2 r)"))
    pipelines = (("grid_recovery", "Grid-only recovery (problem 1)"),
                 ("point_recovery", "Point-aware recovery (this work)"))
    fig, ax = plt.subplots(figsize=(12.5, 6.0), layout="constrained")
    labels, y = [], []
    position = 0.0
    for kind, kind_label in kinds:
        for pipeline, pipeline_label in pipelines:
            mine = [r for r in rows if r["kind"] == kind and r["pipeline"] == pipeline]
            total = len(mine)
            left = 0.0
            for outcome, color in OUTCOME_COLORS.items():
                share = 100.0 * sum(r["outcome"] == outcome for r in mine) / max(total, 1)
                if share > 0:
                    ax.barh(position, share, left=left, color=color, height=0.7,
                            edgecolor="white", linewidth=2)
                    if share >= 7:
                        ax.text(left + share / 2, position, f"{share:.0f} %", ha="center",
                                va="center", fontsize=8.5,
                                color="white" if outcome != "GAVE_UP" else TEXT_PRIMARY)
                left += share
            recovered = [float(r["beams"]) for r in mine if r["outcome"] == "RECOVERED"]
            note = f"median {np.median(recovered):g} beams" if recovered else ""
            if pipeline == "point_recovery":
                artifacts = sum(r["artifact"] == "1" for r in mine)
                note += f"{'; ' if note else ''}{artifacts}/{total} called an artifact"
            ax.text(101.5, position, note, va="center", fontsize=8.5, color=TEXT_SECONDARY)
            labels.append(f"{kind_label.splitlines()[0]}: {pipeline_label}")
            y.append(position)
            position -= 1.0
        position -= 0.6
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=9)
    ax.set_xlim(0, 140)
    ax.set_xticks([0, 25, 50, 75, 100])
    total = len([r for r in rows if r["pipeline"] == "grid_recovery"]) // 3
    ax.set_xlabel(f"share of {total} grid NO_PATH cases per kind (%)")
    style(ax)
    handles = [Patch(facecolor=c, label=OUTCOME_LABELS[o]) for o, c in OUTCOME_COLORS.items()]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False, fontsize=9.5)
    fig.suptitle("Grid NO_PATH from LiDAR maps: telling discretization artifacts from sensor "
                 "ghosts and real blocks", fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot the grid-alignment experiments.")
    parser.add_argument("--input", type=Path, default=REPO_ROOT / "docs" / "alignment",
                        help="directory written by alignment_experiment (default docs/alignment)")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "docs" / "images",
                        help="where to save the figures (default docs/images)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plt.switch_backend("Agg")
    try:
        plot_example(args.input, args.output_dir / "alignment_example.png")
        plot_doorway(args.input, args.output_dir / "alignment_doorway.png")
        plot_classify(args.input, args.output_dir / "alignment_classify.png")
    except FileNotFoundError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
