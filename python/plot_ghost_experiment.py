#!/usr/bin/env python3
"""Plot the "why NO_PATH?" experiments written by build/ghost_experiment.

Reads (from --input, default docs/ghost/):
    example_map.csv, example_solutions.csv, example_path.csv, example_meta.csv
    ghost_recovery_thorough.csv, ghost_recovery_sparse.csv
    ghost_scaling.csv

Writes (to --output-dir, default docs/images/):
    ghost_example.png    a NO_PATH map: the cells of every optimal explanation and
                         the path the first one opens
    ghost_recovery.png   recovery strategies: outcomes and beams spent, for thorough
                         and sparse mapping
    ghost_scaling.png    exact solver time vs. explanation size and map size

Example:
    ./build/ghost_experiment example --seed 23 --density 0.08 --output docs/ghost
    ./build/ghost_experiment recovery --trials 300 --output docs/ghost
    ./build/ghost_experiment scaling --trials 100 --output docs/ghost
    python3 python/plot_ghost_experiment.py
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch, Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
import visualize_map as vm  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
FIGURE_DPI = 120
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#dddddd"
# Outcomes are states: the fixed status colors, always with a text label.
OUTCOME_COLORS = {"RECOVERED": "#0ca30c", "UNSAFE": "#d03b3b", "GAVE_UP": "#b4b2a9"}
OUTCOME_LABELS = {"RECOVERED": "recovered (path free in reality)",
                  "UNSAFE": "unsafe (path through a real obstacle)",
                  "GAVE_UP": "gave up"}
STRATEGIES = ("most_likely", "log_odds", "fewest_cells", "resense_all", "clear_all",
              "blind_delete")
STRATEGY_LABELS = {
    "most_likely": "Most likely (evidence weights)",
    "log_odds": "Most likely (log-odds weights)",
    "fewest_cells": "Fewest cells",
    "resense_all": "Re-sense every obstacle",
    "clear_all": "Clear map, re-scan (ROS-style)",
    "blind_delete": "Delete explanation unseen",
}
EXPLANATION_COLOR = "#2a78d6"   # categorical slot 1: cells of the chosen explanation
ALTERNATIVE_COLOR = "#86b6ef"   # lighter step of the same hue: other optimal choices
DENSITY_STEPS = ("#b7d3f6", "#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b")


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(f"'{path}' not found; run build/ghost_experiment first")
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def style(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(colors=TEXT_SECONDARY)


def plot_example(input_dir: Path, output: Path) -> None:
    """The map, every cell that belongs to some optimal explanation, the chosen one,
    and the path it opens."""
    cells = read_rows(input_dir / "example_map.csv")
    meta = read_rows(input_dir / "example_meta.csv")[0]
    width = max(int(r["x"]) for r in cells) + 1
    height = max(int(r["y"]) for r in cells) + 1
    grid = np.zeros((height, width), dtype=int)
    inflated = np.zeros((height, width), dtype=int)
    for r in cells:
        grid[int(r["y"]), int(r["x"])] = int(r["occupied"])
        inflated[int(r["y"]), int(r["x"])] = int(r["inflated"])
    solutions: dict[int, list[tuple[int, int]]] = {}
    for r in read_rows(input_dir / "example_solutions.csv"):
        solutions.setdefault(int(r["solution"]), []).append((int(r["x"]), int(r["y"])))
    path = np.array([[int(r["x"]), int(r["y"])] for r in read_rows(input_dir / "example_path.csv")])
    resolution = float(meta["resolution"])

    fig, ax = plt.subplots(figsize=(7.6, 9.3), layout="constrained")
    vm.draw_grid(ax, grid, resolution)
    extra = vm.draw_inflation(ax, grid, inflated)
    # How many optimal explanations each cell takes part in.
    counts: dict[tuple[int, int], int] = {}
    for cells_of in solutions.values():
        for c in cells_of:
            counts[c] = counts.get(c, 0) + 1
    chosen = set(solutions[0])
    for (x, y), n in counts.items():
        color = EXPLANATION_COLOR if (x, y) in chosen else ALTERNATIVE_COLOR
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor=color, edgecolor="white",
                               linewidth=1.2, zorder=3))
        ax.text(x, y, str(n), ha="center", va="center", fontsize=7,
                color="white" if (x, y) in chosen else TEXT_PRIMARY, zorder=3.5)
    ax.plot(path[:, 0], path[:, 1], color=vm.PATH_COLOR, linewidth=2.2, zorder=2.5,
            label="Path opened by the chosen explanation")
    start = (int(meta["start_x"]), int(meta["start_y"]))
    goal = (int(meta["goal_x"]), int(meta["goal_y"]))
    vm.draw_start_goal(ax, start, goal)
    handles, _ = ax.get_legend_handles_labels()
    extra += [Patch(facecolor=EXPLANATION_COLOR, label="Chosen ghost cells (remove both)"),
              Patch(facecolor=ALTERNATIVE_COLOR,
                    label="In another optimal explanation (number: how many)")]
    fig.legend(handles=vm._legend_handles() + extra + handles, loc="outside lower center",
               ncol=2, frameon=False, fontsize=9)
    fig.suptitle(f"NO_PATH for a {float(meta['robot_radius']):g} m robot: removing "
                 f"{float(meta['cost']):g} cells would open a path, and "
                 f"{meta['solutions']} different pairs would do\n"
                 f"(exact search, {float(meta['time_ms']):.1f} ms)", fontsize=11)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def summarize_recovery(rows: list[dict[str, str]]) -> dict[str, dict]:
    out = {}
    for s in STRATEGIES:
        mine = [r for r in rows if r["strategy"] == s]
        total = len(mine)
        counts = {o: sum(r["outcome"] == o for r in mine) for o in OUTCOME_COLORS}
        beams = [float(r["beams"]) for r in mine if r["outcome"] == "RECOVERED"]
        cleared = sum(int(r["real_cells_cleared"]) for r in mine)
        out[s] = {"total": total, "counts": counts,
                  "median_beams": float(np.median(beams)) if beams else float("nan"),
                  "real_cleared": cleared}
    return out


def plot_recovery(input_dir: Path, output: Path) -> None:
    """Outcome shares per strategy (stacked to 100 %) and beams spent, for two
    mapping scenarios side by side."""
    scenarios = (("thorough", "Thorough mapping (viewpoints every 2 m)"),
                 ("sparse", "Sparse mapping (every 4 m; unseen space counts as free)"))
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.2), layout="constrained",
                             width_ratios=(2.2, 1.0), sharey=True)
    y = np.arange(len(STRATEGIES))[::-1]
    for row, (key, title) in enumerate(scenarios):
        summary = summarize_recovery(read_rows(input_dir / f"ghost_recovery_{key}.csv"))
        ax, ax_b = axes[row]
        for i, s in enumerate(STRATEGIES):
            left = 0.0
            total = summary[s]["total"]
            for outcome, color in OUTCOME_COLORS.items():
                share = 100.0 * summary[s]["counts"][outcome] / total
                if share <= 0:
                    continue
                ax.barh(y[i], share, left=left, color=color, height=0.62,
                        edgecolor="white", linewidth=2)
                if share >= 7:
                    ax.text(left + share / 2, y[i], f"{share:.0f} %", ha="center",
                            va="center", fontsize=8.5,
                            color="white" if outcome != "GAVE_UP" else TEXT_PRIMARY)
                left += share
            cleared = summary[s]["real_cleared"]
            if cleared:
                ax.text(101, y[i], f"{cleared} real cell{'s' if cleared != 1 else ''} "
                        "wrongly cleared", va="center",
                        fontsize=8, color=TEXT_SECONDARY)
            beams = summary[s]["median_beams"]
            if np.isfinite(beams):
                shown = max(beams, 0.8)  # log axis: "no beams" drawn at the left edge
                ax_b.plot([0.8, shown], [y[i], y[i]], color=GRID_COLOR, linewidth=2, zorder=1)
                ax_b.plot(shown, y[i], marker="o", markersize=8, color=EXPLANATION_COLOR,
                          markeredgecolor="white", markeredgewidth=1.5, zorder=2)
                ax_b.text(shown * 1.25, y[i], "none" if beams == 0 else f"{beams:g}",
                          va="center", fontsize=8.5, color=TEXT_PRIMARY)
        ax.set_xlim(0, 135)
        ax.set_xticks([0, 25, 50, 75, 100])
        ax.set_xlabel(f"share of {summary[STRATEGIES[0]]['total']} NO_PATH cases (%)")
        ax.set_title(title, loc="left", fontsize=11, color=TEXT_PRIMARY)
        ax.set_yticks(y)
        ax.set_yticklabels([STRATEGY_LABELS[s] for s in STRATEGIES])
        ax_b.set_xscale("log")
        ax_b.set_xlim(0.6, 1e5)
        ax_b.set_xlabel("median beams when recovered (log scale)")
        ax_b.grid(axis="x", color=GRID_COLOR, linewidth=0.6)
        for a in (ax, ax_b):
            style(a)
    handles = [Patch(facecolor=c, label=OUTCOME_LABELS[o]) for o, c in OUTCOME_COLORS.items()]
    fig.legend(handles=handles, loc="outside lower center", ncol=3, frameon=False, fontsize=9.5)
    fig.suptitle("Recovering from NO_PATH caused by ghost obstacles: re-sense only the "
                 "most likely culprits", fontsize=12)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def plot_scaling(input_dir: Path, output: Path) -> None:
    """Solver time against the size of the minimum explanation (one marker per map;
    color = obstacle density), and against map size."""
    rows = [r for r in read_rows(input_dir / "ghost_scaling.csv") if r["status"] == "FOUND"]
    fig, (ax_k, ax_n) = plt.subplots(1, 2, figsize=(13, 5.4), layout="constrained",
                                     width_ratios=(1.5, 1.0))
    base = [r for r in rows if r["map_cells"] == "1600" and float(r["radius_m"]) == 0.3
            and r["connectivity"] == "4"]
    densities = sorted({float(r["density"]) for r in base})
    rng = np.random.default_rng(0)
    for color, density in zip(DENSITY_STEPS, densities, strict=False):
        pts = [r for r in base if float(r["density"]) == density]
        k = np.array([int(r["ghost_cells"]) for r in pts], dtype=float)
        t = np.array([float(r["solver_ms"]) for r in pts])
        ax_k.scatter(k + rng.uniform(-0.18, 0.18, len(k)), t, s=22, color=color,
                     edgecolor="white", linewidth=0.6, label=f"{100 * density:g} %", zorder=3)
    brute = [float(r["bruteforce_ms"]) for r in base if r["bruteforce_ms"]]
    exact_small = [float(r["solver_ms"]) for r in base if r["bruteforce_ms"]]
    subtitle = ""
    if brute:
        subtitle = (f"\nwhere brute force can check (1-2 cells): median {np.median(brute):.1f} ms "
                    f"brute force vs. {np.median(exact_small):.1f} ms exact")
    ax_k.set_yscale("log")
    ax_k.set_xlabel("minimum number of ghost cells (explanation size)")
    ax_k.set_ylabel("exact solver time per map (ms, log scale)")
    ax_k.set_title("40 x 40 cells at 0.25 m, robot radius 0.3 m (one dot per NO_PATH map)"
                   + subtitle, loc="left", fontsize=10)
    ax_k.legend(title="obstacle density", frameon=False, fontsize=8.5, title_fontsize=9,
                loc="lower right")
    ax_k.grid(color=GRID_COLOR, linewidth=0.6)
    sizes = [r for r in rows if float(r["density"]) == 0.06 and float(r["radius_m"]) == 0.3
             and r["connectivity"] == "4"]
    cell_counts = sorted({int(r["map_cells"]) for r in sizes})
    data = [[float(r["solver_ms"]) for r in sizes if int(r["map_cells"]) == n] for n in cell_counts]
    positions = np.arange(len(cell_counts))
    for p, values in zip(positions, data, strict=True):
        ax_n.scatter(p + rng.uniform(-0.15, 0.15, len(values)), values, s=16,
                     color=EXPLANATION_COLOR, alpha=0.6, edgecolor="none", zorder=3)
        ax_n.plot([p - 0.28, p + 0.28], [np.median(values)] * 2, color=TEXT_PRIMARY,
                  linewidth=2, zorder=4)
    ax_n.set_xticks(positions)
    ax_n.set_xticklabels([f"{int(np.sqrt(n))}x{int(np.sqrt(n))}" for n in cell_counts])
    ax_n.set_yscale("log")
    ax_n.set_xlabel("map size (cells), density 6 %")
    ax_n.set_ylabel("solver time (ms, log scale)")
    ax_n.set_title("Map size (one dot per NO_PATH map; bar: median)", loc="left", fontsize=10)
    ax_n.grid(color=GRID_COLOR, linewidth=0.6)
    for a in (ax_k, ax_n):
        style(a)
    fig.suptitle("Exact minimum-ghost search: milliseconds for small explanations, "
                 "growing steeply with their size (the problem is NP-hard)", fontsize=12)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot the ghost-cell (NO_PATH) experiments.")
    parser.add_argument("--input", type=Path, default=REPO_ROOT / "docs" / "ghost",
                        help="directory written by ghost_experiment (default docs/ghost)")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "docs" / "images",
                        help="where to save the figures (default docs/images)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plt.switch_backend("Agg")
    try:
        plot_example(args.input, args.output_dir / "ghost_example.png")
        plot_recovery(args.input, args.output_dir / "ghost_recovery.png")
        plot_scaling(args.input, args.output_dir / "ghost_scaling.png")
    except FileNotFoundError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
