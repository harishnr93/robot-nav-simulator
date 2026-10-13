#!/usr/bin/env python3
"""Plot the P vs. PID heading-control experiment written by build/heading_experiment.

Reads (from --input, default results/heading/):
    heading_metrics.csv
    heading_<scenario>_p.csv, heading_<scenario>_pid.csv   time,reference,theta,...

Writes (to --output-dir, default: --input):
    heading_<scenario>.png   heading vs. reference and commanded turn rate, P and PID
    heading_metrics.png      overshoot, settling time, steady-state error, IAE,
                             control effort and command variation side by side

Example:
    ./build/heading_experiment
    python3 python/plot_heading_control.py
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

FIGURE_DPI = 120
CONTROLLER_COLORS = {"P": "#3a7dbf", "PID": "#d9772b"}   # categorical, fixed order
REFERENCE_COLOR = "#6b6b6b"
BAND_COLOR = "#d9d9d9"
GRID_COLOR = "#dddddd"
SCENARIO_TITLES = {
    "step": "90° step",
    "step_disturbance": "90° step with a constant disturbance",
    "ramp": "Ramp: following an arc at constant turn rate",
    "path_profile": "Heading profile of a planned path (8-connected, obstacles map)",
}
SETTLING_BAND_FRACTION = 0.02
RUN_COLUMNS = ("time", "reference", "theta", "error", "command", "omega")
METRIC_COLUMNS = ("scenario", "controller", "kp", "ki", "kd", "tau_s", "disturbance_radps",
                  "max_omega_radps", "overshoot_pct", "settling_time_s",
                  "steady_state_error_rad", "iae_rad_s", "effort_abs_rad",
                  "command_variation_radps", "duration_s")


class HeadingResultError(Exception):
    """Raised when result files are missing or malformed."""


def load_metrics(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise HeadingResultError(f"'{path}' not found; run build/heading_experiment first")
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = set(METRIC_COLUMNS) - set(reader.fieldnames or ())
        if missing:
            raise HeadingResultError(f"'{path}' is missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if not rows:
        raise HeadingResultError(f"'{path}' has no rows")
    return rows


def load_run(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise HeadingResultError(f"'{path}' not found")
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None or tuple(header) != RUN_COLUMNS:
            raise HeadingResultError(f"'{path}' has unexpected columns {header}")
        try:
            data = np.array([[float(v) for v in row] for row in reader], dtype=float)
        except ValueError as error:
            raise HeadingResultError(f"'{path}' has a malformed value: {error}") from error
    if data.ndim != 2 or len(data) < 2:
        raise HeadingResultError(f"'{path}' has too few samples")
    return {name: data[:, i] for i, name in enumerate(RUN_COLUMNS)}


def _style(ax) -> None:
    ax.grid(color=GRID_COLOR, linewidth=0.6)
    ax.spines[["top", "right"]].set_visible(False)


def plot_scenario(scenario: str, runs: dict[str, dict[str, np.ndarray]],
                  metrics: dict[str, dict[str, str]], output: Path) -> None:
    """Heading (deg) vs. reference, and commanded turn rate (rad/s), for P and PID."""
    fig, (ax_h, ax_u) = plt.subplots(2, 1, figsize=(8.5, 6.4), sharex=True,
                                     layout="constrained", height_ratios=(3, 2))
    first = next(iter(runs.values()))
    reference = np.degrees(first["reference"])
    ax_h.plot(first["time"], reference, color=REFERENCE_COLOR, linestyle="--",
              linewidth=1.6, label="reference")
    if scenario.startswith("step"):
        target = reference[-1]
        band = SETTLING_BAND_FRACTION * abs(target)
        ax_h.axhspan(target - band, target + band, color=BAND_COLOR, alpha=0.6, linewidth=0,
                     label="±2 % settling band")
    for controller, run in runs.items():
        m = metrics[controller]
        color = CONTROLLER_COLORS[controller]
        label = controller
        if m["overshoot_pct"]:
            settle = m["settling_time_s"]
            label += (f": overshoot {float(m['overshoot_pct']):.1f} %, "
                      + (f"settles in {float(settle):.2f} s" if settle else "never settles"))
        else:
            label += f": IAE {float(m['iae_rad_s']):.2f} rad·s"
        ax_h.plot(run["time"], np.degrees(run["theta"]), color=color, linewidth=2, label=label)
        # The command is held over each control period (zero-order hold); the last
        # sample's command is never applied, so it is not drawn.
        ax_u.step(run["time"][:-1], run["command"][:-1], where="post", color=color,
                  linewidth=1.6,
                  label=f"{controller}: effort ∫|u| = {float(m['effort_abs_rad']):.2f} rad")
    limit = float(next(iter(metrics.values()))["max_omega_radps"])
    for sign in (1, -1):
        ax_u.axhline(sign * limit, color=REFERENCE_COLOR, linestyle=":", linewidth=1.2)
    ax_u.text(first["time"][0], limit, "limit ", va="bottom", ha="left",
              color=REFERENCE_COLOR, fontsize=9)
    ax_h.set_ylabel("heading θ (deg)")
    ax_u.set_ylabel("commanded turn rate u (rad/s)")
    ax_u.set_xlabel("time (s)")
    _style(ax_h)
    _style(ax_u)
    ax_h.legend(frameon=False, loc="best", fontsize=9)
    ax_u.legend(frameon=False, loc="best", fontsize=9)
    any_metrics = next(iter(metrics.values()))
    disturbance = float(any_metrics["disturbance_radps"])
    gains = metrics["PID"]
    fig.suptitle(f"{SCENARIO_TITLES.get(scenario, scenario)}\n"
                 f"τ = {float(any_metrics['tau_s']):g} s"
                 + (f", disturbance {disturbance:g} rad/s" if disturbance else "")
                 + f"; P: Kp {float(metrics['P']['kp']):g}; PID: Kp {float(gains['kp']):g}, "
                   f"Ki {float(gains['ki']):g}, Kd {float(gains['kd']):g}", fontsize=11)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def plot_metrics(rows: list[dict[str, str]], output: Path) -> None:
    """Small multiples: one metric per panel, P and PID side by side per scenario."""
    scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
    by_key = {(r["scenario"], r["controller"]): r for r in rows}
    panels = [
        ("overshoot_pct", "overshoot (%)", lambda v: v, True),
        ("settling_time_s", "settling time (s)", lambda v: v, True),
        ("steady_state_error_rad", "|steady-state error| (deg)", lambda v: abs(np.degrees(v)), False),
        ("iae_rad_s", "IAE ∫|e| dt (rad·s)", lambda v: v, False),
        ("effort_abs_rad", "control effort ∫|u| dt (rad)", lambda v: v, False),
        ("command_variation_radps", "command variation Σ|Δu| (rad/s)", lambda v: v, False),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.2), layout="constrained")
    width = 0.38
    for ax, (column, title, transform, step_only) in zip(axes.flat, panels, strict=True):
        shown = [s for s in scenarios if not step_only or s.startswith("step")]
        x = np.arange(len(shown))
        for i, controller in enumerate(("P", "PID")):
            values, labels = [], []
            for scenario in shown:
                text = by_key[(scenario, controller)][column]
                if text == "":
                    values.append(np.nan)
                    labels.append("never" if column == "settling_time_s" else "–")
                else:
                    values.append(transform(float(text)))
                    labels.append(f"{values[-1]:.2f}")
            positions = x + (i - 0.5) * width
            heights = np.nan_to_num(np.array(values, dtype=float), nan=0.0)
            bars = ax.bar(positions, heights, width * 0.92, color=CONTROLLER_COLORS[controller],
                          label=controller)
            for bar, label in zip(bars, labels, strict=True):
                ax.annotate(label, (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                            xytext=(0, 2), textcoords="offset points", ha="center",
                            va="bottom", fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels([s.replace("_", "\n") for s in shown], fontsize=9)
        ax.set_title(title, fontsize=10)
        ax.margins(y=0.15)
        _style(ax)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper right", frameon=False)
    fig.suptitle("P vs. PID heading control: metrics per scenario (lower is better)",
                 fontsize=12)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=FIGURE_DPI)
    plt.close(fig)
    print(f"Saved {output}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot the P vs. PID heading experiment.")
    parser.add_argument("--input", type=Path, default=Path("results/heading"),
                        help="directory written by heading_experiment (default results/heading)")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="where to save the figures (default: --input)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plt.switch_backend("Agg")
    output_dir = args.output_dir or args.input
    try:
        rows = load_metrics(args.input / "heading_metrics.csv")
        scenarios = list(dict.fromkeys(r["scenario"] for r in rows))
        for scenario in scenarios:
            metrics = {r["controller"]: r for r in rows if r["scenario"] == scenario}
            runs = {c: load_run(args.input / f"heading_{scenario}_{c.lower()}.csv")
                    for c in ("P", "PID")}
            plot_scenario(scenario, runs, metrics, output_dir / f"heading_{scenario}.png")
        plot_metrics(rows, output_dir / "heading_metrics.png")
    except HeadingResultError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
