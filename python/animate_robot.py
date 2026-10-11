#!/usr/bin/env python3
"""Animate the simulated robot using the trajectory recorded by the C++ simulator.

The trajectory CSV is the source of truth. Each animation frame shows a
simulation time t, and the robot is drawn at the trajectory position at t
(linear interpolation between the recorded samples, which is exact because the
robot moves in a straight line within each timestep). The robot therefore moves
according to the recorded timestamps, never by jumping between waypoints.

Playback: one second of animation shows `playback_speed` seconds of simulation.
By default the speed is chosen so the animation lasts about 12 s (never slower
than real time). The last frame is held briefly to show the final status.

If planning failed (no trajectory), nothing is animated: a static map with a
"No valid path" message is saved as navigation_failed.png instead.

Reads (from --results-dir, written by build/robot_simulator):
    planning_metrics.csv, path.csv, trajectory.csv, simulation_metrics.csv

Writes (to --output-dir, default --results-dir):
    navigation.gif           with --save-gif (PillowWriter; the default output)
    navigation.mp4           with --save-mp4 (needs FFmpeg)
    navigation_failed.png    when there is no trajectory to animate

Example:
    ./build/robot_simulator --map maps/obstacles.csv --start 1 1 --goal 18 18
    python3 python/animate_robot.py --map maps/obstacles.csv --save-gif
"""

from __future__ import annotations

import argparse
import contextlib
import sys
from pathlib import Path
from collections.abc import Callable
from typing import NamedTuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.backend_bases import FigureCanvasBase
from PIL import Image

import visualize_map as vm

DEFAULT_FPS = 15
TARGET_ANIMATION_DURATION_S = 12.0   # used to pick the default playback speed
END_HOLD_S = 1.5                     # final frame is held this long
ANIMATION_DPI = 80                   # lower than the static figures: smaller GIFs
START_TOLERANCE_M = 1e-6
STATUS_EDGE_COLORS = {"GOAL_REACHED": vm.START_COLOR}  # every other status: GOAL_COLOR


class AnimationError(Exception):
    """Raised when an animation cannot be produced."""


class RobotAnimation(NamedTuple):
    figure: plt.Figure
    animation: animation.FuncAnimation
    frame_count: int
    update: Callable[[int], tuple]   # draws frame k's state into the artists; returns them


def default_playback_speed(sim_duration_s: float) -> float:
    """Simulation seconds per animation second: about TARGET_ANIMATION_DURATION_S
    of animation, but never slower than real time."""
    return max(sim_duration_s / TARGET_ANIMATION_DURATION_S, 1.0)


def frame_times(start_s: float, end_s: float, fps: float,
                playback_speed: float) -> np.ndarray:
    """Simulation time shown in each moving frame, from start_s to exactly end_s.

    Frames are evenly spaced and at most playback_speed / fps apart.
    """
    duration = end_s - start_s
    if duration <= 0.0:
        return np.array([start_s])
    intervals = int(np.ceil(duration * fps / playback_speed - 1e-9))
    return np.linspace(start_s, end_s, max(intervals, 1) + 1)


def position_at(trajectory: np.ndarray, t: float) -> tuple[float, float]:
    """Robot position (m) at time t, interpolated from the recorded samples."""
    times = trajectory[:, vm.TRAJ_TIME]
    return (float(np.interp(t, times, trajectory[:, vm.TRAJ_X])),
            float(np.interp(t, times, trajectory[:, vm.TRAJ_Y])))


def traveled_until(trajectory: np.ndarray, t: float) -> np.ndarray:
    """(M, 2) world points traveled up to time t, ending at the robot position."""
    times = trajectory[:, vm.TRAJ_TIME]
    passed = trajectory[times <= t][:, [vm.TRAJ_X, vm.TRAJ_Y]]
    current = np.array([position_at(trajectory, t)])
    return np.vstack([passed, current])


def target_waypoint_at(trajectory: np.ndarray, t: float) -> int:
    """Waypoint index the robot is heading to at time t."""
    times = trajectory[:, vm.TRAJ_TIME]
    index = max(int(np.searchsorted(times, t, side="right")) - 1, 0)
    return int(trajectory[index, vm.TRAJ_TARGET])


def check_trajectory_matches(metrics: vm.PlanningMetrics,
                             trajectory: np.ndarray) -> None:
    """The trajectory must start at the planned start-cell center."""
    start_world = (np.array(metrics.start) + 0.5) * metrics.resolution
    first = trajectory[0, [vm.TRAJ_X, vm.TRAJ_Y]]
    if np.abs(first - start_world).max() > START_TOLERANCE_M:
        raise vm.ResultFileError(
            "trajectory.csv does not start at the planned start cell; "
            "re-run build/robot_simulator")


def build_animation(grid: np.ndarray, metrics: vm.PlanningMetrics,
                    waypoints: np.ndarray, trajectory: np.ndarray,
                    sim: vm.SimulationMetrics, fps: float, playback_speed: float,
                    dpi: int = ANIMATION_DPI):
    """Create the figure, the per-frame update function and a FuncAnimation."""
    res = metrics.resolution
    t_start = trajectory[0, vm.TRAJ_TIME]
    t_end = trajectory[-1, vm.TRAJ_TIME]
    moving_times = frame_times(t_start, t_end, fps, playback_speed)
    hold_frames = int(round(END_HOLD_S * fps))
    times = np.concatenate([moving_times, np.full(hold_frames, t_end)])
    last_waypoint = len(waypoints) - 1

    # The figure is created at the export dpi so saving needs no rescaling.
    fig, ax = plt.subplots(figsize=vm.FIGURE_SIZE, dpi=dpi, layout="constrained")
    vm.draw_grid(ax, grid, res)
    ax.plot(waypoints[:, 0], waypoints[:, 1], color=vm.PATH_COLOR, linewidth=6,
            alpha=0.35, solid_capstyle="round", label="Planned A* path", zorder=2)
    vm.draw_start_goal(ax, metrics.start, metrics.goal)
    (trail,) = ax.plot([], [], color=vm.TRAJECTORY_COLOR, linewidth=2,
                       label="Robot trajectory", zorder=3)
    (robot,) = ax.plot([], [], marker="o", markersize=13, color=vm.ROBOT_COLOR,
                       markeredgecolor="black", markeredgewidth=1.0, linestyle="none",
                       label="Robot", zorder=3.5)
    text_box = dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.9,
                    edgecolor=vm.GRID_LINE_COLOR)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, ha="left", va="top",
                        fontsize=11, family="monospace", zorder=6, bbox=text_box)
    status_text = ax.text(0.98, 0.98, "", transform=ax.transAxes, ha="right",
                          va="top", fontsize=11, fontweight="bold", zorder=6,
                          bbox=dict(text_box))
    fig.suptitle(f"Robot navigation  ({sim.speed_mps:g} m/s, dt {sim.dt_s:g} s, "
                 f"playback {playback_speed:.1f}x)")
    handles, _ = ax.get_legend_handles_labels()
    fig.legend(handles=vm._legend_handles() + handles, loc="outside lower center",
               ncol=3, frameon=False)
    # Compute the layout once, then freeze it: re-running constrained layout on
    # every frame would roughly double the rendering time.
    fig.canvas.draw()
    fig.set_layout_engine("none")

    def update(frame: int):
        t = float(times[frame])
        x, y = position_at(trajectory, t)
        traveled = traveled_until(trajectory, t)
        robot.set_data([vm.world_to_grid(x, res)], [vm.world_to_grid(y, res)])
        trail.set_data(vm.world_to_grid(traveled[:, 0], res),
                       vm.world_to_grid(traveled[:, 1], res))
        time_text.set_text(f"t = {t - t_start:5.1f} s / {t_end - t_start:.1f} s")
        if frame < len(moving_times) - 1:
            target = target_waypoint_at(trajectory, t)
            status_text.set_text(f"NAVIGATING  waypoint {target}/{last_waypoint}")
            status_text.get_bbox_patch().set_edgecolor(vm.GRID_LINE_COLOR)
        else:
            status_text.set_text(sim.status)
            status_text.get_bbox_patch().set_edgecolor(
                STATUS_EDGE_COLORS.get(sim.status, vm.GOAL_COLOR))
        return robot, trail, time_text, status_text

    anim = animation.FuncAnimation(fig, update, frames=len(times),
                                   interval=1000.0 / fps, blit=True, repeat=True)
    return RobotAnimation(fig, anim, len(times), update)


def canvas_draws_synchronously(canvas) -> bool:
    """True for canvases (Agg) whose buffer is current right after drawing."""
    return type(canvas).draw_idle is FigureCanvasBase.draw_idle


class CanvasPillowWriter(animation.PillowWriter):
    """PillowWriter that takes the frame from the canvas buffer.

    The stock grab_frame() renders the whole figure again through savefig. On a
    synchronous (Agg) canvas the buffer already holds the frame that was just drawn,
    so it is copied directly. The output is pixel-identical; interactive canvases
    fall back to the standard behaviour.
    """

    def grab_frame(self, **savefig_kwargs):
        canvas = self.fig.canvas
        if not canvas_draws_synchronously(canvas):
            super().grab_frame(**savefig_kwargs)
            return
        buffer = canvas.buffer_rgba()
        height, width = buffer.shape[:2]
        frame = Image.frombuffer("RGBA", (width, height), bytes(buffer), "raw", "RGBA", 0, 1)
        # Same as PillowWriter: opaque frames go through RGB, which quantizes to the
        # GIF palette better than RGBA.
        self._frames.append(frame if frame.getextrema()[3][0] < 255 else frame.convert("RGB"))


class CanvasFFMpegWriter(animation.FFMpegWriter):
    """FFMpegWriter that pipes the canvas buffer instead of re-rendering via savefig.

    FFMpegWriter's frames are raw RGBA figure renders at the figure dpi, which is
    exactly what the Agg canvas buffer holds after a draw.
    """

    def grab_frame(self, **savefig_kwargs):
        canvas = self.fig.canvas
        if not canvas_draws_synchronously(canvas) or self.frame_format != "rgba":
            super().grab_frame(**savefig_kwargs)
            return
        self._proc.stdin.write(bytes(canvas.buffer_rgba()))


def render_frames_blitted(robot_anim: RobotAnimation):
    """Render every frame into the canvas, yielding after each one.

    Blitting: the static background (grid, axes, legend, planned path) is drawn
    once; each frame restores it and redraws only the changing artists plus every
    artist that lies above them in z-order (e.g. the start/goal markers), in the
    same z-order a full render uses. The frames are pixel-identical to full
    renders and about 20x cheaper.
    """
    fig = robot_anim.figure
    ax = fig.axes[0]
    dynamic = list(robot_anim.update(0))
    lowest = min(artist.get_zorder() for artist in dynamic)
    redraw = sorted((a for a in ax.get_children()
                     if a in dynamic or (a.get_visible() and a.get_zorder() >= lowest)),
                    key=lambda a: a.get_zorder())  # stable: keeps creation order on ties
    previous = [a.get_animated() for a in redraw]
    for artist in redraw:
        artist.set_animated(True)   # excluded from the background render
    try:
        canvas = fig.canvas
        canvas.draw()
        background = canvas.copy_from_bbox(fig.bbox)
        for frame in range(robot_anim.frame_count):
            canvas.restore_region(background)
            robot_anim.update(frame)
            for artist in redraw:
                ax.draw_artist(artist)
            yield frame
    finally:
        for artist, was_animated in zip(redraw, previous, strict=True):
            artist.set_animated(was_animated)


def export_animation(robot_anim: RobotAnimation, outputs: list[tuple[Path, object]]) -> None:
    """Write all requested files, rendering each frame only once."""
    fig = robot_anim.figure
    for path, _ in outputs:
        path.parent.mkdir(parents=True, exist_ok=True)
    if canvas_draws_synchronously(fig.canvas):
        with contextlib.ExitStack() as stack:
            for path, writer in outputs:
                stack.enter_context(writer.saving(fig, path, fig.dpi))
            for _ in render_frames_blitted(robot_anim):
                for _, writer in outputs:
                    writer.grab_frame()
    else:  # interactive backend (--show): standard Animation.save per file
        for path, writer in outputs:
            robot_anim.animation.save(path, writer=writer)
    for path, _ in outputs:
        print(f"Saved {path} ({path.stat().st_size / 1024:.0f} KB)")


def remove_stale_outputs(output_dir: Path) -> None:
    """Delete animations from an earlier run so they cannot be mistaken for this one."""
    for name in ("navigation.gif", "navigation.mp4"):
        stale = output_dir / name
        if stale.is_file():
            stale.unlink()
            print(f"Removed stale {stale} (it belonged to an earlier run)")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Animate the robot trajectory produced by build/robot_simulator.")
    parser.add_argument("--map", type=Path, required=True, help="map CSV file")
    parser.add_argument("--results-dir", type=Path, default=Path("results"),
                        help="directory with the robot_simulator CSV outputs "
                             "(default results/)")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="where to save animations (default: --results-dir)")
    parser.add_argument("--save-gif", action="store_true",
                        help="save navigation.gif (default when no other output "
                             "is requested)")
    parser.add_argument("--save-mp4", action="store_true",
                        help="save navigation.mp4 (requires FFmpeg)")
    parser.add_argument("--show", action="store_true",
                        help="play the animation in a window")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS,
                        help=f"animation frames per second (default {DEFAULT_FPS})")
    parser.add_argument("--playback-speed", type=float, default=None,
                        help="simulation seconds per animation second "
                             "(default: about a 12 s animation)")
    parser.add_argument("--dpi", type=int, default=ANIMATION_DPI,
                        help=f"resolution of saved animations (default {ANIMATION_DPI})")
    args = parser.parse_args(argv)
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.playback_speed is not None and args.playback_speed <= 0:
        parser.error("--playback-speed must be positive")
    if args.dpi <= 0:
        parser.error("--dpi must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.show:
        plt.switch_backend("Agg")  # file output only; no display needed
    output_dir = args.output_dir or args.results_dir
    save_gif = args.save_gif or not (args.show or args.save_mp4)

    try:
        grid = vm.load_map(args.map)
        metrics = vm.load_metrics(args.results_dir / "planning_metrics.csv")
        waypoints = vm.load_path(args.results_dir / "path.csv")
        vm.check_results_match_map(grid, metrics, waypoints, args.map)
        trajectory = vm.load_trajectory(args.results_dir / "trajectory.csv")
        sim = vm.load_simulation_metrics(args.results_dir / "simulation_metrics.csv")
        if len(trajectory) > 0:
            check_trajectory_matches(metrics, trajectory)
    except vm.ResultFileError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if not metrics.success or len(trajectory) == 0:
        print(f"No valid path ({metrics.status}): there is no robot motion to animate")
        remove_stale_outputs(output_dir)
        vm.plot_planned_path(grid, metrics, waypoints, output_dir / "navigation_failed.png")
        if args.show:
            plt.show()
        return 0

    duration = trajectory[-1, vm.TRAJ_TIME] - trajectory[0, vm.TRAJ_TIME]
    playback = args.playback_speed or default_playback_speed(duration)
    robot_anim = build_animation(grid, metrics, waypoints, trajectory,
                                 sim, args.fps, playback, args.dpi)
    print(f"Animating {duration:.1f} s of simulation at {playback:.2f}x: "
          f"{robot_anim.frame_count} frames at {args.fps:g} fps")

    status = 0
    outputs = []
    if save_gif:
        outputs.append((output_dir / "navigation.gif", CanvasPillowWriter(fps=args.fps)))
    if args.save_mp4:
        if animation.FFMpegWriter.isAvailable():
            outputs.append((output_dir / "navigation.mp4", CanvasFFMpegWriter(fps=args.fps)))
        else:
            print("Error: MP4 export needs FFmpeg (sudo apt install ffmpeg); "
                  "GIF export works without it", file=sys.stderr)
            status = 1
    if outputs:
        export_animation(robot_anim, outputs)
    if args.show:
        plt.show()
    plt.close(robot_anim.figure)
    return status


if __name__ == "__main__":
    sys.exit(main())
