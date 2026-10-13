#!/usr/bin/env python3
"""One-command demonstration: plan, simulate, visualize and animate.

Steps:
    1. run the compiled C++ simulator (build/robot_simulator), which plans an A*
       path, simulates the robot and writes CSV results;
    2. save the static figures (visualize_map.py);
    3. optionally animate the robot (animate_robot.py): --animate opens a window,
       --save-gif / --save-mp4 write files.

The simulator is started with subprocess and an argument list (never a shell
command string), so map paths and other arguments are passed through verbatim.

Examples:
    python3 python/run_demo.py --save-gif
    python3 python/run_demo.py --map maps/obstacles.csv --start 1 1 --goal 18 18 \\
        --animate --save-gif
    python3 python/run_demo.py --map maps/maze.csv --start 1 1 --goal 19 19 \\
        --speed 1.0 --save-gif
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import animate_robot
import visualize_map

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SIMULATOR = REPO_ROOT / "build" / "robot_simulator"

# robot_simulator exit codes after which results exist and should be visualized:
# 0 goal reached, 3 invalid start/goal, 4 no path, 6 navigation failed.
EXIT_CODES_WITH_RESULTS = {0, 3, 4, 6}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan, simulate and visualize a robot navigation run.")
    parser.add_argument("--map", type=Path, default=REPO_ROOT / "maps" / "obstacles.csv",
                        help="map CSV (default maps/obstacles.csv)")
    parser.add_argument("--start", type=int, nargs=2, default=[1, 1], metavar=("X", "Y"),
                        help="start cell (default 1 1)")
    parser.add_argument("--goal", type=int, nargs=2, default=[18, 18], metavar=("X", "Y"),
                        help="goal cell (default 18 18)")
    parser.add_argument("--resolution", type=float, help="meters per cell (default 0.5)")
    parser.add_argument("--speed", type=float, help="robot speed in m/s (default 0.5)")
    parser.add_argument("--dt", type=float, help="simulation timestep in s (default 0.1)")
    parser.add_argument("--goal-tolerance", type=float,
                        help="allowed final distance to the goal in m (default 0.05)")
    parser.add_argument("--max-time", type=float,
                        help="simulation time limit in s (default 600)")
    parser.add_argument("--robot-model", choices=("point", "diff-drive"),
                        help="robot motion model (default point)")
    parser.add_argument("--max-angular-speed", type=float,
                        help="differential drive: angular speed limit in rad/s (default 1.5)")
    parser.add_argument("--heading-gain", type=float,
                        help="differential drive: heading P gain in 1/s (default 5.0)")
    parser.add_argument("--initial-heading", type=float,
                        help="differential drive: heading at t = 0 in degrees (default 0)")
    parser.add_argument("--lidar-beams", type=int,
                        help="simulated LiDAR beams (default 0 = off)")
    parser.add_argument("--lidar-fov", type=float, help="LiDAR field of view, deg (default 360)")
    parser.add_argument("--lidar-range", type=float, help="LiDAR maximum range, m (default 5)")
    parser.add_argument("--lidar-noise", type=float,
                        help="LiDAR Gaussian range noise, m (default 0)")
    parser.add_argument("--lidar-seed", type=int, help="LiDAR noise seed (default 0)")
    parser.add_argument("--ekf", action="store_true",
                        help="EKF localization from noisy odometry (and the LiDAR, if on); "
                             "needs --robot-model diff-drive")
    parser.add_argument("--odom-noise-dist", type=float,
                        help="odometry distance error, m per sqrt(m) (default 0.05)")
    parser.add_argument("--odom-noise-turn", type=float,
                        help="odometry heading error, rad per sqrt(rad) turned (default 0.05)")
    parser.add_argument("--odom-noise-drift", type=float,
                        help="odometry heading drift, rad per sqrt(m) driven (default 0.02)")
    parser.add_argument("--odom-seed", type=int, help="odometry noise seed (default 0)")
    parser.add_argument("--ekf-range-sigma", type=float,
                        help="LiDAR range std assumed by the EKF, m (default 0.05)")
    parser.add_argument("--ekf-gate", type=float,
                        help="EKF innovation gate, standard deviations (default 3)")
    parser.add_argument("--dynamic-obstacles", type=Path,
                        help="moving obstacles file (radius_m,speed_mps,x0,y0,x1,y1,... per line)")
    parser.add_argument("--sensing-range", type=float,
                        help="dynamic obstacles are seen within this range, m (default 3)")
    parser.add_argument("--replan-margin", type=float,
                        help="clearance around a seen obstacle when replanning, m (default 0.25)")
    parser.add_argument("--prediction-horizon", type=float,
                        help="seconds of estimated obstacle motion to avoid (default 2)")
    parser.add_argument("--replan-patience", type=float,
                        help="wait this long for a blocked path before a detour, s (default 6)")
    parser.add_argument("--connectivity", type=int, choices=(4, 8),
                        help="grid moves: 4 = straight (default), 8 = also diagonal")
    parser.add_argument("--robot-radius", type=float,
                        help="robot footprint radius in m (default 0 = point robot)")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "results",
                        help="results directory (default results/)")
    parser.add_argument("--animate", action="store_true",
                        help="play the robot animation in a window")
    parser.add_argument("--save-gif", action="store_true",
                        help="save the animation as navigation.gif")
    parser.add_argument("--save-mp4", action="store_true",
                        help="save the animation as navigation.mp4 (requires FFmpeg)")
    parser.add_argument("--fps", type=float, help="animation frames per second")
    parser.add_argument("--playback-speed", type=float,
                        help="simulation seconds per animation second")
    parser.add_argument("--simulator", type=Path, default=DEFAULT_SIMULATOR,
                        help="path to the robot_simulator executable "
                             "(default build/robot_simulator)")
    return parser.parse_args(argv)


def simulator_command(args: argparse.Namespace) -> list[str]:
    """Argument list for robot_simulator. Optional values are only passed when set,
    so the C++ defaults stay the single source of truth."""
    command = [str(args.simulator), "--map", str(args.map),
               "--start", str(args.start[0]), str(args.start[1]),
               "--goal", str(args.goal[0]), str(args.goal[1]),
               "--output", str(args.output)]
    optional = (("--resolution", args.resolution), ("--speed", args.speed),
                ("--dt", args.dt), ("--goal-tolerance", args.goal_tolerance),
                ("--max-time", args.max_time), ("--robot-radius", args.robot_radius),
                ("--connectivity", args.connectivity), ("--robot-model", args.robot_model),
                ("--max-angular-speed", args.max_angular_speed),
                ("--heading-gain", args.heading_gain),
                ("--initial-heading", args.initial_heading),
                ("--lidar-beams", args.lidar_beams), ("--lidar-fov", args.lidar_fov),
                ("--lidar-range", args.lidar_range), ("--lidar-noise", args.lidar_noise),
                ("--lidar-seed", args.lidar_seed),
                ("--odom-noise-dist", args.odom_noise_dist),
                ("--odom-noise-turn", args.odom_noise_turn),
                ("--odom-noise-drift", args.odom_noise_drift), ("--odom-seed", args.odom_seed),
                ("--ekf-range-sigma", args.ekf_range_sigma), ("--ekf-gate", args.ekf_gate),
                ("--dynamic-obstacles",
                 None if args.dynamic_obstacles is None else str(args.dynamic_obstacles)),
                ("--sensing-range", args.sensing_range), ("--replan-margin", args.replan_margin),
                ("--prediction-horizon", args.prediction_horizon),
                ("--replan-patience", args.replan_patience))
    for flag, value in optional:
        if value is not None:
            command += [flag, value if isinstance(value, str) else repr(value)]
    if args.ekf:
        command.append("--ekf")
    return command


def run_simulator(args: argparse.Namespace) -> int:
    if not args.simulator.is_file():
        print(f"Error: simulator not found at '{args.simulator}'. Build it first:\n"
              f"    cmake -S . -B build && cmake --build build -j", file=sys.stderr)
        return 127
    print("$ " + " ".join(simulator_command(args)), flush=True)
    completed = subprocess.run(simulator_command(args), check=False)  # no shell
    return completed.returncode


def animation_args(args: argparse.Namespace) -> list[str] | None:
    """Arguments for animate_robot.main(), or None if no animation was requested."""
    if not (args.animate or args.save_gif or args.save_mp4):
        return None
    argv = ["--map", str(args.map), "--results-dir", str(args.output)]
    if args.animate:
        argv.append("--show")
    if args.save_gif:
        argv.append("--save-gif")
    if args.save_mp4:
        argv.append("--save-mp4")
    if args.fps is not None:
        argv += ["--fps", repr(args.fps)]
    if args.playback_speed is not None:
        argv += ["--playback-speed", repr(args.playback_speed)]
    return argv


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    sim_status = run_simulator(args)
    if sim_status not in EXIT_CODES_WITH_RESULTS:
        print(f"robot_simulator failed with exit code {sim_status}; "
              f"nothing to visualize", file=sys.stderr)
        return sim_status

    print("\n--- Static figures ---", flush=True)
    viz_status = visualize_map.main(["--map", str(args.map),
                                     "--results-dir", str(args.output)])
    anim_argv = animation_args(args)
    anim_status = 0
    if anim_argv is not None:
        print("\n--- Animation ---", flush=True)
        anim_status = animate_robot.main(anim_argv)

    # The simulator's own outcome (e.g. 4 = no path) takes precedence.
    return sim_status or viz_status or anim_status


if __name__ == "__main__":
    sys.exit(main())
