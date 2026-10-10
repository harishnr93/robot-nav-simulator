#!/usr/bin/env python3
"""Tests for the Python visualization and animation modules.

Runs under CTest (see CMakeLists.txt), which sets:
    AMR_SOURCE_DIR   repository root
    AMR_SIMULATOR    path to the built robot_simulator executable
Can also be run directly from the repository root after building:
    python3 tests/test_python_visualization.py
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

SOURCE_DIR = Path(os.environ.get("AMR_SOURCE_DIR", Path(__file__).resolve().parent.parent))
SIMULATOR = Path(os.environ.get("AMR_SIMULATOR", SOURCE_DIR / "build" / "robot_simulator"))
sys.path.insert(0, str(SOURCE_DIR / "python"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

import animate_robot as ar  # noqa: E402
import run_demo  # noqa: E402
import visualize_map as vm  # noqa: E402

OBSTACLES_MAP = SOURCE_DIR / "maps" / "obstacles.csv"
UNREACHABLE_MAP = SOURCE_DIR / "tests" / "data" / "unreachable_goal.csv"


def run_simulator(output: Path, map_file: Path, start, goal, *extra: str) -> int:
    command = [str(SIMULATOR), "--map", str(map_file),
               "--start", *map(str, start), "--goal", *map(str, goal),
               "--output", str(output), *extra]
    return subprocess.run(command, capture_output=True, check=False).returncode


def load_results(results: Path, map_file: Path):
    grid = vm.load_map(map_file)
    metrics = vm.load_metrics(results / "planning_metrics.csv")
    waypoints = vm.load_path(results / "path.csv")
    trajectory = vm.load_trajectory(results / "trajectory.csv")
    sim = vm.load_simulation_metrics(results / "simulation_metrics.csv")
    return grid, metrics, waypoints, trajectory, sim


# A straight 1 m trajectory: x from 0.25 to 1.25 m at 0.5 m/s, sampled every 0.5 s.
SIMPLE_TRAJECTORY = np.array([
    [0.0, 0.25, 0.25, 1],
    [0.5, 0.50, 0.25, 1],
    [1.0, 0.75, 0.25, 2],
    [1.5, 1.00, 0.25, 2],
    [2.0, 1.25, 0.25, 2],
])


class FrameTimingTests(unittest.TestCase):
    def test_frames_cover_the_whole_simulation(self):
        times = ar.frame_times(0.0, 34.0, fps=15, playback_speed=2.83)
        self.assertEqual(times[0], 0.0)
        self.assertEqual(times[-1], 34.0)
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertLessEqual(np.diff(times).max(), 2.83 / 15 + 1e-12)

    def test_real_time_playback_has_one_frame_per_fps_interval(self):
        times = ar.frame_times(0.0, 4.0, fps=15, playback_speed=1.0)
        self.assertEqual(len(times), 61)
        np.testing.assert_allclose(np.diff(times), 1 / 15)

    def test_zero_duration_gives_a_single_frame(self):
        np.testing.assert_array_equal(ar.frame_times(3.0, 3.0, 15, 1.0), [3.0])

    def test_default_playback_speed(self):
        self.assertAlmostEqual(ar.default_playback_speed(36.0), 3.0)
        self.assertEqual(ar.default_playback_speed(2.0), 1.0)  # never slower than real time


class InterpolationTests(unittest.TestCase):
    def test_position_matches_recorded_samples(self):
        for t, x, y, _ in SIMPLE_TRAJECTORY:
            self.assertEqual(ar.position_at(SIMPLE_TRAJECTORY, t), (x, y))

    def test_position_between_samples_is_linear(self):
        x, y = ar.position_at(SIMPLE_TRAJECTORY, 0.25)
        self.assertAlmostEqual(x, 0.375)
        self.assertAlmostEqual(y, 0.25)

    def test_position_is_clamped_to_the_trajectory(self):
        self.assertEqual(ar.position_at(SIMPLE_TRAJECTORY, 99.0), (1.25, 0.25))

    def test_traveled_path_ends_at_robot(self):
        traveled = ar.traveled_until(SIMPLE_TRAJECTORY, 1.2)
        np.testing.assert_allclose(traveled[:-1], SIMPLE_TRAJECTORY[:3, 1:3])
        np.testing.assert_allclose(traveled[-1], ar.position_at(SIMPLE_TRAJECTORY, 1.2))

    def test_target_waypoint(self):
        self.assertEqual(ar.target_waypoint_at(SIMPLE_TRAJECTORY, 0.0), 1)
        self.assertEqual(ar.target_waypoint_at(SIMPLE_TRAJECTORY, 0.9), 1)
        self.assertEqual(ar.target_waypoint_at(SIMPLE_TRAJECTORY, 1.0), 2)


class ResultValidationTests(unittest.TestCase):
    def test_rejects_non_increasing_timestamps(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory.csv"
            path.write_text("time,x,y,target_waypoint_index\n0.0,0.25,0.25,1\n"
                            "0.0,0.30,0.25,1\n")
            with self.assertRaises(vm.ResultFileError):
                vm.load_trajectory(path)

    def test_rejects_missing_columns(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trajectory.csv"
            path.write_text("time,x,y\n0.0,0.25,0.25\n")
            with self.assertRaises(vm.ResultFileError):
                vm.load_trajectory(path)


@unittest.skipUnless(SIMULATOR.is_file(), f"simulator not built at {SIMULATOR}")
class AnimationIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.results = Path(self.tmp.name)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_frames_follow_recorded_timestamps(self):
        self.assertEqual(run_simulator(self.results, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        grid, metrics, waypoints, trajectory, sim = load_results(self.results, OBSTACLES_MAP)
        fig, anim, frame_count = ar.build_animation(grid, metrics, waypoints, trajectory,
                                                    sim, fps=10, playback_speed=3.7)
        # 3.7x at 10 fps puts frames between the 0.1 s trajectory samples.
        times = np.concatenate([ar.frame_times(0.0, 34.0, 10, 3.7),
                                np.full(int(round(ar.END_HOLD_S * 10)), 34.0)])
        self.assertEqual(frame_count, len(times))
        robot, trail, time_text, status_text = anim._func(0)
        for frame in (0, 17, 40, frame_count - 1):
            robot, trail, time_text, status_text = anim._func(frame)
            x, y = ar.position_at(trajectory, times[frame])
            np.testing.assert_allclose(
                [robot.get_xdata()[0], robot.get_ydata()[0]],
                [vm.world_to_grid(x, metrics.resolution),
                 vm.world_to_grid(y, metrics.resolution)])
            self.assertIn(f"{times[frame]:5.1f} s", time_text.get_text())
        self.assertEqual(status_text.get_text(), "GOAL_REACHED")
        robot, trail, time_text, status_text = anim._func(17)
        self.assertTrue(status_text.get_text().startswith("NAVIGATING"))
        # Independent check from the motion model: the first leg (t <= 14 s) runs
        # along row 1 from x = 0.75 m at 0.5 m/s, so x(t) = 0.75 + 0.5 t, y = 0.75 m.
        t = times[17]
        self.assertTrue(0.0 < t < 14.0 and abs(t * 10 - round(t * 10)) > 0.1)
        expected_x_m = 0.75 + 0.5 * t
        np.testing.assert_allclose([robot.get_xdata()[0], robot.get_ydata()[0]],
                                   [expected_x_m / 0.5 - 0.5, 1.0], atol=1e-9)

    def test_gif_export(self):
        self.assertEqual(run_simulator(self.results, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        status = ar.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(self.results),
                          "--save-gif", "--fps", "5", "--playback-speed", "17"])
        self.assertEqual(status, 0)
        gif = self.results / "navigation.gif"
        with Image.open(gif) as image:
            self.assertEqual(image.format, "GIF")
            self.assertGreater(image.n_frames, 5)
            self.assertEqual(image.info.get("loop"), 0)

    def test_timeout_run_is_animated_with_its_status(self):
        self.assertEqual(run_simulator(self.results, OBSTACLES_MAP, (1, 1), (18, 18),
                                       "--max-time", "3"), 6)
        grid, metrics, waypoints, trajectory, sim = load_results(self.results, OBSTACLES_MAP)
        fig, anim, frame_count = ar.build_animation(grid, metrics, waypoints, trajectory,
                                                    sim, fps=5, playback_speed=1.0)
        *_, status_text = anim._func(frame_count - 1)
        self.assertEqual(status_text.get_text(), "TIMEOUT")

    def test_failed_plan_is_not_animated(self):
        self.assertEqual(run_simulator(self.results, UNREACHABLE_MAP, (0, 0), (4, 4)), 4)
        stale = self.results / "navigation.gif"
        stale.write_bytes(b"old animation")
        status = ar.main(["--map", str(UNREACHABLE_MAP), "--results-dir", str(self.results),
                          "--save-gif"])
        self.assertEqual(status, 0)
        self.assertFalse(stale.exists())
        self.assertTrue((self.results / "navigation_failed.png").is_file())

    def test_mp4_without_ffmpeg_reports_error_but_keeps_gif(self):
        self.assertEqual(run_simulator(self.results, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        with mock.patch.object(ar.animation.FFMpegWriter, "isAvailable", return_value=False):
            status = ar.main(["--map", str(OBSTACLES_MAP), "--results-dir",
                              str(self.results), "--save-gif", "--save-mp4",
                              "--fps", "5", "--playback-speed", "17"])
        self.assertEqual(status, 1)
        self.assertTrue((self.results / "navigation.gif").is_file())
        self.assertFalse((self.results / "navigation.mp4").exists())

    def test_trajectory_from_another_run_is_rejected(self):
        self.assertEqual(run_simulator(self.results, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        other = self.results / "other"
        self.assertEqual(run_simulator(other, OBSTACLES_MAP, (0, 0), (18, 18)), 0)
        (self.results / "trajectory.csv").write_text((other / "trajectory.csv").read_text())
        status = ar.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(self.results)])
        self.assertEqual(status, 1)


class RunDemoTests(unittest.TestCase):
    def test_simulator_command_is_an_argument_list(self):
        args = run_demo.parse_args(["--map", "maps/a b;rm -rf.csv", "--speed", "0.75"])
        command = run_demo.simulator_command(args)
        self.assertIn("maps/a b;rm -rf.csv", command)  # passed verbatim, one argument
        self.assertEqual(command[command.index("--speed") + 1], "0.75")
        self.assertNotIn("--dt", command)  # unset options keep the C++ defaults

    def test_animation_only_when_requested(self):
        self.assertIsNone(run_demo.animation_args(run_demo.parse_args([])))
        argv = run_demo.animation_args(run_demo.parse_args(["--animate", "--save-gif"]))
        self.assertIn("--show", argv)
        self.assertIn("--save-gif", argv)

    def test_missing_simulator_is_reported(self):
        status = run_demo.main(["--simulator", "/nonexistent/robot_simulator"])
        self.assertEqual(status, 127)


if __name__ == "__main__":
    unittest.main(verbosity=2)
