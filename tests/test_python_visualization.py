#!/usr/bin/env python3
"""Tests for the Python visualization, animation and analysis modules.

Runs under CTest (see CMakeLists.txt), which sets:
    AMR_SOURCE_DIR   repository root
    AMR_SIMULATOR    path to the built robot_simulator executable
Can also be run directly from the repository root after building:
    python3 tests/test_python_visualization.py
"""

from __future__ import annotations

import csv
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

import analyze_results  # noqa: E402
import animate_robot as ar  # noqa: E402
import plot_heading_control  # noqa: E402
import run_benchmark  # noqa: E402
import run_demo  # noqa: E402
import visualize_map as vm  # noqa: E402

OBSTACLES_MAP = SOURCE_DIR / "maps" / "obstacles.csv"
UNREACHABLE_MAP = SOURCE_DIR / "tests" / "data" / "unreachable_goal.csv"
FINE_MAP = SOURCE_DIR / "maps" / "obstacles_fine.csv"
FINE_ARGS = ("--resolution", "0.25", "--robot-radius", "0.2")


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
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory,
                                        sim, fps=10, playback_speed=3.7)
        frame_count = robot_anim.frame_count
        # 3.7x at 10 fps puts frames between the 0.1 s trajectory samples.
        times = np.concatenate([ar.frame_times(0.0, 34.0, 10, 3.7),
                                np.full(int(round(ar.END_HOLD_S * 10)), 34.0)])
        self.assertEqual(frame_count, len(times))
        for frame in (0, 17, 40, frame_count - 1):
            robot, trail, time_text, status_text = robot_anim.update(frame)
            x, y = ar.position_at(trajectory, times[frame])
            np.testing.assert_allclose(
                [robot.get_xdata()[0], robot.get_ydata()[0]],
                [vm.world_to_grid(x, metrics.resolution),
                 vm.world_to_grid(y, metrics.resolution)])
            self.assertIn(f"{times[frame]:5.1f} s", time_text.get_text())
        self.assertEqual(status_text.get_text(), "GOAL_REACHED")
        robot, trail, time_text, status_text = robot_anim.update(17)
        self.assertTrue(status_text.get_text().startswith("NAVIGATING"))
        # Independent check from the motion model: the first leg (t <= 14 s) runs
        # along row 1 from x = 0.75 m at 0.5 m/s, so x(t) = 0.75 + 0.5 t, y = 0.75 m.
        t = times[17]
        self.assertTrue(0.0 < t < 14.0 and abs(t * 10 - round(t * 10)) > 0.1)
        expected_x_m = 0.75 + 0.5 * t
        np.testing.assert_allclose([robot.get_xdata()[0], robot.get_ydata()[0]],
                                   [expected_x_m / 0.5 - 0.5, 1.0], atol=1e-9)

    def test_blitted_frames_equal_full_renders(self):
        self.assertEqual(run_simulator(self.results, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        grid, metrics, waypoints, trajectory, sim = load_results(self.results, OBSTACLES_MAP)
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=4.0)
        canvas = robot_anim.figure.canvas
        checked = {0, 9, robot_anim.frame_count - 1}
        blitted = {frame: np.array(canvas.buffer_rgba())
                   for frame in ar.render_frames_blitted(robot_anim) if frame in checked}
        for frame in sorted(checked):
            # Full render of the same state. FuncAnimation(blit=True) marks its
            # artists as animated, which a normal draw skips, so clear that first.
            for artist in robot_anim.update(frame):
                artist.set_animated(False)
            canvas.draw()
            np.testing.assert_array_equal(blitted[frame], np.array(canvas.buffer_rgba()),
                                          err_msg=f"frame {frame}")

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
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory,
                                        sim, fps=5, playback_speed=1.0)
        *_, status_text = robot_anim.update(robot_anim.frame_count - 1)
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


@unittest.skipUnless(SIMULATOR.is_file(), f"simulator not built at {SIMULATOR}")
class FootprintVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.results = Path(self.tmp.name)
        self.assertEqual(run_simulator(self.results, FINE_MAP, (20, 2), (20, 37), *FINE_ARGS), 0)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_inflated_map_is_loaded_from_the_simulator(self):
        grid, metrics, *_ = load_results(self.results, FINE_MAP)
        self.assertEqual(metrics.robot_radius, 0.2)
        inflated = vm.load_inflated_map(self.results, grid, metrics)
        self.assertEqual(inflated.shape, grid.shape)
        self.assertTrue(np.all(inflated >= grid))      # obstacles stay obstacles
        self.assertEqual(int((inflated > grid).sum()), 372)  # as reported by the CLI

    def test_mismatched_inflated_map_is_rejected(self):
        grid, metrics, *_ = load_results(self.results, FINE_MAP)
        (self.results / "inflated_map.csv").write_text("0,0\n0,0\n")
        with self.assertRaises(vm.ResultFileError):
            vm.load_inflated_map(self.results, grid, metrics)

    def test_animated_robot_is_a_true_size_disc(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.results, FINE_MAP)
        inflated = vm.load_inflated_map(self.results, grid, metrics)
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=4.0, inflated=inflated)
        robot, *_ = robot_anim.update(7)
        self.assertAlmostEqual(robot.get_radius(), 0.2 / 0.25)  # radius in grid cells
        t = ar.frame_times(trajectory[0, 0], trajectory[-1, 0], 5, 4.0)[7]
        x, y = ar.position_at(trajectory, t)
        np.testing.assert_allclose(robot.center, (x / 0.25 - 0.5, y / 0.25 - 0.5))

    def test_blitted_disc_frames_equal_full_renders(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.results, FINE_MAP)
        inflated = vm.load_inflated_map(self.results, grid, metrics)
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=6.0, inflated=inflated)
        canvas = robot_anim.figure.canvas
        checked = {0, 11, robot_anim.frame_count - 1}
        blitted = {frame: np.array(canvas.buffer_rgba())
                   for frame in ar.render_frames_blitted(robot_anim) if frame in checked}
        for frame in sorted(checked):
            for artist in robot_anim.update(frame):
                artist.set_animated(False)
            canvas.draw()
            np.testing.assert_array_equal(blitted[frame], np.array(canvas.buffer_rgba()),
                                          err_msg=f"frame {frame}")

    def test_static_figures_and_gif_with_footprint(self):
        self.assertEqual(vm.main(["--map", str(FINE_MAP), "--results-dir", str(self.results)]), 0)
        self.assertEqual(ar.main(["--map", str(FINE_MAP), "--results-dir", str(self.results),
                                  "--save-gif", "--fps", "4", "--playback-speed", "12"]), 0)
        for name in ("map_visualization.png", "planned_path.png", "trajectory.png",
                     "navigation.gif"):
            self.assertTrue((self.results / name).is_file(), name)


@unittest.skipUnless(SIMULATOR.is_file(), f"simulator not built at {SIMULATOR}")
class ConnectivityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_octile_plan_is_labelled_and_drawn(self):
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18),
                                       "--connectivity", "8"), 0)
        grid, metrics, waypoints, *_ = load_results(self.dir, OBSTACLES_MAP)
        self.assertEqual(metrics.connectivity, 8)
        self.assertEqual(metrics.diagonal_steps, 8)
        fig = vm.plot_planned_path(grid, metrics, waypoints, self.dir / "p.png")
        self.assertIn("8-connected A* path: 26 steps (8 diagonal)", fig._suptitle.get_text())
        # Diagonal moves really appear in the drawn path.
        steps = np.abs(np.diff(waypoints, axis=0))
        self.assertEqual(int(np.sum((steps[:, 0] == 1) & (steps[:, 1] == 1))), 8)

    def test_four_connected_title_is_unchanged(self):
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        grid, metrics, waypoints, *_ = load_results(self.dir, OBSTACLES_MAP)
        fig = vm.plot_planned_path(grid, metrics, waypoints, self.dir / "p.png")
        self.assertTrue(fig._suptitle.get_text().startswith("A* path: 34 steps, 17.00 m"))

    def test_octile_benchmark_and_comparison(self):
        common = ["--sizes", "6", "12", "--densities", "0", "0.3", "--seeds", "3",
                  "--simulator", str(SIMULATOR), "--no-analyze"]
        self.assertEqual(run_benchmark.main(common + ["--connectivity", "4",
                                                      "--output-dir", str(self.dir / "b4")]), 0)
        self.assertEqual(run_benchmark.main(common + ["--connectivity", "8",
                                                      "--output-dir", str(self.dir / "b8")]), 0)
        with (self.dir / "b8" / "benchmark_runs.csv").open() as handle:
            rows = list(csv.DictReader(handle))
        self.assertTrue(all(r["connectivity"] == "8" for r in rows))
        self.assertTrue(all(r["optimal"] == "1" for r in rows))   # exact Dijkstra agrees
        status = analyze_results.main(["compare", "--four", str(self.dir / "b4" / "benchmark_runs.csv"),
                                       "--eight", str(self.dir / "b8" / "benchmark_runs.csv"),
                                       "--output-dir", str(self.dir / "cmp")])
        self.assertEqual(status, 0)
        for name in ("connectivity_comparison.csv", "connectivity_comparison.md",
                     "path_length_reduction_vs_grid_size.png"):
            self.assertTrue((self.dir / "cmp" / name).is_file(), name)
        # Obstacle-free corner to corner: 8-connectivity saves exactly 1 - sqrt(2)/2.
        summary = analyze_results.compare_connectivity(self.dir / "b4" / "benchmark_runs.csv",
                                                       self.dir / "b8" / "benchmark_runs.csv")
        self.assertAlmostEqual(max(r["path_reduction_max_pct"] for r in summary),
                               100 * (1 - np.sqrt(2) / 2), places=1)

    def test_comparison_rejects_different_maps(self):
        common = ["--densities", "0.2", "--seeds", "2", "--simulator", str(SIMULATOR),
                  "--no-analyze"]
        run_benchmark.main(common + ["--sizes", "6", "--output-dir", str(self.dir / "a")])
        run_benchmark.main(common + ["--sizes", "8", "--connectivity", "8",
                                     "--output-dir", str(self.dir / "b")])
        with self.assertRaises(analyze_results.AnalysisError):
            analyze_results.compare_connectivity(self.dir / "a" / "benchmark_runs.csv",
                                                 self.dir / "b" / "benchmark_runs.csv")


@unittest.skipUnless(SIMULATOR.is_file(), f"simulator not built at {SIMULATOR}")
class DiffDriveVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18),
                                       "--robot-model", "diff-drive"), 0)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_heading_columns_are_loaded(self):
        *_, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        self.assertTrue(vm.has_heading(trajectory))
        self.assertEqual(trajectory.shape[1], 7)
        self.assertEqual(sim.robot_model, "diff-drive")
        self.assertEqual(sim.turns, 3)
        self.assertAlmostEqual(sim.turn_time_s, 4.5)
        # Turn-in-place samples: no translation, non-zero angular velocity.
        turning = (trajectory[:, vm.TRAJ_V] == 0) & (trajectory[:, vm.TRAJ_OMEGA] != 0)
        self.assertEqual(int(turning.sum()), 45)            # 3 turns x 15 steps
        moved = np.hypot(np.diff(trajectory[:, vm.TRAJ_X]), np.diff(trajectory[:, vm.TRAJ_Y]))
        np.testing.assert_array_equal(moved[turning[1:]], 0.0)

    def test_point_trajectory_has_no_heading(self):
        point_dir = self.dir / "point"
        self.assertEqual(run_simulator(point_dir, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        *_, trajectory, sim = load_results(point_dir, OBSTACLES_MAP)
        self.assertFalse(vm.has_heading(trajectory))
        self.assertEqual(trajectory.shape[1], 4)
        self.assertEqual(sim.robot_model, "point")
        self.assertIsNone(sim.turns)

    def test_heading_interpolation_takes_the_short_way(self):
        # theta jumps from +179 deg to -179 deg: the robot turned 2 deg, not 358 deg.
        trajectory = np.array([[0.0, 1.0, 1.0, 1, np.radians(179), 0.0, 0.2],
                               [1.0, 1.0, 1.0, 1, np.radians(-179), 0.0, 0.2]])
        mid = ar.heading_at(trajectory, 0.5)
        self.assertAlmostEqual(np.degrees(mid) % 360, 180.0, places=6)
        self.assertTrue(ar.is_turning_at(trajectory, 0.5))
        self.assertFalse(ar.is_turning_at(trajectory, 1.0))   # after the last sample

    def test_animation_shows_heading_and_turning(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=10, playback_speed=1.0)
        times = ar.frame_times(trajectory[0, 0], trajectory[-1, 0], 10, 1.0)
        turn_frame = int(np.argmax([ar.is_turning_at(trajectory, t) for t in times]))
        drive_frame = 5   # t = 0.5 s, driving east along the first row
        robot, trail, heading, time_text, status_text = robot_anim.update(drive_frame)
        self.assertTrue(status_text.get_text().startswith("NAVIGATING"))
        xs, ys = heading.get_data()
        self.assertAlmostEqual(ys[1] - ys[0], 0.0)           # heading 0: points along +x
        self.assertGreater(xs[1] - xs[0], 0.0)
        robot, trail, heading, time_text, status_text = robot_anim.update(turn_frame)
        self.assertTrue(status_text.get_text().startswith("TURNING"))
        xs, ys = heading.get_data()
        expected = ar.heading_at(trajectory, times[turn_frame])
        self.assertAlmostEqual(np.arctan2(ys[1] - ys[0], xs[1] - xs[0]), expected, places=9)

    def test_blitted_heading_frames_equal_full_renders(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=4.0)
        canvas = robot_anim.figure.canvas
        checked = {0, 18, robot_anim.frame_count - 1}
        blitted = {frame: np.array(canvas.buffer_rgba())
                   for frame in ar.render_frames_blitted(robot_anim) if frame in checked}
        for frame in sorted(checked):
            for artist in robot_anim.update(frame):
                artist.set_animated(False)
            canvas.draw()
            np.testing.assert_array_equal(blitted[frame], np.array(canvas.buffer_rgba()),
                                          err_msg=f"frame {frame}")

    def test_position_figure_has_heading_panel(self):
        *_, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        fig = vm.plot_position_vs_time(trajectory, sim, self.dir / "pos.png")
        self.assertEqual(len(fig.axes), 3)
        self.assertIn("heading", fig.axes[2].get_ylabel())
        point_dir = self.dir / "point"
        run_simulator(point_dir, OBSTACLES_MAP, (1, 1), (18, 18))
        *_, point_traj, point_sim = load_results(point_dir, OBSTACLES_MAP)
        self.assertEqual(len(vm.plot_position_vs_time(point_traj, point_sim,
                                                      self.dir / "p2.png").axes), 2)


@unittest.skipUnless(SIMULATOR.is_file(), f"simulator not built at {SIMULATOR}")
class LidarVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18),
                                       "--robot-model", "diff-drive", "--lidar-beams", "24"), 0)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_scans_are_loaded_and_match_the_trajectory(self):
        *_, trajectory, _ = load_results(self.dir, OBSTACLES_MAP)
        scans = vm.load_lidar(self.dir / "lidar_scans.csv")
        self.assertEqual(scans.ranges.shape, (len(trajectory), 24))
        np.testing.assert_allclose(scans.times, trajectory[:, vm.TRAJ_TIME])
        vm.check_lidar_matches_trajectory(scans, trajectory)
        self.assertTrue(np.all(scans.ranges[~scans.hits] == 5.0))   # no return: max range
        self.assertTrue(np.all(scans.ranges[scans.hits] <= 5.0))
        # Beams are heading-relative: beam 12 (offset 0) points along the heading.
        np.testing.assert_allclose(scans.angles[:, 12],
                                   trajectory[:, vm.TRAJ_THETA], atol=1e-8)
        with self.assertRaises(vm.ResultFileError):
            vm.check_lidar_matches_trajectory(scans, trajectory[:-1])

    def test_malformed_scans_are_rejected(self):
        path = self.dir / "lidar_scans.csv"
        lines = path.read_text().splitlines()
        path.write_text("\n".join(lines[:1] + lines[2:]) + "\n")   # drop beam 0 of scan 0
        with self.assertRaises(vm.ResultFileError):
            vm.load_lidar(path)
        self.assertIsNone(vm.load_lidar(self.dir / "missing.csv"))

    def test_ray_segments(self):
        grid, metrics, *_ , trajectory, _ = load_results(self.dir, OBSTACLES_MAP)
        scans = vm.load_lidar(self.dir / "lidar_scans.csv")
        segments, hits = vm.lidar_segments(scans, trajectory, 7, metrics.resolution)
        self.assertEqual(segments.shape, (24, 2, 2))
        start = vm.world_to_grid(trajectory[7, [vm.TRAJ_X, vm.TRAJ_Y]], metrics.resolution)
        np.testing.assert_allclose(segments[:, 0, :], np.tile(start, (24, 1)))
        lengths = np.hypot(*(segments[:, 1, :] - segments[:, 0, :]).T) * metrics.resolution
        np.testing.assert_allclose(lengths, scans.ranges[7], atol=1e-9)
        self.assertEqual(len(hits), int(scans.hits[7].sum()))

    def test_animation_draws_the_latest_scan(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        scans = vm.load_lidar(self.dir / "lidar_scans.csv")
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=3.0, lidar=scans)
        artists = robot_anim.update(9)
        rays, hits = artists[3], artists[4]          # after robot, trail, heading line
        self.assertEqual(len(rays.get_segments()), 24)
        # The latest scan at the frame's time (about 9 * 0.6 = 5.4 s, computed exactly
        # as the animation does).
        t = ar.frame_times(trajectory[0, vm.TRAJ_TIME], trajectory[-1, vm.TRAJ_TIME], 5, 3.0)[9]
        index = int(np.searchsorted(scans.times, t, side="right") - 1)
        segments, _ = vm.lidar_segments(scans, trajectory, index, metrics.resolution)
        np.testing.assert_allclose(rays.get_segments(), segments)
        self.assertEqual(len(hits.get_xdata()), int(scans.hits[index].sum()))
        canvas = robot_anim.figure.canvas
        checked = {0, 9, robot_anim.frame_count - 1}
        blitted = {f: np.array(canvas.buffer_rgba())
                   for f in ar.render_frames_blitted(robot_anim) if f in checked}
        for frame in sorted(checked):
            for artist in robot_anim.update(frame):
                artist.set_animated(False)
            canvas.draw()
            np.testing.assert_array_equal(blitted[frame], np.array(canvas.buffer_rgba()),
                                          err_msg=f"frame {frame}")

    def test_scan_figure(self):
        self.assertEqual(vm.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(self.dir)]), 0)
        self.assertTrue((self.dir / "lidar_scan.png").is_file())
        point_dir = self.dir / "no_lidar"
        run_simulator(point_dir, OBSTACLES_MAP, (1, 1), (18, 18))
        vm.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(point_dir)])
        self.assertFalse((point_dir / "lidar_scan.png").exists())


class LocalizationVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18),
                                       "--robot-model", "diff-drive", "--lidar-beams", "24",
                                       "--ekf"), 0)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_localization_is_loaded_and_matches_the_trajectory(self):
        *_, trajectory, _ = load_results(self.dir, OBSTACLES_MAP)
        loc = vm.load_localization(self.dir / "localization.csv")
        self.assertEqual(loc.truth.shape, (len(trajectory), 3))
        vm.check_localization_matches_trajectory(loc, trajectory)
        np.testing.assert_allclose(loc.truth[:, 2], trajectory[:, vm.TRAJ_THETA], atol=1e-8)
        # Covariances are symmetric and positive definite.
        np.testing.assert_allclose(loc.covariance, np.swapaxes(loc.covariance, 1, 2))
        self.assertTrue(np.all(np.linalg.eigvalsh(loc.covariance) > 0))
        # The LiDAR keeps the EKF far closer to the truth than dead reckoning.
        est = np.hypot(*loc.errors(loc.estimate)[:, :2].T)
        odom = np.hypot(*loc.errors(loc.odometry)[:, :2].T)
        self.assertLess(est.max(), 0.05)
        self.assertGreater(odom.max(), 10 * est.max())
        self.assertGreater(loc.beams_used.sum(), 0)
        with self.assertRaises(vm.ResultFileError):
            vm.check_localization_matches_trajectory(loc, trajectory[:-1])
        self.assertIsNone(vm.load_localization(self.dir / "missing.csv"))

    def test_heading_errors_are_wrapped(self):
        loc = vm.load_localization(self.dir / "localization.csv")
        loc.estimate[0, 2] = loc.truth[0, 2] + 2 * np.pi - 0.01
        self.assertAlmostEqual(loc.errors(loc.estimate)[0, 2], 0.01, places=9)

    def test_covariance_ellipse(self):
        # Variances 0.04 and 0.01 m^2 along rotated axes: 3-sigma semi-axes 0.6 and 0.3 m,
        # i.e. 1.2 and 0.6 cells at 0.5 m per cell.
        angle = np.radians(30)
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        covariance = rotation @ np.diag([0.04, 0.01]) @ rotation.T
        ellipse = vm.covariance_ellipse((2.0, 3.0), covariance, 0.5)
        self.assertAlmostEqual(ellipse.get_width(), 2.4)
        self.assertAlmostEqual(ellipse.get_height(), 1.2)
        self.assertAlmostEqual(ellipse.get_angle() % 180, 30.0)

    def test_figures(self):
        self.assertEqual(vm.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(self.dir)]), 0)
        self.assertTrue((self.dir / "localization.png").is_file())
        self.assertTrue((self.dir / "localization_errors.png").is_file())
        # Without --ekf there is no localization.csv (a stale one is removed) and no figure.
        plain = self.dir / "plain"
        run_simulator(plain, OBSTACLES_MAP, (1, 1), (18, 18), "--robot-model", "diff-drive")
        (plain / "localization.csv").write_text("stale")
        run_simulator(plain, OBSTACLES_MAP, (1, 1), (18, 18), "--robot-model", "diff-drive")
        self.assertFalse((plain / "localization.csv").exists())
        vm.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(plain)])
        self.assertFalse((plain / "localization.png").exists())

    def test_animation_draws_the_estimate(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        loc = vm.load_localization(self.dir / "localization.csv")
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=3.0, localization=loc)
        artists = robot_anim.update(9)
        odom_trail, odom_marker, ellipse, ekf_marker = artists[3:7]
        # The latest sample at the frame's time (about 5.4 s; computed exactly as the
        # animation does, since 9 * 0.6 is not exact in floating point).
        t = ar.frame_times(trajectory[0, vm.TRAJ_TIME], trajectory[-1, vm.TRAJ_TIME], 5, 3.0)[9]
        k = int(np.searchsorted(loc.times, t, side="right") - 1)
        res = metrics.resolution
        self.assertEqual(len(odom_trail.get_xdata()), k + 1)
        self.assertAlmostEqual(odom_marker.get_xdata()[0], vm.world_to_grid(loc.odometry[k, 0], res))
        self.assertAlmostEqual(ekf_marker.get_xdata()[0], vm.world_to_grid(loc.estimate[k, 0], res))
        self.assertAlmostEqual(ellipse.get_center()[1], vm.world_to_grid(loc.estimate[k, 1], res))
        canvas = robot_anim.figure.canvas
        checked = {0, 9, robot_anim.frame_count - 1}
        blitted = {f: np.array(canvas.buffer_rgba())
                   for f in ar.render_frames_blitted(robot_anim) if f in checked}
        for frame in sorted(checked):
            for artist in robot_anim.update(frame):
                artist.set_animated(False)
            canvas.draw()
            np.testing.assert_array_equal(blitted[frame], np.array(canvas.buffer_rgba()),
                                          err_msg=f"frame {frame}")


DYNAMIC_OBSTACLES = SOURCE_DIR / "maps" / "obstacles_dynamic.csv"


class DynamicObstacleVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18),
                                       "--dynamic-obstacles", str(DYNAMIC_OBSTACLES)), 0)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_outputs_load_and_match_the_trajectory(self):
        *_, trajectory, _ = load_results(self.dir, OBSTACLES_MAP)
        obstacles, replans = vm.load_dynamic(self.dir)
        self.assertEqual(obstacles.x.shape, (len(trajectory), 3))
        vm.check_dynamic_matches_trajectory(obstacles, trajectory)
        np.testing.assert_allclose(obstacles.radius, [0.3, 0.3, 0.3])
        # Obstacles move back and forth along their routes: obstacle 0 on column 10.
        np.testing.assert_allclose(obstacles.x[:, 0], 5.25)
        self.assertAlmostEqual(obstacles.y[:, 0].min(), 0.25, places=6)
        self.assertGreater(obstacles.visible.sum(), 0)
        self.assertGreaterEqual(replans.success.sum(), 1)
        for ok, start, path in zip(replans.success, replans.from_cells, replans.paths,
                                   strict=True):
            if ok:
                np.testing.assert_array_equal(path[0], start)
                np.testing.assert_array_equal(path[-1], (18, 18))
        # The robot never touched an obstacle, and it did wait for some.
        self.assertGreater(vm.obstacle_clearance(obstacles, trajectory, 0.0).min(), 0.0)
        self.assertTrue(vm.waiting_mask(trajectory).any())
        with self.assertRaises(vm.ResultFileError):
            vm.check_dynamic_matches_trajectory(obstacles, trajectory[:-1])

    def test_current_path(self):
        _, _, waypoints, *_ = load_results(self.dir, OBSTACLES_MAP)
        _, replans = vm.load_dynamic(self.dir)
        np.testing.assert_array_equal(replans.current_path(waypoints, 0.0), waypoints)
        first = int(np.flatnonzero(replans.success)[0])
        np.testing.assert_array_equal(
            replans.current_path(waypoints, replans.times[first]), replans.paths[first])

    def test_figure_and_stale_outputs(self):
        self.assertEqual(vm.main(["--map", str(OBSTACLES_MAP), "--results-dir", str(self.dir)]), 0)
        self.assertTrue((self.dir / "replanning.png").is_file())
        # Without --dynamic-obstacles the dynamic files are removed and there is no figure.
        run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18))
        for name in ("dynamic_obstacles.csv", "replans.csv", "replanned_paths.csv"):
            self.assertFalse((self.dir / name).exists(), name)
        self.assertIsNone(vm.load_dynamic(self.dir))

    def test_animation_draws_obstacles_and_the_current_plan(self):
        grid, metrics, waypoints, trajectory, sim = load_results(self.dir, OBSTACLES_MAP)
        dynamic = vm.load_dynamic(self.dir)
        obstacles, replans = dynamic
        robot_anim = ar.build_animation(grid, metrics, waypoints, trajectory, sim,
                                        fps=5, playback_speed=3.0, dynamic=dynamic)
        frame = 20
        t = ar.frame_times(trajectory[0, vm.TRAJ_TIME], trajectory[-1, vm.TRAJ_TIME], 5, 3.0)[frame]
        artists = robot_anim.update(frame)
        plan, discs = artists[2], artists[3:6]   # after robot and trail (point robot)
        np.testing.assert_array_equal(plan.get_xdata(), replans.current_path(waypoints, t)[:, 0])
        k = int(np.searchsorted(obstacles.times, t, side="right") - 1)
        self.assertAlmostEqual(discs[0].get_center()[1],
                               vm.world_to_grid(np.interp(t, obstacles.times, obstacles.y[:, 0]),
                                                metrics.resolution))
        self.assertEqual(discs[0].get_alpha(), 0.9 if obstacles.visible[k, 0] else 0.35)
        canvas = robot_anim.figure.canvas
        checked = {0, frame, robot_anim.frame_count - 1}
        blitted = {f: np.array(canvas.buffer_rgba())
                   for f in ar.render_frames_blitted(robot_anim) if f in checked}
        for f in sorted(checked):
            for artist in robot_anim.update(f):
                artist.set_animated(False)
            canvas.draw()
            np.testing.assert_array_equal(blitted[f], np.array(canvas.buffer_rgba()),
                                          err_msg=f"frame {f}")


GHOST_DATA = SOURCE_DIR / "docs" / "ghost"


class GhostExperimentPlotTests(unittest.TestCase):
    """plot_ghost_experiment.py on the committed experiment data."""

    def test_figures(self):
        import plot_ghost_experiment as pg
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            self.assertEqual(pg.main(["--input", str(GHOST_DATA), "--output-dir", str(out)]), 0)
            for name in ("ghost_example.png", "ghost_recovery.png", "ghost_scaling.png"):
                self.assertTrue((out / name).is_file(), name)
        plt.close("all")

    def test_recovery_summary(self):
        import plot_ghost_experiment as pg
        summary = pg.summarize_recovery(pg.read_rows(GHOST_DATA / "ghost_recovery_thorough.csv"))
        for strategy in pg.STRATEGIES:
            counts = summary[strategy]["counts"]
            self.assertEqual(sum(counts.values()), summary[strategy]["total"])
        # The documented headline: the most likely explanation recovered every case.
        self.assertEqual(summary["most_likely"]["counts"]["RECOVERED"],
                         summary["most_likely"]["total"])

    def test_missing_input_is_reported(self):
        import plot_ghost_experiment as pg
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(pg.main(["--input", tmp, "--output-dir", tmp]), 1)


ALIGNMENT_DATA = SOURCE_DIR / "docs" / "alignment"


class AlignmentExperimentPlotTests(unittest.TestCase):
    """plot_alignment_experiment.py on the committed experiment data."""

    def test_figures(self):
        import plot_alignment_experiment as pa
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            self.assertEqual(pa.main(["--input", str(ALIGNMENT_DATA), "--output-dir", str(out)]), 0)
            for name in ("alignment_example.png", "alignment_doorway.png", "alignment_classify.png"):
                self.assertTrue((out / name).is_file(), name)
        plt.close("all")

    def test_documented_doorway_result(self):
        import plot_alignment_experiment as pa
        rows = pa.read_rows(ALIGNMENT_DATA / "alignment_doorway.csv")
        def share(method, width):
            v = [int(r["passable"]) for r in rows
                 if r["method"] == method and abs(float(r["door_width"]) - width) < 1e-9]
            return sum(v) / len(v)
        # The point planner passes every alignment from 0.67 m; any overlap does not.
        for width in (0.67, 0.75, 0.85, 0.95):
            self.assertEqual(share("lidar_points", width), 1.0)
        self.assertLess(share("any_overlap", 0.85), 0.6)
        # Nobody passes a door narrower than the robot.
        for method in pa.METHODS:
            self.assertEqual(share(method, 0.55), 0.0)

    def test_missing_input_is_reported(self):
        import plot_alignment_experiment as pa
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(pa.main(["--input", tmp, "--output-dir", tmp]), 1)


HEADING_EXPERIMENT = SIMULATOR.parent / "heading_experiment"


@unittest.skipUnless(HEADING_EXPERIMENT.is_file(), f"not built: {HEADING_EXPERIMENT}")
class HeadingControlPlotTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        completed = subprocess.run([str(HEADING_EXPERIMENT), "--map", str(OBSTACLES_MAP),
                                    "--output", str(self.dir)], capture_output=True, check=False)
        self.assertEqual(completed.returncode, 0)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_metrics_and_figures(self):
        rows = plot_heading_control.load_metrics(self.dir / "heading_metrics.csv")
        self.assertEqual(len(rows), 8)       # 4 scenarios x {P, PID}
        by_key = {(r["scenario"], r["controller"]): r for r in rows}
        # Theory: P leaves -d/Kp under a disturbance and w/Kp on a ramp; PID ~ 0.
        self.assertAlmostEqual(float(by_key[("step_disturbance", "P")]["steady_state_error_rad"]),
                               -0.3 / 4, places=3)
        self.assertAlmostEqual(float(by_key[("ramp", "P")]["steady_state_error_rad"]),
                               0.5 / 4, places=3)
        self.assertLess(abs(float(by_key[("ramp", "PID")]["steady_state_error_rad"])), 0.002)
        self.assertEqual(by_key[("step_disturbance", "P")]["settling_time_s"], "")  # never
        self.assertEqual(plot_heading_control.main(["--input", str(self.dir)]), 0)
        for name in ("heading_step.png", "heading_step_disturbance.png", "heading_ramp.png",
                     "heading_path_profile.png", "heading_metrics.png"):
            self.assertTrue((self.dir / name).is_file(), name)

    def test_run_files_are_consistent(self):
        run = plot_heading_control.load_run(self.dir / "heading_step_p.csv")
        self.assertEqual(run["time"][0], 0.0)
        np.testing.assert_allclose(np.diff(run["time"]), 0.01, atol=1e-9)
        np.testing.assert_allclose(run["error"], np.angle(np.exp(1j * (run["reference"] - run["theta"]))),
                                   atol=1e-8)
        self.assertLessEqual(np.abs(run["command"]).max(), 1.5 + 1e-9)

    def test_missing_or_malformed_input_is_reported(self):
        self.assertEqual(plot_heading_control.main(["--input", str(self.dir / "nothing")]), 1)
        (self.dir / "heading_step_pid.csv").write_text("time,theta\n0,0\n")
        with self.assertRaises(plot_heading_control.HeadingResultError):
            plot_heading_control.load_run(self.dir / "heading_step_pid.csv")


class AnalysisTests(unittest.TestCase):
    def test_wilson_interval(self):
        low, high = analyze_results.wilson_interval(120, 120)
        self.assertAlmostEqual(high, 1.0)
        self.assertAlmostEqual(low, 0.969, places=3)
        low, high = analyze_results.wilson_interval(10, 20)
        self.assertLess(low, 0.5)
        self.assertGreater(high, 0.5)
        self.assertAlmostEqual(low + high, 1.0)  # symmetric around 0.5

    def test_summary_keeps_failed_runs(self):
        data = {
            "grid_size": np.array([10, 10, 10, 10]),
            "density": np.array([0.3, 0.3, 0.3, 0.3]),
            "planning_success": np.array([True, False, True, False]),
            "planning_time_ms": np.array([1.0, 2.0, 3.0, 4.0]),
            "expanded_nodes": np.array([10, 2, 30, 3]),
            "path_length_m": np.array([9.0, np.nan, 10.0, np.nan]),
            "optimal": np.array([1.0, 1.0, 1.0, 1.0]),
            "navigation_success": np.array([True, False, True, False]),
            "travel_time_s": np.array([18.0, np.nan, 20.0, np.nan]),
            "collision_count": np.array([0, 0, 0, 0]),
        }
        (row,) = analyze_results.summarize(data)
        self.assertEqual(row["runs"], 4)
        self.assertEqual(row["planning_successes"], 2)
        self.assertEqual(row["navigation_success_rate"], 0.5)
        self.assertEqual(row["planning_time_median_ms"], 2.5)  # failed runs included
        self.assertEqual(row["planning_time_max_ms"], 4.0)
        self.assertEqual(row["path_length_median_m"], 9.5)     # successful plans only

    def test_benchmark_maps_are_reproducible(self):
        a = run_benchmark.make_map(30, 0.3, seed=4)
        b = run_benchmark.make_map(30, 0.3, seed=4)
        c = run_benchmark.make_map(30, 0.3, seed=5)
        np.testing.assert_array_equal(a, b)
        self.assertFalse(np.array_equal(a, c))
        self.assertEqual(a[0, 0], 0)      # start cleared
        self.assertEqual(a[29, 29], 0)    # goal cleared


@unittest.skipUnless(SIMULATOR.is_file(), f"simulator not built at {SIMULATOR}")
class AnalysisIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        plt.close("all")
        self.tmp.cleanup()

    def test_run_metrics_are_recomputed_from_raw_files(self):
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        metrics = analyze_results.analyze_run(self.dir)
        self.assertEqual(metrics["path_length_m"], 17.0)   # 34 steps of 0.5 m
        self.assertEqual(metrics["travel_time_s"], 34.0)   # 17 m at 0.5 m/s
        self.assertEqual(metrics["navigation_success"], 1)

    def test_inconsistent_results_are_detected(self):
        self.assertEqual(run_simulator(self.dir, OBSTACLES_MAP, (1, 1), (18, 18)), 0)
        metrics_file = self.dir / "simulation_metrics.csv"
        header, row = metrics_file.read_text().splitlines()
        fields = row.split(",")
        fields[4] = "99.000000"  # travel_time_s no longer matches trajectory.csv
        metrics_file.write_text(header + "\n" + ",".join(fields) + "\n")
        with self.assertRaises(analyze_results.AnalysisError):
            analyze_results.analyze_run(self.dir)

    def test_small_benchmark_end_to_end(self):
        status = run_benchmark.main(["--sizes", "6", "12", "--densities", "0", "0.3",
                                     "--seeds", "3", "--output-dir", str(self.dir),
                                     "--simulator", str(SIMULATOR)])
        self.assertEqual(status, 0)
        with (self.dir / "benchmark_runs.csv").open() as handle:
            self.assertEqual(sum(1 for _ in handle) - 1, 12)   # 2 x 2 x 3 runs
        for name in ("benchmark_summary.csv", "benchmark_summary.md",
                     "benchmark_metadata.json", "planning_time_vs_grid_size.png",
                     "path_length_vs_grid_size.png", "expanded_nodes_vs_grid_size.png",
                     "success_rate_vs_obstacle_density.png"):
            self.assertTrue((self.dir / name).is_file(), name)


class RunDemoTests(unittest.TestCase):
    def test_simulator_command_is_an_argument_list(self):
        args = run_demo.parse_args(["--map", "maps/a b;rm -rf.csv", "--speed", "0.75"])
        command = run_demo.simulator_command(args)
        self.assertIn("maps/a b;rm -rf.csv", command)  # passed verbatim, one argument
        self.assertEqual(command[command.index("--speed") + 1], "0.75")
        self.assertNotIn("--dt", command)  # unset options keep the C++ defaults
        self.assertNotIn("--robot-radius", command)
        radius_args = run_demo.parse_args(["--robot-radius", "0.3"])
        radius_command = run_demo.simulator_command(radius_args)
        self.assertEqual(radius_command[radius_command.index("--robot-radius") + 1], "0.3")
        self.assertNotIn("--connectivity", command)
        octile_command = run_demo.simulator_command(run_demo.parse_args(["--connectivity", "8"]))
        self.assertEqual(octile_command[octile_command.index("--connectivity") + 1], "8")
        # String options are passed as-is (not repr()-quoted), numbers as numbers.
        dd_command = run_demo.simulator_command(run_demo.parse_args(
            ["--robot-model", "diff-drive", "--initial-heading", "90", "--heading-gain", "4"]))
        self.assertEqual(dd_command[dd_command.index("--robot-model") + 1], "diff-drive")
        self.assertEqual(dd_command[dd_command.index("--initial-heading") + 1], "90.0")
        self.assertEqual(dd_command[dd_command.index("--heading-gain") + 1], "4.0")
        lidar_command = run_demo.simulator_command(run_demo.parse_args(
            ["--lidar-beams", "36", "--lidar-noise", "0.02"]))
        self.assertEqual(lidar_command[lidar_command.index("--lidar-beams") + 1], "36")
        self.assertEqual(lidar_command[lidar_command.index("--lidar-noise") + 1], "0.02")
        self.assertNotIn("--ekf", lidar_command)
        ekf_command = run_demo.simulator_command(run_demo.parse_args(
            ["--ekf", "--odom-noise-drift", "0.01", "--odom-seed", "4"]))
        self.assertIn("--ekf", ekf_command)
        self.assertEqual(ekf_command[ekf_command.index("--odom-noise-drift") + 1], "0.01")
        self.assertEqual(ekf_command[ekf_command.index("--odom-seed") + 1], "4")
        dynamic_command = run_demo.simulator_command(run_demo.parse_args(
            ["--dynamic-obstacles", "maps/obstacles_dynamic.csv", "--replan-patience", "3"]))
        self.assertEqual(dynamic_command[dynamic_command.index("--dynamic-obstacles") + 1],
                         "maps/obstacles_dynamic.csv")
        self.assertEqual(dynamic_command[dynamic_command.index("--replan-patience") + 1], "3.0")
        self.assertNotIn("--sensing-range", dynamic_command)

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
