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
