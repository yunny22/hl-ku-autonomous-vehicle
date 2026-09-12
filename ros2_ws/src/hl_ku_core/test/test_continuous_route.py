"""Run with pytest, or python -m unittest discover -s test -p test_continuous_route.py."""

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from hl_ku_core.route import Route, Waypoint
from hl_ku_core.route_prepare import antenna_path_to_base_link, main, save_route
from hl_ku_core.route_smoothing import smooth_route
from hl_ku_core.route_tracking import RouteFollower, TrackingConfig


def make_route(xy, speed=0.3, direction=1):
    points, station = [], 0.0
    for i, (x, y) in enumerate(xy):
        if i:
            station += math.hypot(x - xy[i - 1][0], y - xy[i - 1][1])
        last = i == len(xy) - 1
        points.append(Waypoint(i, station, x, y, 0.0 if last else speed,
                               "FINISH" if last else "NORMAL", direction))
    return Route(points)


class ContinuousRouteTests(unittest.TestCase):
    def test_sparse_route_uses_continuous_target(self):
        route = make_route([(0, 0), (5, 0), (10, 0)])
        follower = RouteFollower(route)
        for _ in range(10):
            result = follower.update(2.2, 0.2, 0.0, 0.05)
        self.assertAlmostEqual(result.progress_m, 2.2)
        self.assertAlmostEqual(result.target.x_m, 3.12)
        self.assertGreater(result.speed_mps, 0)

    def test_passed_waypoint_is_not_chased_and_jitter_does_not_reverse_progress(self):
        follower = RouteFollower(make_route([(0, 0), (1, 0), (3, 0)]))
        a = follower.update(1.2, 0.15, 0, 0.05)
        b = follower.update(1.18, 0.15, 0, 0.05)
        self.assertGreater(a.target.x_m, 1.2)
        self.assertGreaterEqual(b.progress_m, a.progress_m)

    def test_heading_disambiguates_close_opposite_segments(self):
        route = make_route([(0, 0), (4, 0), (4, 0.3), (0, 0.3)])
        match = route.project(2, 0.20, 0, 0, route.length_m)
        self.assertIsNotNone(match)
        self.assertAlmostEqual(match.sample.y_m, 0.0)
        reverse_heading = route.project(2, 0.10, math.pi, 0, route.length_m)
        self.assertAlmostEqual(reverse_heading.sample.y_m, 0.3)

    def test_progress_window_excludes_nearby_later_section(self):
        route = make_route([(0, 0), (4, 0), (4, 3), (0, 3), (0, 0.2), (4, 0.2)])
        follower = RouteFollower(route)
        follower.update(1.0, 0.05, 0, 0.05)
        result = follower.update(1.1, 0.18, 0, 0.05)
        self.assertLess(result.progress_m, 2)

    def test_pose_jump_does_not_advance_progress(self):
        follower = RouteFollower(make_route([(0, 0), (20, 0)]))
        follower.update(0, 0, 0, 0.05)
        result = follower.update(10, 0, 0, 0.05)
        self.assertEqual(result.status, "pose_jump")
        self.assertEqual(result.speed_mps, 0)
        self.assertEqual(result.progress_m, 0)

    def test_excessive_deviation_stops_but_small_offset_can_recover(self):
        follower = RouteFollower(make_route([(0, 0), (5, 0)]))
        for _ in range(10):
            result = follower.update(0.5, 0.2, 0, 0.05)
        self.assertGreater(result.speed_mps, 0)
        result = follower.update(0.6, 1.2, 0, 0.05)
        self.assertEqual(result.status, "outside_tracking_corridor")
        self.assertEqual(result.speed_mps, 0)

    def test_reverse_leg_matches_body_heading(self):
        follower = RouteFollower(make_route([(0, 0), (-3, 0)], direction=-1))
        result = follower.update(-0.5, 0.0, 0.0, 0.05)
        self.assertLess(result.speed_mps, 0)
        self.assertLess(result.target.x_m, -0.5)

    def test_final_point_proximity_at_start_of_loop_does_not_finish(self):
        route = make_route([(0, 0), (5, 0), (5, 5), (0, 5), (0, 0.1)])
        result = RouteFollower(route).update(0, 0, 0, 0.05)
        self.assertNotEqual(result.status, "finished")

    def test_finish_requires_end_progress_and_is_latched(self):
        follower = RouteFollower(make_route([(0, 0), (3, 0)]))
        result = follower.update(2.7, 0.1, 0, 0.05)
        self.assertEqual(result.status, "finished")
        self.assertEqual(result.metadata.mission, "FINISH")
        self.assertEqual(follower.update(2.6, 0.2, 0, 0.05).speed_mps, 0)
        self.assertEqual(follower.stopped("pose_stale").metadata.mission, "FINISH")

    def test_steering_rate_including_correction_is_bounded(self):
        follower = RouteFollower(make_route([(0, 0), (5, 0)]))
        previous = 0
        for correction in [0.4] * 10 + [-0.4] * 10:
            result = follower.update(0.5, 0, 0, 0.05, steering_correction_rad=correction)
            self.assertLessEqual(abs(result.steering_rad - previous), 0.6 * 0.05 + 1e-12)
            previous = result.steering_rad

    def test_invalid_pose_and_parameters_fail_closed(self):
        follower = RouteFollower(make_route([(0, 0), (3, 0)]))
        self.assertEqual(follower.update(math.nan, 0, 0, 0.05).speed_mps, 0)
        with self.assertRaises(ValueError):
            TrackingConfig(maximum_steering_rate_rad_s=0)
        with self.assertRaises(ValueError):
            make_route([(0, 0), (0, 0)])

    def test_smoothing_straight_route_and_csv_round_trip(self):
        route, report = smooth_route(make_route([(0, 0), (3, 0), (6, 0)]))
        self.assertLess(report.maximum_deviation_m, 1e-8)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "route.csv"
            save_route(route, path)
            loaded = Route.load_csv(path)
        self.assertEqual(loaded.waypoints[-1].mission, "FINISH")
        self.assertAlmostEqual(loaded.sample_at_s(2.37).x_m, 2.37)
        self.assertIsNotNone(loaded.waypoints[0].curvature_inv_m)

    def test_course04_fairing_meets_deviation_and_radius_limits(self):
        path = Path(__file__).resolve().parents[1] / "routes/example_route.csv"
        source = Route.load_csv(path)
        route, report = smooth_route(source)
        self.assertLessEqual(report.maximum_deviation_m, 0.20)
        self.assertLessEqual(report.maximum_curvature_inv_m, 1 / 1.12)
        self.assertLessEqual(max(np.diff(route.stations)), 0.1002)
        self.assertAlmostEqual(route.waypoints[0].x_m, source.waypoints[0].x_m)
        self.assertAlmostEqual(route.waypoints[-1].y_m, source.waypoints[-1].y_m)
        self.assertTrue(all(p.mission != "FINISH" for p in route.waypoints[:-1]))

    def test_impossible_tight_corner_is_rejected(self):
        with self.assertRaises(ValueError):
            smooth_route(make_route([(0, 0), (1, 0), (1, 1)]), maximum_deviation_m=0.01)

    def test_event_anchor_and_speed_are_preserved(self):
        route = Route([Waypoint(0, 0, 0, 0, .3, "NORMAL", 1),
                       Waypoint(1, 2, 2, 0, .2, "HILL", 1),
                       Waypoint(2, 4, 4, 0, 0, "FINISH", 1)])
        result, _ = smooth_route(route)
        hill = next(p for p in result.waypoints if p.mission == "HILL")
        self.assertAlmostEqual(hill.x_m, 2)
        self.assertEqual(hill.target_speed_mps, .2)

    def test_measured_vehicle_yaw_controls_antenna_correction(self):
        route = make_route([(0, 0), (1, 0), (1, 1)])
        result = antenna_path_to_base_link(route, .802, 0, [0, 0, math.pi / 2])
        self.assertAlmostEqual(result.waypoints[1].x_m, .198)
        self.assertAlmostEqual(result.waypoints[1].y_m, 0)

    def test_cli_rejects_double_lever_arm_and_preserves_source(self):
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder) / "source.csv", Path(folder) / "smooth.csv"
            save_route(make_route([(0, 0), (3, 0)]), source)
            original = source.read_bytes()
            with self.assertRaises(ValueError):
                main([str(source), str(output), "--base-to-antenna-x-m", ".802"])
            self.assertFalse(output.exists())
            self.assertEqual(source.read_bytes(), original)
            with self.assertRaises(ValueError):
                main([str(source), str(source)])

    def test_bicycle_simulation_recovers_offset_and_finishes_course04(self):
        source = Route.load_csv(Path(__file__).resolve().parents[1] / "routes/example_route.csv")
        route, _ = smooth_route(source)
        follower = RouteFollower(route)
        first = route.sample_at_s(0)
        yaw = first.yaw_rad + 0.08
        x, y = first.x_m - 0.2 * math.sin(first.yaw_rad), first.y_m + 0.2 * math.cos(first.yaw_rad)
        speed, actual_steering, last_command = 0.0, 0.0, 0.0
        dt, disturbed, max_error = 0.05, False, 0.0
        for _ in range(7000):
            result = follower.update(x, y, yaw, dt, speed)
            self.assertLessEqual(abs(result.steering_rad - last_command), .6 * dt + 1e-10)
            last_command = result.steering_rad
            self.assertIn(result.status, ("tracking", "steering_settling", "finished"))
            max_error = max(max_error, result.cross_track_error_m)
            if result.status == "finished":
                break
            speed = result.speed_mps
            # Illustrative actuator lag; installed command sign is negative=left.
            actual_steering += (result.steering_rad - actual_steering) * dt / 0.15
            yaw_rate = -speed * math.tan(actual_steering) / .58
            x += speed * math.cos(yaw + yaw_rate * dt / 2) * dt
            y += speed * math.sin(yaw + yaw_rate * dt / 2) * dt
            yaw += yaw_rate * dt
            if result.progress_m > 18 and not disturbed:
                x -= .15 * math.sin(yaw)
                y += .15 * math.cos(yaw)
                disturbed = True
        else:
            self.fail("course did not finish within simulated time limit")
        self.assertTrue(disturbed)
        self.assertLess(max_error, .40)


if __name__ == "__main__":
    unittest.main()
