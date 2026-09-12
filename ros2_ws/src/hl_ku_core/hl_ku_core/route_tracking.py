"""Stateful continuous route following, independent of ROS for simulation/tests."""

from __future__ import annotations

import math
from dataclasses import dataclass, fields

from .control import pure_pursuit_steering
from .geometry import clamp, world_to_body
from .route import PathSample, Route, Waypoint


@dataclass(frozen=True)
class TrackingConfig:
    wheelbase_m: float = 0.58
    maximum_steering_rad: float = 0.48
    lookahead_base_m: float = 0.80
    lookahead_speed_gain_sec: float = 0.40
    minimum_lookahead_m: float = 0.60
    maximum_lookahead_m: float = 1.80
    curvature_lookahead_gain_m: float = 0.60
    maximum_steering_rate_rad_s: float = 0.60
    steering_settle_threshold_rad: float = 0.20
    maximum_lateral_acceleration_mps2: float = 0.30
    maximum_tracking_error_m: float = 1.0
    maximum_heading_error_rad: float = 2.1
    heading_weight_m: float = 0.60
    projection_backtrack_m: float = 0.30
    projection_search_ahead_m: float = 3.0
    initial_search_distance_m: float = 5.0
    maximum_pose_jump_m: float = 2.0
    finish_tolerance_m: float = 0.50

    def __post_init__(self) -> None:
        if not all(math.isfinite(getattr(self, f.name)) and getattr(self, f.name) > 0 for f in fields(self)):
            raise ValueError("tracking parameters must be finite and positive")
        if self.minimum_lookahead_m > self.maximum_lookahead_m:
            raise ValueError("minimum lookahead exceeds maximum lookahead")
        if self.maximum_heading_error_rad > math.pi:
            raise ValueError("heading gate must not exceed pi")


@dataclass(frozen=True)
class TrackingResult:
    speed_mps: float
    steering_rad: float
    progress_m: float
    target: PathSample
    metadata: Waypoint
    cross_track_error_m: float
    status: str


class RouteFollower:
    def __init__(self, route: Route, config: TrackingConfig = TrackingConfig()) -> None:
        self.route = route
        self.config = config
        self.progress_m: float | None = None
        self.last_steering_rad = 0.0
        self._last_xy: tuple[float, float] | None = None
        self.finished = False

    def stopped(self, status: str, error_m: float = math.inf) -> TrackingResult:
        station = self.progress_m if self.progress_m is not None else 0.0
        sample = self.route.sample_at_s(station)
        metadata = self.route.waypoints[-1] if self.finished else self.route.waypoints[sample.index]
        return TrackingResult(0.0, self.last_steering_rad, station, sample, metadata, error_m, status)

    def update(
        self, x_m: float, y_m: float, yaw_rad: float, dt_sec: float,
        measured_speed_mps: float | None = None,
        lateral_offset_m: float = 0.0, steering_correction_rad: float = 0.0,
    ) -> TrackingResult:
        cfg, route = self.config, self.route
        if not all(math.isfinite(v) for v in (x_m, y_m, yaw_rad, dt_sec,
                                             lateral_offset_m, steering_correction_rad)) or dt_sec <= 0:
            return self.stopped("invalid_pose_or_time")
        if measured_speed_mps is not None and not math.isfinite(measured_speed_mps):
            return self.stopped("invalid_velocity")
        if self.finished:
            return TrackingResult(0.0, self.last_steering_rad, route.length_m,
                route.sample_at_s(route.length_m), route.waypoints[-1],
                math.hypot(x_m - route.waypoints[-1].x_m, y_m - route.waypoints[-1].y_m), "finished")
        dt = min(dt_sec, 0.10)  # no large steering jump after a stalled callback
        if self.progress_m is None:
            low, high = 0.0, cfg.initial_search_distance_m
        else:
            assert self._last_xy is not None
            movement = math.hypot(x_m - self._last_xy[0], y_m - self._last_xy[1])
            if movement > cfg.maximum_pose_jump_m:
                return self.stopped("pose_jump")
            # Metric, motion-bounded search avoids jumping to a nearby later lap.
            advance = min(cfg.projection_search_ahead_m, max(0.25, movement * 1.5 + 0.10))
            low = max(0.0, self.progress_m - cfg.projection_backtrack_m)
            high = self.progress_m + advance
        projection = route.project(x_m, y_m, yaw_rad, low, high,
                                   cfg.heading_weight_m, cfg.maximum_heading_error_rad)
        if projection is None:
            return self.stopped("heading_mismatch")
        station = max(self.progress_m or 0.0, projection.sample.s_m)
        reference = route.sample_at_s(station)
        distance = math.hypot(reference.x_m - x_m, reference.y_m - y_m)
        if distance > cfg.maximum_tracking_error_m + abs(lateral_offset_m):
            return self.stopped("outside_tracking_corridor", distance)
        self.progress_m, self._last_xy = station, (x_m, y_m)
        metadata = route.waypoints[reference.index]
        final = route.waypoints[-1]
        final_distance = math.hypot(final.x_m - x_m, final.y_m - y_m)
        # Proximity to the final XY alone is insufficient on loops/crossings.
        if (route.length_m - station <= cfg.finish_tolerance_m
                and final_distance <= cfg.finish_tolerance_m and final.mission == "FINISH"):
            self.finished = True
            self.progress_m = route.length_m
            return TrackingResult(0.0, self.last_steering_rad, route.length_m,
                                  route.sample_at_s(route.length_m), final, distance, "finished")
        if metadata.mission == "FINISH":
            metadata = route.metadata_waypoint(reference.index, x_m, y_m, cfg.finish_tolerance_m)
        speed = abs(measured_speed_mps) if measured_speed_mps is not None else metadata.target_speed_mps
        base_lookahead = cfg.lookahead_base_m + cfg.lookahead_speed_gain_sec * speed
        preview_length = min(route.length_m - station, cfg.maximum_lookahead_m)
        count = max(1, math.ceil(preview_length / 0.10))
        curvature = max(abs(route.sample_at_s(station + preview_length * i / count).curvature_inv_m)
                        for i in range(count + 1))
        lookahead = clamp(base_lookahead / (1.0 + cfg.curvature_lookahead_gain_m * curvature),
                          cfg.minimum_lookahead_m, cfg.maximum_lookahead_m)
        target = route.sample_at_s(station + lookahead)
        # Do not look through a direction change; each reverse leg needs its own heading.
        for i in range(reference.index + 1, len(route.waypoints)):
            if route.stations[i] > target.s_m:
                break
            if route.waypoints[i].direction != metadata.direction:
                target = route.sample_at_s(route.stations[i])
                break
        target_x = target.x_m - math.sin(target.yaw_rad) * lateral_offset_m
        target_y = target.y_m + math.cos(target.yaw_rad) * lateral_offset_m
        body_x, _ = world_to_body(target_x, target_y, x_m, y_m, yaw_rad)
        if body_x * metadata.direction <= 0.0:
            return self.stopped("target_behind_vehicle", distance)
        desired = pure_pursuit_steering(x_m, y_m, yaw_rad, target_x, target_y,
                                       cfg.wheelbase_m, cfg.maximum_steering_rad, metadata.direction)
        desired = clamp(desired + steering_correction_rad, -cfg.maximum_steering_rad, cfg.maximum_steering_rad)
        step = cfg.maximum_steering_rate_rad_s * dt
        steering = clamp(desired, self.last_steering_rad - step, self.last_steering_rad + step)
        self.last_steering_rad = steering
        required_curvature = max(curvature, abs(math.tan(desired) / cfg.wheelbase_m))
        speed_limit = math.sqrt(cfg.maximum_lateral_acceleration_mps2 / max(required_curvature, 1.0e-6))
        requested_speed = min(metadata.target_speed_mps, speed_limit)
        status = "tracking"
        if abs(desired - steering) > cfg.steering_settle_threshold_rad:
            requested_speed, status = 0.0, "steering_settling"
        return TrackingResult(requested_speed * metadata.direction, steering, station,
                              target, metadata, distance, status)
