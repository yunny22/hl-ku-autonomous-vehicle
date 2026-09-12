"""Pure controller math used by ROS nodes and unit tests."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Sequence

from .geometry import clamp, world_to_body


def pure_pursuit_steering(
    vehicle_x_m: float,
    vehicle_y_m: float,
    vehicle_yaw_rad: float,
    target_x_m: float,
    target_y_m: float,
    wheelbase_m: float,
    maximum_steering_rad: float,
    direction: int = 1,
) -> float:
    body_x, body_y = world_to_body(
        target_x_m,
        target_y_m,
        vehicle_x_m,
        vehicle_y_m,
        vehicle_yaw_rad,
    )
    if direction < 0:
        body_x, body_y = -body_x, -body_y
    distance_sq = body_x * body_x + body_y * body_y
    if distance_sq < 1.0e-6:
        return 0.0
    curvature = 2.0 * body_y / distance_sq
    # The installed T870/NUCLEO convention is negative=left, positive=right.
    # Pure-pursuit geometry produces positive curvature for a target on the
    # vehicle's left, so convert that geometric sign at this boundary.
    steering = -math.atan(wheelbase_m * curvature)
    if direction < 0:
        steering = -steering
    return clamp(steering, -maximum_steering_rad, maximum_steering_rad)


@dataclass
class SpeedPiController:
    kp: float
    ki: float
    integral_limit: float
    maximum_duty: float
    deadband_speed_mps: float
    forward_speeds: Sequence[float]
    forward_duties: Sequence[float]
    reverse_speeds: Sequence[float]
    reverse_duties: Sequence[float]
    nominal_voltage: float
    minimum_forward_duty: float = 0.0
    _integral: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    @staticmethod
    def _interpolate(speed: float, speeds: Sequence[float], duties: Sequence[float]) -> float:
        if len(speeds) != len(duties) or not speeds:
            return 0.0
        value = abs(speed)
        if value <= speeds[0]:
            return duties[0] * (value / max(speeds[0], 1.0e-6))
        if value >= speeds[-1]:
            return duties[-1]
        upper = bisect.bisect_right(speeds, value)
        lower = upper - 1
        ratio = (value - speeds[lower]) / (speeds[upper] - speeds[lower])
        return duties[lower] + ratio * (duties[upper] - duties[lower])

    def update(
        self,
        target_speed_mps: float,
        measured_speed_mps: float,
        battery_voltage: float,
        dt_sec: float,
        freeze_integrator: bool = False,
    ) -> float:
        if abs(target_speed_mps) <= self.deadband_speed_mps:
            self.reset()
            return 0.0
        forward = target_speed_mps > 0.0
        feedforward = self._interpolate(
            target_speed_mps,
            self.forward_speeds if forward else self.reverse_speeds,
            self.forward_duties if forward else self.reverse_duties,
        )
        sign = 1.0 if forward else -1.0
        if battery_voltage > 1.0:
            feedforward *= self.nominal_voltage / battery_voltage
        error = target_speed_mps - measured_speed_mps
        if not freeze_integrator and dt_sec > 0.0:
            self._integral = clamp(
                self._integral + error * dt_sec,
                -self.integral_limit,
                self.integral_limit,
            )
        duty = sign * abs(feedforward) + self.kp * error + self.ki * self._integral
        duty = clamp(duty, -self.maximum_duty, self.maximum_duty)
        if forward and duty > 0.0:
            duty = max(
                duty,
                clamp(self.minimum_forward_duty, 0.0, self.maximum_duty),
            )
        return duty
