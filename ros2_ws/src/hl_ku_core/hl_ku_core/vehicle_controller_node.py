"""Outer speed controller for a traction motor without an encoder."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from hl_ku_interfaces.msg import ActuatorCommand, DriveCommand, MissionStatus, VehicleFeedback

from .control import SpeedPiController


class VehicleControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_controller")
        self.declare_parameter("kp", 0.10)
        self.declare_parameter("ki", 0.04)
        self.declare_parameter("integral_limit", 2.0)
        self.declare_parameter("maximum_duty", 0.75)
        self.declare_parameter("minimum_forward_duty", 0.0)
        self.declare_parameter("maximum_forward_speed_mps", 1.50)
        self.declare_parameter("maximum_reverse_speed_mps", 0.70)
        self.declare_parameter("deadband_speed_mps", 0.03)
        self.declare_parameter("nominal_voltage", 22.2)
        self.declare_parameter("forward_speeds_mps", [0.25, 0.6, 1.0, 1.5])
        self.declare_parameter("forward_duties", [0.18, 0.27, 0.38, 0.55])
        self.declare_parameter("reverse_speeds_mps", [0.2, 0.4, 0.7])
        self.declare_parameter("reverse_duties", [0.20, 0.29, 0.43])
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("velocity_timeout_sec", 0.40)
        self.declare_parameter("mission_timeout_sec", 0.30)
        self.declare_parameter("hill_hold_enabled", False)
        self.declare_parameter("hill_hold_duty", 0.0)
        self._controller = SpeedPiController(
            kp=float(self.get_parameter("kp").value),
            ki=float(self.get_parameter("ki").value),
            integral_limit=float(self.get_parameter("integral_limit").value),
            maximum_duty=float(self.get_parameter("maximum_duty").value),
            deadband_speed_mps=float(self.get_parameter("deadband_speed_mps").value),
            forward_speeds=list(self.get_parameter("forward_speeds_mps").value),
            forward_duties=list(self.get_parameter("forward_duties").value),
            reverse_speeds=list(self.get_parameter("reverse_speeds_mps").value),
            reverse_duties=list(self.get_parameter("reverse_duties").value),
            nominal_voltage=float(self.get_parameter("nominal_voltage").value),
            minimum_forward_duty=float(
                self.get_parameter("minimum_forward_duty").value
            ),
        )
        self._command: DriveCommand | None = None
        self._command_time = None
        self._velocity = 0.0
        self._velocity_time = None
        self._mission: MissionStatus | None = None
        self._mission_time = None
        self._battery_voltage = float(self.get_parameter("nominal_voltage").value)
        self._sequence = 0
        self._last_update = time.monotonic()
        self._publisher = self.create_publisher(
            ActuatorCommand, "/vehicle/actuator_command_raw", 10
        )
        self.create_subscription(
            DriveCommand, "/mission/command", self._on_command, 10
        )
        self.create_subscription(
            TwistStamped, "/localization/vehicle_velocity", self._on_velocity, 20
        )
        self.create_subscription(MissionStatus, "/mission/status", self._on_mission, 10)
        self.create_subscription(
            VehicleFeedback, "/vehicle/feedback", self._on_feedback, 20
        )
        self.create_timer(0.02, self._update)

    def _on_command(self, message: DriveCommand) -> None:
        self._command = message
        self._command_time = time.monotonic()

    def _on_velocity(self, message: TwistStamped) -> None:
        self._velocity = float(message.twist.linear.x)
        self._velocity_time = time.monotonic()

    def _on_mission(self, message: MissionStatus) -> None:
        self._mission = message
        self._mission_time = time.monotonic()

    def _on_feedback(self, message: VehicleFeedback) -> None:
        if message.battery_voltage > 1.0:
            self._battery_voltage = float(message.battery_voltage)

    def _is_fresh(self, stamp, timeout: float) -> bool:
        if stamp is None:
            return False
        age = time.monotonic() - stamp
        return 0.0 <= age <= timeout

    def _update(self) -> None:
        now = self.get_clock().now()
        monotonic_now = time.monotonic()
        dt = min(0.1, max(0.0, monotonic_now - self._last_update))
        self._last_update = monotonic_now
        command_fresh = self._is_fresh(
            self._command_time, float(self.get_parameter("command_timeout_sec").value)
        )
        velocity_fresh = self._is_fresh(
            self._velocity_time, float(self.get_parameter("velocity_timeout_sec").value)
        )
        mission_fresh = self._is_fresh(
            self._mission_time, float(self.get_parameter("mission_timeout_sec").value)
        )
        command_values_valid = self._command is not None and all(
            math.isfinite(value)
            for value in (
                self._command.speed_mps,
                self._command.steering_angle_rad,
                self._velocity,
            )
        )
        inputs_fresh = command_fresh and velocity_fresh and mission_fresh and command_values_valid
        hill_hold_active = bool(self.get_parameter("hill_hold_enabled").value) and (
            self._mission is not None
            and self._mission.state == MissionStatus.STATE_HILL_HOLD
        )
        brake = (
            not command_fresh
            or not velocity_fresh
            or not mission_fresh
            or not command_values_valid
            or self._mission.brake_required
        )
        if inputs_fresh and hill_hold_active:
            brake = False
        target_speed = float(self._command.speed_mps) if command_values_valid else 0.0
        target_speed = max(
            -float(self.get_parameter("maximum_reverse_speed_mps").value),
            min(
                float(self.get_parameter("maximum_forward_speed_mps").value),
                target_speed,
            ),
        )
        duty = 0.0
        if hill_hold_active and not brake:
            self._controller.reset()
            duty = float(self.get_parameter("hill_hold_duty").value)
        elif not brake:
            duty = self._controller.update(
                target_speed,
                self._velocity,
                self._battery_voltage,
                dt,
            )
        else:
            self._controller.reset()
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        output = ActuatorCommand()
        output.header.stamp = now.to_msg()
        output.header.frame_id = "base_link"
        output.drive_duty = duty
        output.steering_angle_rad = (
            float(self._command.steering_angle_rad) if self._command else 0.0
        )
        output.brake = brake
        output.enable = not brake
        output.mode = ActuatorCommand.MODE_BRAKE if brake else ActuatorCommand.MODE_AUTONOMOUS
        output.sequence = self._sequence
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
