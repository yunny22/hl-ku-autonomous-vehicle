"""Bridge safe HL_KU actuator commands to the vehicle's existing NUCLEO UART."""

from __future__ import annotations

import math
import time

import rclpy
import serial
from rclpy.node import Node

from hl_ku_interfaces.msg import ActuatorCommand, VehicleFeedback

from .nucleo_serial import (
    SteeringAdcCalibration,
    drive_duty_to_command,
    drive_transaction,
    steering_transaction,
)


class NucleoSerialBridgeNode(Node):
    """Use ACK-checked position steering while retaining the current ADC map.

    The installed text protocol returns command acknowledgements, not measured
    telemetry.  Consequently steering feedback published here is the last
    acknowledged target and battery voltage is reported as unavailable (0 V).
    The NUCLEO remains responsible for its local ADC position loop/watchdog.
    """

    def __init__(self) -> None:
        super().__init__("nucleo_serial_bridge")
        # Hardware access is opt-in.  A deployment supplies its local serial
        # device and calibration values through a private parameter file.
        self.declare_parameter("enabled", False)
        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.20)
        self.declare_parameter("serial_timeout_sec", 0.025)
        self.declare_parameter("drive_command_scale", 100.0)
        self.declare_parameter("maximum_drive_command", 12)
        self.declare_parameter("allow_reverse", False)
        self.declare_parameter("maximum_steering_rad", 0.48)
        self.declare_parameter("steer_left_adc", 2)
        self.declare_parameter("steer_center_adc", 1)
        self.declare_parameter("steer_right_adc", 0)

        rate = float(self.get_parameter("send_rate_hz").value)
        timeout = float(self.get_parameter("command_timeout_sec").value)
        serial_timeout = float(self.get_parameter("serial_timeout_sec").value)
        if not math.isfinite(rate) or rate < 10.0:
            raise ValueError("send_rate_hz must be finite and at least 10 Hz")
        if not math.isfinite(timeout) or not 0.0 < timeout < 0.30:
            raise ValueError("command_timeout_sec must be below the 0.30 s firmware watchdog")
        if not math.isfinite(serial_timeout) or not 0.0 < serial_timeout < timeout:
            raise ValueError("serial_timeout_sec must be positive and below command timeout")

        self._steering = SteeringAdcCalibration(
            maximum_steering_rad=float(
                self.get_parameter("maximum_steering_rad").value
            ),
            left_adc=int(self.get_parameter("steer_left_adc").value),
            center_adc=int(self.get_parameter("steer_center_adc").value),
            right_adc=int(self.get_parameter("steer_right_adc").value),
        )
        self._steering.validate()
        self._drive_scale = float(self.get_parameter("drive_command_scale").value)
        self._maximum_drive_command = int(
            self.get_parameter("maximum_drive_command").value
        )
        drive_duty_to_command(0.0, self._drive_scale, self._maximum_drive_command)
        self._allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self._timeout = timeout
        self._command: ActuatorCommand | None = None
        self._command_time: float | None = None
        self._received_command = False
        self._fault_latched = 0
        self._last_acknowledged_steering_rad = 0.0
        self._last_warning_time = 0.0
        self._serial = None
        enabled = bool(self.get_parameter("enabled").value)
        port = str(self.get_parameter("port").value).strip()
        if enabled and port:
            self._serial = serial.Serial(
                port,
                int(self.get_parameter("baudrate").value),
                timeout=serial_timeout,
                write_timeout=min(0.05, timeout),
            )
            self._serial.reset_input_buffer()
        self._feedback_pub = self.create_publisher(
            VehicleFeedback, "/vehicle/feedback", 20
        )
        self.create_subscription(
            ActuatorCommand,
            "/vehicle/actuator_command_safe",
            self._on_command,
            20,
        )
        self._send_stop(repetitions=3)
        self.create_timer(1.0 / rate, self._exchange)
        if self._serial is None:
            self.get_logger().info(
                "NUCLEO UART is disabled; provide local device and calibration parameters to enable it"
            )
        else:
            self.get_logger().info(
                "NUCLEO UART ready; steering calibration loaded from deployment parameters; "
                f"route-test drive command limited to +/-{self._maximum_drive_command}"
            )

    def destroy_node(self):  # type: ignore[override]
        try:
            self._send_stop(repetitions=3)
            if self._serial is not None and self._serial.is_open:
                self._serial.close()
        finally:
            return super().destroy_node()

    def _warn(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time >= 1.0:
            self.get_logger().warning(text)
            self._last_warning_time = now

    def _on_command(self, message: ActuatorCommand) -> None:
        self._command = message
        self._command_time = time.monotonic()
        self._received_command = True

    def _fresh_command(self) -> bool:
        if self._command_time is None:
            return False
        age = time.monotonic() - self._command_time
        return 0.0 <= age <= self._timeout

    def _transact(self, command: str, expected: str) -> bool:
        self._serial.write((command + "\n").encode("ascii"))
        self._serial.flush()
        reply = self._serial.readline().decode("ascii", "replace").strip()
        if reply == expected:
            return True
        self._warn(f"unexpected NUCLEO reply {reply!r}; expected {expected!r}")
        return False

    def _send_stop(self, repetitions: int = 1) -> bool:
        if not getattr(self, "_serial", None) or not self._serial.is_open:
            return False
        command, expected = drive_transaction(0)
        success = True
        for _ in range(repetitions):
            try:
                success = self._transact(command, expected) and success
            except (OSError, serial.SerialException) as error:
                self._warn(f"NUCLEO serial stop failed: {error}")
                return False
            if repetitions > 1:
                time.sleep(0.02)
        return success

    def _latch_protocol_fault(self, reason: str) -> None:
        self._fault_latched |= VehicleFeedback.FAULT_PROTOCOL
        self._warn(reason + "; protocol fault latched until bridge restart")
        self._send_stop()

    def _publish_feedback(
        self,
        sequence: int,
        applied_duty: float,
        drive_enabled: bool,
        brake_active: bool,
    ) -> None:
        message = VehicleFeedback()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.steering_angle_rad = self._last_acknowledged_steering_rad
        message.steering_target_rad = self._last_acknowledged_steering_rad
        message.steering_output = math.nan
        message.applied_drive_duty = applied_duty
        message.battery_voltage = 0.0
        message.board_temperature_c = math.nan
        message.fault_flags = self._fault_latched
        message.last_command_sequence = sequence
        message.drive_enabled = drive_enabled and not self._fault_latched
        message.brake_active = brake_active or bool(self._fault_latched)
        self._feedback_pub.publish(message)

    def _exchange(self) -> None:
        if self._serial is None or not self._serial.is_open:
            sequence = self._command.sequence if self._command else 0
            self._publish_feedback(sequence, 0.0, False, True)
            return
        command = self._command if self._fresh_command() else None
        if command is None:
            if self._received_command:
                self._fault_latched |= VehicleFeedback.FAULT_WATCHDOG
                self._warn("safe actuator command timed out; watchdog fault latched")
            self._send_stop()
            sequence = self._command.sequence if self._command else 0
            self._publish_feedback(sequence, 0.0, False, True)
            return

        drive = float(command.drive_duty)
        steering = float(command.steering_angle_rad)
        sequence = int(command.sequence)
        values_valid = math.isfinite(drive) and math.isfinite(steering)
        active = command.enable and not command.brake
        braking = not command.enable and command.brake
        mode_valid = command.mode in (
            ActuatorCommand.MODE_AUTONOMOUS,
            ActuatorCommand.MODE_MANUAL,
        ) if active else command.mode in (
            ActuatorCommand.MODE_BRAKE,
            ActuatorCommand.MODE_FAULT,
            ActuatorCommand.MODE_MANUAL,
        )
        if (
            not values_valid
            or not (active or braking)
            or not mode_valid
            or (braking and abs(drive) > 1.0e-6)
            or (not self._allow_reverse and drive < 0.0)
            or abs(steering) > self._steering.maximum_steering_rad + 1.0e-6
        ):
            self._latch_protocol_fault("invalid safe actuator command rejected")
            self._publish_feedback(sequence, 0.0, False, True)
            return

        if self._fault_latched or braking:
            if not self._send_stop():
                self._latch_protocol_fault("NUCLEO stop acknowledgement failed")
            self._publish_feedback(sequence, 0.0, False, True)
            return

        drive_command = 0
        try:
            drive_command = drive_duty_to_command(
                drive, self._drive_scale, self._maximum_drive_command
            )
            target_adc = self._steering.target_adc(steering)
            drive_line, drive_ack = drive_transaction(drive_command)
            steer_line, steer_ack = steering_transaction(target_adc)
            if not self._transact(drive_line, drive_ack):
                self._latch_protocol_fault("NUCLEO drive acknowledgement failed")
            elif not self._transact(steer_line, steer_ack):
                self._latch_protocol_fault("NUCLEO steering acknowledgement failed")
            else:
                self._last_acknowledged_steering_rad = steering
        except (OSError, ValueError, serial.SerialException) as error:
            self._latch_protocol_fault(f"NUCLEO command failed: {error}")

        applied = 0.0 if self._fault_latched else drive_command / self._drive_scale
        self._publish_feedback(
            sequence,
            applied,
            not bool(self._fault_latched),
            bool(self._fault_latched),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NucleoSerialBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
