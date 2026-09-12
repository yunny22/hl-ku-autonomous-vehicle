"""Apply regulation-oriented mission decisions to the route command."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from hl_ku_interfaces.msg import DriveCommand, MissionStatus, PerceptionState

from .geometry import clamp
from .mission import MissionCoordinator, Observation, State


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("auto_arm", False)
        self.declare_parameter("default_speed_mps", 1.0)
        self.declare_parameter("hill_stop_s_m", 0.0)
        self.declare_parameter("hill_hold_sec", 3.2)
        self.declare_parameter("hill_speed_mps", 0.7)
        self.declare_parameter("dummy_trigger_m", 4.0)
        self.declare_parameter("dummy_hold_sec", 3.2)
        self.declare_parameter("traffic_stop_trigger_m", 2.0)
        self.declare_parameter("maximum_steering_rad", 0.48)
        self.declare_parameter("path_command_timeout_sec", 0.20)
        self._coordinator = MissionCoordinator(
            default_speed_mps=float(self.get_parameter("default_speed_mps").value),
            hill_stop_s_m=float(self.get_parameter("hill_stop_s_m").value),
            hill_hold_sec=float(self.get_parameter("hill_hold_sec").value),
            hill_speed_mps=float(self.get_parameter("hill_speed_mps").value),
            dummy_trigger_m=float(self.get_parameter("dummy_trigger_m").value),
            dummy_hold_sec=float(self.get_parameter("dummy_hold_sec").value),
            traffic_stop_trigger_m=float(self.get_parameter("traffic_stop_trigger_m").value),
        )
        if bool(self.get_parameter("auto_arm").value):
            self._coordinator.arm()
        self._path_command: DriveCommand | None = None
        self._path_command_time = None
        self._perception = PerceptionState()
        self._perception.stop_line_distance_m = math.inf
        self._perception.obstacle_distance_m = math.inf
        self._route_s = 0.0
        self._zone = "NORMAL"
        self._speed = 0.0
        self._command_pub = self.create_publisher(
            DriveCommand, "/mission/command", 10
        )
        self._status_pub = self.create_publisher(MissionStatus, "/mission/status", 10)
        self.create_subscription(
            DriveCommand, "/planning/path_command", self._on_path_command, 10
        )
        self.create_subscription(PerceptionState, "/perception/state", self._on_perception, 10)
        self.create_subscription(Float32, "/planning/route_s", self._on_s, 10)
        self.create_subscription(String, "/planning/route_zone", self._on_zone, 10)
        self.create_subscription(
            TwistStamped, "/localization/vehicle_velocity", self._on_velocity, 20
        )
        self.create_service(Trigger, "/mission/arm", self._arm)
        self.create_service(Trigger, "/mission/fault", self._fault)
        self.create_timer(0.05, self._update)

    def _on_path_command(self, message: DriveCommand) -> None:
        self._path_command = message
        self._path_command_time = time.monotonic()

    def _on_perception(self, message: PerceptionState) -> None:
        self._perception = message

    def _on_s(self, message: Float32) -> None:
        self._route_s = float(message.data)

    def _on_zone(self, message: String) -> None:
        self._zone = message.data

    def _on_velocity(self, message: TwistStamped) -> None:
        self._speed = float(message.twist.linear.x)

    def _arm(self, _request, response):
        self._coordinator.arm()
        response.success = self._coordinator.state != State.INIT
        response.message = "mission armed" if response.success else "arm rejected"
        return response

    def _fault(self, _request, response):
        self._coordinator.fault()
        response.success = True
        response.message = "mission fault latched; restart nodes to clear"
        return response

    def _update(self) -> None:
        now = self.get_clock().now()
        decision = self._coordinator.update(
            Observation(
                now_sec=time.monotonic(),
                zone=self._zone,
                route_s_m=self._route_s,
                speed_mps=self._speed,
                stop_line_detected=self._perception.stop_line_detected,
                stop_line_distance_m=self._perception.stop_line_distance_m,
                traffic_light=self._perception.traffic_light,
                obstacle_in_path=self._perception.obstacle_in_path,
                obstacle_distance_m=self._perception.obstacle_distance_m,
                allowed_lane=self._perception.allowed_lane,
            )
        )
        command = DriveCommand()
        command.header.stamp = now.to_msg()
        command.header.frame_id = "base_link"
        path_age = (
            time.monotonic() - self._path_command_time
            if self._path_command_time is not None
            else math.inf
        )
        path_fresh = 0.0 <= path_age <= float(
            self.get_parameter("path_command_timeout_sec").value
        )
        path_valid = self._path_command is not None and all(
            math.isfinite(value)
            for value in (
                self._path_command.speed_mps,
                self._path_command.steering_angle_rad,
            )
        )
        if path_valid and path_fresh:
            requested = float(self._path_command.speed_mps)
            speed_limit = max(0.0, decision.speed_limit_mps)
            command.speed_mps = math.copysign(min(abs(requested), speed_limit), requested)
            command.steering_angle_rad = self._path_command.steering_angle_rad
        if decision.state == State.S_OBSTACLE:
            command.steering_angle_rad += self._perception.avoidance_steering_offset_rad
        maximum = float(self.get_parameter("maximum_steering_rad").value)
        command.steering_angle_rad = clamp(command.steering_angle_rad, -maximum, maximum)
        if decision.brake or not path_fresh or not path_valid:
            command.speed_mps = 0.0
        self._command_pub.publish(command)
        status = MissionStatus()
        status.header = command.header
        status.state = int(decision.state)
        status.state_name = decision.state.name
        status.zone = self._zone
        status.route_s_m = self._route_s
        status.speed_limit_mps = decision.speed_limit_mps
        status.lateral_offset_m = decision.lateral_offset_m
        status.brake_required = decision.brake or not path_fresh or not path_valid
        status.mission_complete = decision.state == State.FINISH
        status.detail = (
            decision.detail
            if path_fresh and path_valid
            else "path command stale or invalid"
        )
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
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
