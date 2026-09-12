"""GNSS pose route tracker with bounded camera-lane correction."""

from __future__ import annotations

import math
import time
from dataclasses import fields
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from std_msgs.msg import Float32, String

from hl_ku_interfaces.msg import DriveCommand, MissionStatus, PerceptionState

from .route import Route
from .route_tracking import RouteFollower, TrackingConfig


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PathTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_tracker")
        self.declare_parameter("route_file", "")
        self.declare_parameter("route_calibrated", False)
        defaults = TrackingConfig()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        config = TrackingConfig(**{f.name: float(self.get_parameter(f.name).value) for f in fields(defaults)})
        self.declare_parameter("pose_timeout_sec", 0.50)
        self.declare_parameter("velocity_timeout_sec", 1.5)
        self.declare_parameter("lane_offset_gain", 0.30)
        self.declare_parameter("lane_heading_gain", 0.45)
        self.declare_parameter("minimum_lane_confidence", 0.55)
        self.declare_parameter("enable_lane_correction", True)
        route_path = Path(str(self.get_parameter("route_file").value)).expanduser()
        self._route: Route | None = None
        if route_path.is_file():
            try:
                self._route = Route.load_csv(route_path)
            except (OSError, ValueError, KeyError) as error:
                self.get_logger().error(f"route load failed: {error}")
        else:
            self.get_logger().error(f"route file does not exist: {route_path}")
        self._pose: PoseWithCovarianceStamped | None = None
        self._perception: PerceptionState | None = None
        self._mission: MissionStatus | None = None
        self._follower = RouteFollower(self._route, config) if self._route else None
        self._pose_rx_sec = -math.inf
        self._velocity: float | None = None
        self._velocity_rx_sec = -math.inf
        self._last_update_sec = time.monotonic()
        for name in ("pose_timeout_sec", "velocity_timeout_sec"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self._command_pub = self.create_publisher(
            DriveCommand, "/planning/path_command", 10
        )
        self._s_pub = self.create_publisher(Float32, "/planning/route_s", 10)
        self._zone_pub = self.create_publisher(String, "/planning/route_zone", 10)
        self._path_pub = self.create_publisher(PathMessage, "/planning/reference_path", 1)
        self._status_pub = self.create_publisher(String, "/planning/tracking_status", 10)
        self._target_pub = self.create_publisher(PoseStamped, "/planning/tracking_target", 10)
        self._start_pose_pub = self.create_publisher(
            PoseStamped, "/planning/route_start_pose", 1
        )
        self._finish_pose_pub = self.create_publisher(
            PoseStamped, "/planning/route_finish_pose", 1
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/gnss_pose",
            self._on_pose,
            20,
        )
        self.create_subscription(PerceptionState, "/perception/state", self._on_perception, 10)
        self.create_subscription(TwistStamped, "/localization/vehicle_velocity", self._on_velocity, 20)
        self.create_subscription(MissionStatus, "/mission/status", self._on_mission, 10)
        self.create_timer(0.05, self._update)
        self.create_timer(1.0, self._publish_reference_path)

    def _on_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._pose = message
        self._pose_rx_sec = time.monotonic()

    def _on_velocity(self, message: TwistStamped) -> None:
        self._velocity = float(message.twist.linear.x)
        self._velocity_rx_sec = time.monotonic()

    def _on_perception(self, message: PerceptionState) -> None:
        self._perception = message

    def _on_mission(self, message: MissionStatus) -> None:
        self._mission = message

    def _publish_reference_path(self) -> None:
        if self._route is None:
            return
        path = PathMessage()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for i, point in enumerate(self._route.waypoints):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x_m
            pose.pose.position.y = point.y_m
            sample = self._route.sample_at_s(self._route.stations[i])
            yaw = sample.yaw_rad + (math.pi if point.direction < 0 else 0.0)
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose)
        self._path_pub.publish(path)
        self._start_pose_pub.publish(self._endpoint_pose(path.header, 0))
        self._finish_pose_pub.publish(
            self._endpoint_pose(path.header, len(self._route.waypoints) - 1)
        )

    def _endpoint_pose(self, header, index: int) -> PoseStamped:
        """Build a vehicle-heading arrow for an endpoint of the route."""
        assert self._route is not None
        points = self._route.waypoints
        point = points[index]
        if point.yaw_rad is not None:
            yaw = point.yaw_rad
        elif len(points) < 2:
            yaw = 0.0
        elif index == 0:
            neighbor = points[1]
            yaw = math.atan2(neighbor.y_m - point.y_m, neighbor.x_m - point.x_m)
        else:
            neighbor = points[index - 1]
            yaw = math.atan2(point.y_m - neighbor.y_m, point.x_m - neighbor.x_m)
        if point.direction < 0:
            yaw += math.pi
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = point.x_m
        pose.pose.position.y = point.y_m
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _update(self) -> None:
        now = time.monotonic()
        dt = max(1.0e-6, now - self._last_update_sec)
        self._last_update_sec = now
        if self._follower is None:
            command = DriveCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = "base_link"
            self._command_pub.publish(command)
            message = String()
            message.data = "route_unavailable"
            self._status_pub.publish(message)
            return
        correction = 0.0
        perception = self._perception
        if (
            bool(self.get_parameter("enable_lane_correction").value)
            and perception
            and perception.lane_confidence >= float(
                self.get_parameter("minimum_lane_confidence").value
            )
        ):
            correction = -float(self.get_parameter("lane_offset_gain").value) * perception.lane_offset_m
            correction -= float(self.get_parameter("lane_heading_gain").value) * perception.lane_heading_error_rad
        if self._pose is None or now - self._pose_rx_sec > float(self.get_parameter("pose_timeout_sec").value):
            result = self._follower.stopped("pose_stale")
        else:
            position = self._pose.pose.pose.position
            q = self._pose.pose.pose.orientation
            quaternion = (q.x, q.y, q.z, q.w)
            norm = math.sqrt(sum(v * v for v in quaternion))
            if not all(math.isfinite(v) for v in quaternion) or norm < 1.0e-6:
                result = self._follower.stopped("invalid_orientation")
            else:
                yaw = quaternion_yaw(*(v / norm for v in quaternion))
                velocity = self._velocity if now - self._velocity_rx_sec <= float(self.get_parameter("velocity_timeout_sec").value) else None
                result = self._follower.update(position.x, position.y, yaw, dt, velocity,
                    self._mission.lateral_offset_m if self._mission else 0.0, correction)
        speed, steering = result.speed_mps, result.steering_rad
        if not bool(self.get_parameter("route_calibrated").value):
            speed = 0.0
        command = DriveCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        command.speed_mps = speed
        command.steering_angle_rad = steering
        self._command_pub.publish(command)
        s_message = Float32()
        s_message.data = float(result.progress_m)
        self._s_pub.publish(s_message)
        zone_message = String()
        zone_message.data = result.metadata.mission
        self._zone_pub.publish(zone_message)
        status_message = String()
        status_message.data = result.status
        self._status_pub.publish(status_message)
        target_pose = PoseStamped()
        target_pose.header.stamp = command.header.stamp
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = result.target.x_m
        target_pose.pose.position.y = result.target.y_m
        target_pose.pose.orientation.z = math.sin(0.5 * result.target.yaw_rad)
        target_pose.pose.orientation.w = math.cos(0.5 * result.target.yaw_rad)
        self._target_pub.publish(target_pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathTrackerNode()
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
