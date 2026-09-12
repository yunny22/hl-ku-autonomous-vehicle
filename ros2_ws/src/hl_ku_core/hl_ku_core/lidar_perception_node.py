"""2D LiDAR forward-corridor obstacle detection and conservative gap steering."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from .geometry import clamp


class LidarPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_perception")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("vehicle_half_width_m", 0.35)
        self.declare_parameter("corridor_margin_m", 0.15)
        self.declare_parameter("lidar_to_front_bumper_m", 0.15)
        self.declare_parameter("detection_range_m", 8.0)
        self.declare_parameter("obstacle_trigger_m", 5.0)
        self.declare_parameter("minimum_cluster_points", 3)
        self.declare_parameter("gap_max_angle_deg", 55.0)
        self.declare_parameter("avoidance_gain", 0.65)
        self.declare_parameter("maximum_avoidance_steering_rad", 0.30)
        self._distance_pub = self.create_publisher(Float32, "/perception/obstacle_distance_m", 10)
        self._obstacle_pub = self.create_publisher(Bool, "/perception/obstacle_in_path", 10)
        self._avoid_pub = self.create_publisher(Float32, "/perception/avoidance_steering_rad", 10)
        self._last_invalid_scan_warning_sec = 0.0
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )

    @staticmethod
    def _message(message_type, value):
        message = message_type()
        message.data = value
        return message

    def _on_scan(self, scan: LaserScan) -> None:
        if (
            not scan.ranges
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_max <= scan.range_min
        ):
            now_sec = time.monotonic()
            if now_sec - self._last_invalid_scan_warning_sec >= 1.0:
                self.get_logger().warning("invalid/empty LaserScan ignored")
                self._last_invalid_scan_warning_sec = now_sec
            return
        half_width = float(self.get_parameter("vehicle_half_width_m").value)
        margin = float(self.get_parameter("corridor_margin_m").value)
        lidar_to_bumper = float(self.get_parameter("lidar_to_front_bumper_m").value)
        maximum_range = float(self.get_parameter("detection_range_m").value)
        gap_angle = math.radians(float(self.get_parameter("gap_max_angle_deg").value))
        candidates = []
        corridor_clusters: list[list[float]] = []
        current_cluster: list[float] = []

        def finish_cluster() -> None:
            if current_cluster:
                corridor_clusters.append(current_cluster.copy())
                current_cluster.clear()

        for index, distance in enumerate(scan.ranges):
            if (
                not math.isfinite(distance)
                or distance < scan.range_min
                or distance > scan.range_max
            ):
                finish_cluster()
                continue
            distance = min(distance, maximum_range)
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > gap_angle:
                finish_cluster()
                continue
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x > 0.0 and abs(y) <= half_width + margin:
                current_cluster.append(max(0.0, x - lidar_to_bumper))
            else:
                finish_cluster()
            clearance_score = distance - 1.5 * abs(angle)
            candidates.append((clearance_score, angle))
        finish_cluster()
        minimum_points = int(self.get_parameter("minimum_cluster_points").value)
        qualified_clusters = [
            cluster for cluster in corridor_clusters if len(cluster) >= minimum_points
        ]
        cluster_present = bool(qualified_clusters)
        distance = (
            min(min(cluster) for cluster in qualified_clusters)
            if cluster_present
            else math.inf
        )
        obstacle = cluster_present and distance <= float(
            self.get_parameter("obstacle_trigger_m").value
        )
        best_angle = max(candidates, default=(0.0, 0.0))[1]
        avoidance = clamp(
            float(self.get_parameter("avoidance_gain").value) * best_angle,
            -float(self.get_parameter("maximum_avoidance_steering_rad").value),
            float(self.get_parameter("maximum_avoidance_steering_rad").value),
        )
        self._distance_pub.publish(self._message(Float32, float(distance)))
        self._obstacle_pub.publish(self._message(Bool, bool(obstacle)))
        self._avoid_pub.publish(self._message(Float32, float(avoidance)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
