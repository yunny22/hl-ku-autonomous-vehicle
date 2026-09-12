"""Collect only RTK-fixed samples and print a repeatable ENU datum candidate."""

from __future__ import annotations

import math
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from hl_ku_interfaces.msg import GnssStatus


class GnssSurveyNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss_survey")
        self.declare_parameter("duration_sec", 600.0)
        self.declare_parameter("minimum_samples", 1000)
        self.declare_parameter("maximum_hdop", 1.5)
        self._status: GnssStatus | None = None
        self._samples: list[tuple[float, float, float]] = []
        self._start = self.get_clock().now()
        self._finished = False
        self.create_subscription(GnssStatus, "/gnss/status", self._on_status, 20)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 20)
        self.create_timer(5.0, self._progress)

    def _on_status(self, message: GnssStatus) -> None:
        self._status = message

    def _on_fix(self, message: NavSatFix) -> None:
        if self._finished or self._status is None:
            return
        if self._status.fix_type != GnssStatus.FIX_RTK_FIXED:
            return
        hdop = float(self._status.hdop)
        if math.isfinite(hdop) and hdop > float(self.get_parameter("maximum_hdop").value):
            return
        if all(math.isfinite(value) for value in (message.latitude, message.longitude, message.altitude)):
            self._samples.append((message.latitude, message.longitude, message.altitude))

    def _progress(self) -> None:
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        duration = float(self.get_parameter("duration_sec").value)
        if elapsed < duration:
            self.get_logger().info(
                f"datum survey {elapsed:.0f}/{duration:.0f}s, RTK-fixed samples={len(self._samples)}"
            )
            return
        minimum = int(self.get_parameter("minimum_samples").value)
        if len(self._samples) < minimum:
            self.get_logger().error(
                f"datum rejected: only {len(self._samples)} fixed samples; need {minimum}"
            )
            self._start = self.get_clock().now()
            self._samples.clear()
            return
        latitude = statistics.fmean(sample[0] for sample in self._samples)
        longitude = statistics.fmean(sample[1] for sample in self._samples)
        altitude = statistics.fmean(sample[2] for sample in self._samples)
        lat_spread = statistics.pstdev(sample[0] for sample in self._samples) * 111_320.0
        lon_spread = (
            statistics.pstdev(sample[1] for sample in self._samples)
            * 111_320.0
            * math.cos(math.radians(latitude))
        )
        self.get_logger().info(
            "DATUM ACCEPTED\n"
            f"datum_latitude_deg: {latitude:.10f}\n"
            f"datum_longitude_deg: {longitude:.10f}\n"
            f"datum_altitude_m: {altitude:.4f}\n"
            f"horizontal_1sigma_m: {math.hypot(lat_spread, lon_spread):.4f}\n"
            f"samples: {len(self._samples)}"
        )
        self._finished = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssSurveyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
