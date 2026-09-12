"""Interactive RTK-fixed waypoint recorder: Enter captures, U undoes, Q exits."""

from __future__ import annotations

import csv
import math
import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from hl_ku_interfaces.msg import GnssStatus

from .waypoint_capture import (
    CapturedWaypoint,
    FixedPositionSample,
    average_samples,
    build_route_points,
    sample_quality_issue,
    waypoint_spacing_m,
)


FIX_NAMES = {
    0: "NONE",
    1: "SINGLE",
    2: "DGPS",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
}


class RtkWaypointRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("rtk_waypoint_recorder")
        self.declare_parameter("output_prefix", "")
        self.declare_parameter("allow_overwrite", False)
        self.declare_parameter("minimum_satellites", 15)
        self.declare_parameter("maximum_hdop", 1.5)
        self.declare_parameter("maximum_correction_age_sec", 2.0)
        self.declare_parameter("maximum_fix_age_sec", 0.5)
        self.declare_parameter("averaging_window_sec", 1.0)
        self.declare_parameter("minimum_samples", 3)
        self.declare_parameter("minimum_waypoint_spacing_m", 0.10)
        self.declare_parameter("target_speed_mps", 0.30)

        if not sys.stdin.isatty():
            raise RuntimeError(
                "rtk_waypoint_recorder needs an interactive terminal; run it with ros2 run"
            )
        prefix_text = str(self.get_parameter("output_prefix").value).strip()
        if not prefix_text:
            raise ValueError("set output_prefix, for example route_capture")
        prefix = Path(prefix_text).expanduser().resolve()
        self._wgs84_path = Path(str(prefix) + "_wgs84.csv")
        self._route_path = Path(str(prefix) + "_route.csv")
        self._datum_path = Path(str(prefix) + "_datum.yaml")
        outputs = (self._wgs84_path, self._route_path, self._datum_path)
        if not bool(self.get_parameter("allow_overwrite").value):
            existing = [str(path) for path in outputs if path.exists()]
            if existing:
                raise FileExistsError("refusing to overwrite: " + ", ".join(existing))
        prefix.parent.mkdir(parents=True, exist_ok=True)

        self._stdin_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._status: GnssStatus | None = None
        self._status_rx_sec: float | None = None
        self._latest_sample: FixedPositionSample | None = None
        self._samples: deque[FixedPositionSample] = deque(maxlen=500)
        self._waypoints: list[CapturedWaypoint] = []
        self._quit_requested = False
        self.create_subscription(GnssStatus, "/gnss/status", self._on_status, 20)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, 20)
        self.create_timer(0.05, self._update_keyboard)
        self.create_timer(1.0, self._print_status)
        self.get_logger().warning(
            "GNSS ONLY: hold antenna still, ENTER=capture RTK waypoint, "
            "U=undo, P=status, Q=save/quit"
        )
        self.get_logger().info(
            f"outputs: {self._wgs84_path}, {self._route_path}, {self._datum_path}"
        )

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def destroy_node(self):  # type: ignore[override]
        try:
            termios.tcsetattr(
                self._stdin_fd, termios.TCSADRAIN, self._terminal_settings
            )
        finally:
            return super().destroy_node()

    def _on_status(self, message: GnssStatus) -> None:
        self._status = message
        self._status_rx_sec = time.monotonic()

    def _on_fix(self, message: NavSatFix) -> None:
        status = self._status
        if status is None:
            return
        stamp_sec = float(message.header.stamp.sec) + float(
            message.header.stamp.nanosec
        ) * 1.0e-9
        sample = FixedPositionSample(
            received_monotonic_sec=time.monotonic(),
            stamp_sec=stamp_sec,
            latitude_deg=float(message.latitude),
            longitude_deg=float(message.longitude),
            altitude_m=float(message.altitude),
            fix_type=int(status.fix_type),
            position_valid=bool(status.position_valid),
            satellites=int(status.satellites),
            hdop=float(status.hdop),
            correction_age_sec=float(status.correction_age_sec),
            nmea_checksum_valid=bool(status.nmea_checksum_valid),
        )
        self._latest_sample = sample
        self._samples.append(sample)

    def _quality_issue(self, sample: FixedPositionSample) -> str | None:
        return sample_quality_issue(
            sample,
            int(self.get_parameter("minimum_satellites").value),
            float(self.get_parameter("maximum_hdop").value),
            float(self.get_parameter("maximum_correction_age_sec").value),
        )

    def _current_issue(self) -> str | None:
        sample = self._latest_sample
        if sample is None:
            return "no /gnss/fix received"
        age = time.monotonic() - sample.received_monotonic_sec
        maximum_age = float(self.get_parameter("maximum_fix_age_sec").value)
        if age > maximum_age:
            return f"GNSS fix is stale ({age:.2f}s > {maximum_age:.2f}s)"
        return self._quality_issue(sample)

    def _read_keys(self) -> None:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            self._handle_key(sys.stdin.read(1))

    def _handle_key(self, key: str) -> None:
        lower = key.lower()
        if key in ("\r", "\n"):
            self._capture()
        elif lower == "u" or key in ("\x7f", "\b"):
            self._undo()
        elif lower == "p":
            self._print_status()
        elif lower == "q" or key == "\x03":
            self._quit_requested = True

    def _update_keyboard(self) -> None:
        self._read_keys()

    def _capture(self) -> None:
        current_issue = self._current_issue()
        if current_issue is not None:
            self.get_logger().error(f"WAYPOINT REJECTED: {current_issue}")
            return
        now = time.monotonic()
        window = float(self.get_parameter("averaging_window_sec").value)
        candidates = [
            sample
            for sample in self._samples
            if now - sample.received_monotonic_sec <= window
            and self._quality_issue(sample) is None
        ]
        minimum_samples = int(self.get_parameter("minimum_samples").value)
        if len(candidates) < minimum_samples:
            self.get_logger().error(
                f"WAYPOINT REJECTED: only {len(candidates)} good samples in "
                f"{window:.1f}s; need {minimum_samples}. Hold still and retry."
            )
            return
        waypoint = average_samples(candidates)
        if self._waypoints:
            spacing = waypoint_spacing_m(self._waypoints[-1], waypoint)
            if spacing <= 1.0e-6:
                self.get_logger().error(
                    "WAYPOINT REJECTED: exact duplicate position; move the antenna "
                    "or vehicle before pressing Enter again"
                )
                return
            minimum_spacing = float(
                self.get_parameter("minimum_waypoint_spacing_m").value
            )
            if spacing < minimum_spacing:
                self.get_logger().error(
                    f"WAYPOINT REJECTED: spacing={spacing:.3f}m, "
                    f"need >= {minimum_spacing:.3f}m"
                )
                return
        self._waypoints.append(waypoint)
        self._write_outputs()
        self.get_logger().info(
            f"WAYPOINT {len(self._waypoints) - 1} SAVED: "
            f"lat={waypoint.latitude_deg:.10f}, lon={waypoint.longitude_deg:.10f}, "
            f"samples={waypoint.sample_count}, spread={waypoint.horizontal_spread_m:.3f}m"
        )

    def _undo(self) -> None:
        if not self._waypoints:
            self.get_logger().warning("nothing to undo")
            return
        removed_index = len(self._waypoints) - 1
        self._waypoints.pop()
        self._write_outputs()
        self.get_logger().warning(f"waypoint {removed_index} removed")

    @staticmethod
    def _atomic_write(path: Path, writer) -> None:
        temporary = Path(str(path) + ".tmp")
        writer(temporary)
        temporary.replace(path)

    def _write_outputs(self) -> None:
        waypoints = list(self._waypoints)

        def write_wgs84(path: Path) -> None:
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    (
                        "index",
                        "stamp_sec",
                        "latitude_deg",
                        "longitude_deg",
                        "altitude_m",
                        "fix_type",
                        "satellites_min",
                        "hdop_max",
                        "correction_age_sec_max",
                        "samples",
                        "horizontal_spread_m",
                    )
                )
                for index, point in enumerate(waypoints):
                    writer.writerow(
                        (
                            index,
                            f"{point.stamp_sec:.9f}",
                            f"{point.latitude_deg:.10f}",
                            f"{point.longitude_deg:.10f}",
                            f"{point.altitude_m:.4f}",
                            4,
                            point.satellites_min,
                            f"{point.hdop_max:.3f}",
                            f"{point.correction_age_sec_max:.3f}",
                            point.sample_count,
                            f"{point.horizontal_spread_m:.4f}",
                        )
                    )

        def write_route(path: Path) -> None:
            route = build_route_points(
                waypoints, float(self.get_parameter("target_speed_mps").value)
            )
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    ("s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction")
                )
                for point in route:
                    writer.writerow(
                        (
                            f"{point.s_m:.4f}",
                            f"{point.x_m:.4f}",
                            f"{point.y_m:.4f}",
                            f"{point.target_speed_mps:.3f}",
                            point.mission,
                            point.direction,
                        )
                    )

        def write_datum(path: Path) -> None:
            if waypoints:
                first = waypoints[0]
                document = {
                    "gnss_localizer": {
                        "ros__parameters": {
                            "datum_configured": True,
                            "datum_latitude_deg": first.latitude_deg,
                            "datum_longitude_deg": first.longitude_deg,
                            "datum_altitude_m": first.altitude_m,
                        }
                    },
                    "rtk_waypoint_capture": {
                        "waypoint_count": len(waypoints),
                        "wgs84_file": str(self._wgs84_path),
                        "route_file": str(self._route_path),
                    },
                }
            else:
                document = {"rtk_waypoint_capture": {"waypoint_count": 0}}
            with path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(document, stream, sort_keys=False)

        self._atomic_write(self._wgs84_path, write_wgs84)
        self._atomic_write(self._route_path, write_route)
        self._atomic_write(self._datum_path, write_datum)

    def _print_status(self) -> None:
        status = self._status
        if status is None or self._status_rx_sec is None:
            self.get_logger().warning("GNSS STATUS: waiting for /gnss/status")
            return
        age = time.monotonic() - self._status_rx_sec
        fix_name = FIX_NAMES.get(int(status.fix_type), str(int(status.fix_type)))
        correction = float(status.correction_age_sec)
        correction_text = f"{correction:.2f}s" if math.isfinite(correction) else "nan"
        verdict = "READY" if self._current_issue() is None else "NOT_READY"
        self.get_logger().info(
            f"GNSS {verdict}: fix={fix_name}, sats={status.satellites}, "
            f"HDOP={status.hdop:.2f}, corr_age={correction_text}, "
            f"msg_age={age:.2f}s, saved={len(self._waypoints)}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RtkWaypointRecorderNode()
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node is not None and len(node._waypoints) == 0:
            node.get_logger().warning("no waypoint was captured")
        elif node is not None and len(node._waypoints) == 1:
            node.get_logger().warning(
                "only one waypoint: WGS84 data is saved, route is not drivable"
            )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
