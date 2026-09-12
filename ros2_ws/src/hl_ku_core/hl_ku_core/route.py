"""Route file model and nearest/lookahead queries."""

from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

from .geometry import wrap_angle


ALLOWED_MISSIONS = frozenset(
    {
        "NORMAL",
        "HILL",
        "S_OBSTACLE",
        "TRAFFIC",
        "PERP_PARK",
        "DUMMY",
        "PARALLEL_PARK",
        "END_LANE",
        "FINISH",
    }
)


@dataclass(frozen=True)
class Waypoint:
    index: int
    s_m: float
    x_m: float
    y_m: float
    target_speed_mps: float
    mission: str
    direction: int
    # Tangent of increasing route distance, including on reverse legs.
    yaw_rad: float | None = None
    curvature_inv_m: float | None = None


@dataclass(frozen=True)
class PathSample:
    s_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    curvature_inv_m: float
    index: int


@dataclass(frozen=True)
class PathProjection:
    sample: PathSample
    distance_m: float
    heading_error_rad: float


class Route:
    def __init__(self, waypoints: Iterable[Waypoint]) -> None:
        self.waypoints: List[Waypoint] = list(waypoints)
        if len(self.waypoints) < 2:
            raise ValueError("A route needs at least two waypoints")
        previous_s = -math.inf
        for waypoint in self.waypoints:
            if not all(
                math.isfinite(value)
                for value in (
                    waypoint.s_m,
                    waypoint.x_m,
                    waypoint.y_m,
                    waypoint.target_speed_mps,
                )
            ):
                raise ValueError(f"route waypoint {waypoint.index} is non-finite")
            if waypoint.s_m <= previous_s:
                raise ValueError("route s_m values must be strictly increasing")
            if waypoint.target_speed_mps < 0.0:
                raise ValueError("route target speeds must be non-negative")
            if waypoint.mission not in ALLOWED_MISSIONS:
                raise ValueError(f"unknown mission tag: {waypoint.mission}")
            if waypoint.direction not in (-1, 1):
                raise ValueError("route direction must be -1 or 1")
            for value in (waypoint.yaw_rad, waypoint.curvature_inv_m):
                if value is not None and not math.isfinite(value):
                    raise ValueError("route tangent/curvature must be finite")
            previous_s = waypoint.s_m
        # Geometry queries use actual metres, independently of a hand-edited s_m.
        self.stations = [0.0]
        self._segment_yaws: list[float] = []
        for left, right in zip(self.waypoints, self.waypoints[1:]):
            dx, dy = right.x_m - left.x_m, right.y_m - left.y_m
            length = math.hypot(dx, dy)
            if length <= 1.0e-6:
                raise ValueError("consecutive route points must have nonzero spacing")
            self.stations.append(self.stations[-1] + length)
            self._segment_yaws.append(math.atan2(dy, dx))

    @property
    def length_m(self) -> float:
        return self.stations[-1]

    @classmethod
    def load_csv(cls, path: str | Path) -> "Route":
        points: List[Waypoint] = []
        accumulated = 0.0
        previous = None
        with Path(path).open(newline="", encoding="utf-8") as stream:
            for index, row in enumerate(csv.DictReader(stream)):
                x_m = float(row["x_m"])
                y_m = float(row["y_m"])
                if previous is not None:
                    accumulated += math.hypot(x_m - previous[0], y_m - previous[1])
                s_text = row.get("s_m", "").strip()
                s_m = float(s_text) if s_text else accumulated
                direction = int(row.get("direction", "1") or "1")
                if direction not in (-1, 1):
                    raise ValueError(f"direction must be -1 or 1 at row {index + 2}")
                points.append(
                    Waypoint(
                        index=index,
                        s_m=s_m,
                        x_m=x_m,
                        y_m=y_m,
                        target_speed_mps=float(row["target_speed_mps"]),
                        mission=(row.get("mission", "NORMAL") or "NORMAL").upper(),
                        direction=direction,
                        yaw_rad=(float(row["yaw_rad"]) if row.get("yaw_rad", "").strip() else None),
                        curvature_inv_m=(
                            float(row["curvature_inv_m"])
                            if row.get("curvature_inv_m", "").strip() else None
                        ),
                    )
                )
                previous = (x_m, y_m)
        return cls(points)

    def sample_at_s(self, s_m: float) -> PathSample:
        """Continuous target position on the sampled curve; never snap to a vertex."""
        if not math.isfinite(s_m):
            raise ValueError("route station must be finite")
        station = min(self.length_m, max(0.0, s_m))
        index = min(len(self.waypoints) - 2, max(0, bisect.bisect_right(self.stations, station) - 1))
        left, right = self.waypoints[index:index + 2]
        span = self.stations[index + 1] - self.stations[index]
        ratio = (station - self.stations[index]) / span
        yaw = self._segment_yaws[index]
        if left.yaw_rad is not None and right.yaw_rad is not None:
            yaw = wrap_angle(left.yaw_rad + ratio * wrap_angle(right.yaw_rad - left.yaw_rad))
        curvature = 0.0
        if left.curvature_inv_m is not None and right.curvature_inv_m is not None:
            curvature = left.curvature_inv_m + ratio * (right.curvature_inv_m - left.curvature_inv_m)
        elif 0 < index < len(self._segment_yaws):
            local_length = 0.5 * (self.stations[index + 1] - self.stations[index - 1])
            curvature = wrap_angle(self._segment_yaws[index] - self._segment_yaws[index - 1]) / local_length
        return PathSample(station, left.x_m + ratio * (right.x_m - left.x_m),
                          left.y_m + ratio * (right.y_m - left.y_m), yaw, curvature, index)

    def project(
        self, x_m: float, y_m: float, vehicle_yaw_rad: float,
        minimum_s_m: float, maximum_s_m: float,
        heading_weight_m: float = 0.6,
        maximum_heading_error_rad: float = 2.1,
    ) -> PathProjection | None:
        """Match nearby segments using position AND travel heading in a metric window."""
        if not all(math.isfinite(v) for v in (x_m, y_m, vehicle_yaw_rad,
                minimum_s_m, maximum_s_m, heading_weight_m, maximum_heading_error_rad)):
            raise ValueError("projection inputs must be finite")
        low, high = max(0.0, minimum_s_m), min(self.length_m, maximum_s_m)
        if low > high:
            return None
        first = max(0, bisect.bisect_right(self.stations, low) - 1)
        last = min(len(self.waypoints) - 2, bisect.bisect_right(self.stations, high) - 1)
        best: PathProjection | None = None
        best_score = math.inf
        for index in range(first, last + 1):
            left, right = self.waypoints[index:index + 2]
            dx, dy = right.x_m - left.x_m, right.y_m - left.y_m
            span = self.stations[index + 1] - self.stations[index]
            ratio = ((x_m - left.x_m) * dx + (y_m - left.y_m) * dy) / (span * span)
            station = max(low, min(high, self.stations[index] + max(0.0, min(1.0, ratio)) * span))
            sample = self.sample_at_s(station)
            body_yaw = sample.yaw_rad + (math.pi if left.direction < 0 else 0.0)
            heading_error = wrap_angle(body_yaw - vehicle_yaw_rad)
            if abs(heading_error) > maximum_heading_error_rad:
                continue
            distance = math.hypot(sample.x_m - x_m, sample.y_m - y_m)
            score = distance * distance + (heading_weight_m * heading_error) ** 2
            if score < best_score:
                best_score = score
                best = PathProjection(sample, distance, heading_error)
        return best

    def nearest_index(self, x_m: float, y_m: float, hint: int = 0, window: int = 80) -> int:
        start = max(0, hint - max(5, window // 4))
        stop = min(len(self.waypoints), max(start + 1, hint + window))
        return min(
            range(start, stop),
            key=lambda index: (
                (self.waypoints[index].x_m - x_m) ** 2
                + (self.waypoints[index].y_m - y_m) ** 2
            ),
        )

    def lookahead_index(self, start_index: int, lookahead_m: float) -> int:
        target_s = self.waypoints[start_index].s_m + max(0.0, lookahead_m)
        for index in range(start_index, len(self.waypoints)):
            if self.waypoints[index].s_m >= target_s:
                return index
        return len(self.waypoints) - 1

    def metadata_waypoint(
        self,
        nearest_index: int,
        x_m: float,
        y_m: float,
        finish_tolerance_m: float,
    ) -> Waypoint:
        """Delay FINISH speed/tag until the vehicle reaches the final point."""
        nearest = self.waypoints[nearest_index]
        if nearest.mission != "FINISH":
            return nearest
        final = self.waypoints[-1]
        distance = math.hypot(final.x_m - x_m, final.y_m - y_m)
        if distance <= max(0.0, finish_tolerance_m):
            return final
        for waypoint in reversed(self.waypoints[:nearest_index]):
            if waypoint.mission != "FINISH":
                return waypoint
        return nearest
