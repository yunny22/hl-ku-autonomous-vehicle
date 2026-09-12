"""Regulation-oriented mission state machine without ROS dependencies."""

from __future__ import annotations

import dataclasses
from enum import IntEnum


class State(IntEnum):
    INIT = 0
    READY = 1
    ROUTE = 2
    HILL_APPROACH = 10
    HILL_HOLD = 11
    HILL_CLIMB = 12
    S_OBSTACLE = 20
    TRAFFIC_APPROACH = 30
    TRAFFIC_WAIT = 31
    PERP_PARK = 40
    DUMMY_ARMED = 50
    DUMMY_BRAKE = 51
    DUMMY_HOLD = 52
    PARALLEL_PARK = 60
    END_LANE = 70
    FINISH = 80
    FAULT = 255


LIGHT_UNKNOWN = 0
LIGHT_RED = 1
LIGHT_YELLOW = 2
LIGHT_GREEN = 3


@dataclasses.dataclass(frozen=True)
class Observation:
    now_sec: float
    zone: str
    route_s_m: float
    speed_mps: float
    stop_line_detected: bool = False
    stop_line_distance_m: float = float("inf")
    traffic_light: int = LIGHT_UNKNOWN
    obstacle_in_path: bool = False
    obstacle_distance_m: float = float("inf")
    allowed_lane: int = 0


@dataclasses.dataclass(frozen=True)
class Decision:
    state: State
    speed_limit_mps: float
    brake: bool
    lateral_offset_m: float
    detail: str


class MissionCoordinator:
    def __init__(
        self,
        default_speed_mps: float = 1.0,
        hill_stop_s_m: float = 0.0,
        hill_hold_sec: float = 3.2,
        hill_speed_mps: float = 0.7,
        stopped_speed_mps: float = 0.04,
        dummy_trigger_m: float = 4.0,
        dummy_hold_sec: float = 3.2,
        traffic_stop_trigger_m: float = 2.0,
        green_debounce_sec: float = 0.35,
        end_lane_offset_m: float = 1.5,
    ) -> None:
        self.default_speed_mps = default_speed_mps
        self.hill_stop_s_m = hill_stop_s_m
        self.hill_hold_sec = hill_hold_sec
        self.hill_speed_mps = hill_speed_mps
        self.stopped_speed_mps = stopped_speed_mps
        self.dummy_trigger_m = dummy_trigger_m
        self.dummy_hold_sec = dummy_hold_sec
        self.traffic_stop_trigger_m = traffic_stop_trigger_m
        self.green_debounce_sec = green_debounce_sec
        self.end_lane_offset_m = end_lane_offset_m
        self.state = State.INIT
        self._hold_start_sec: float | None = None
        self._green_start_sec: float | None = None
        self._last_zone = ""

    def arm(self) -> None:
        if self.state == State.INIT:
            self.state = State.READY

    def fault(self) -> None:
        self.state = State.FAULT

    def _zone_entry(self, zone: str) -> None:
        if zone == self._last_zone:
            return
        self._last_zone = zone
        self._hold_start_sec = None
        self._green_start_sec = None
        mapping = {
            "NORMAL": State.ROUTE,
            "HILL": State.HILL_APPROACH,
            "S_OBSTACLE": State.S_OBSTACLE,
            "TRAFFIC": State.TRAFFIC_APPROACH,
            "PERP_PARK": State.PERP_PARK,
            "DUMMY": State.DUMMY_ARMED,
            "PARALLEL_PARK": State.PARALLEL_PARK,
            "END_LANE": State.END_LANE,
            "FINISH": State.FINISH,
        }
        self.state = mapping.get(zone, State.ROUTE)

    def update(self, observation: Observation) -> Decision:
        if self.state == State.FAULT:
            return Decision(self.state, 0.0, True, 0.0, "fault latched")
        if self.state == State.INIT:
            return Decision(self.state, 0.0, True, 0.0, "not armed")
        self._zone_entry(observation.zone.upper())

        if self.state == State.FINISH:
            return Decision(self.state, 0.0, True, 0.0, "course finished")

        if self.state == State.HILL_APPROACH:
            if observation.route_s_m >= self.hill_stop_s_m:
                if abs(observation.speed_mps) <= self.stopped_speed_mps:
                    self.state = State.HILL_HOLD
                    self._hold_start_sec = observation.now_sec
                return Decision(self.state, 0.0, True, 0.0, "hill stop")
            return Decision(self.state, min(0.5, self.hill_speed_mps), False, 0.0, "hill approach")

        if self.state == State.HILL_HOLD:
            if abs(observation.speed_mps) > self.stopped_speed_mps:
                self._hold_start_sec = observation.now_sec
            start = self._hold_start_sec if self._hold_start_sec is not None else observation.now_sec
            held = observation.now_sec - start
            if held >= self.hill_hold_sec:
                self.state = State.HILL_CLIMB
                return Decision(self.state, self.hill_speed_mps, False, 0.0, "hill launch")
            return Decision(self.state, 0.0, True, 0.0, f"hill hold {held:.2f}s")

        if self.state == State.HILL_CLIMB:
            return Decision(self.state, self.hill_speed_mps, False, 0.0, "hill climb")

        if self.state == State.TRAFFIC_APPROACH:
            must_stop = observation.traffic_light != LIGHT_GREEN
            close = (
                observation.stop_line_detected
                and observation.stop_line_distance_m <= self.traffic_stop_trigger_m
            )
            if must_stop and close:
                self.state = State.TRAFFIC_WAIT
                return Decision(self.state, 0.0, True, 0.0, "traffic stop")
            return Decision(self.state, 0.45, False, 0.0, "traffic approach")

        if self.state == State.TRAFFIC_WAIT:
            if observation.traffic_light == LIGHT_GREEN:
                if self._green_start_sec is None:
                    self._green_start_sec = observation.now_sec
                if observation.now_sec - self._green_start_sec >= self.green_debounce_sec:
                    self.state = State.ROUTE
                    return Decision(self.state, self.default_speed_mps, False, 0.0, "green")
            else:
                self._green_start_sec = None
            return Decision(self.state, 0.0, True, 0.0, "red or unknown")

        if self.state == State.DUMMY_ARMED:
            if (
                observation.obstacle_in_path
                and observation.obstacle_distance_m <= self.dummy_trigger_m
            ):
                self.state = State.DUMMY_BRAKE
                return Decision(self.state, 0.0, True, 0.0, "dummy emergency brake")
            return Decision(self.state, min(1.0, self.default_speed_mps), False, 0.0, "dummy armed")

        if self.state == State.DUMMY_BRAKE:
            if abs(observation.speed_mps) <= self.stopped_speed_mps:
                self.state = State.DUMMY_HOLD
                self._hold_start_sec = observation.now_sec
            return Decision(self.state, 0.0, True, 0.0, "waiting for full stop")

        if self.state == State.DUMMY_HOLD:
            if abs(observation.speed_mps) > self.stopped_speed_mps:
                self._hold_start_sec = observation.now_sec
            start = self._hold_start_sec if self._hold_start_sec is not None else observation.now_sec
            held = observation.now_sec - start
            if held >= self.dummy_hold_sec and not observation.obstacle_in_path:
                self.state = State.ROUTE
                return Decision(self.state, 0.4, False, 0.0, "dummy cleared")
            return Decision(self.state, 0.0, True, 0.0, f"dummy hold {held:.2f}s")

        if self.state == State.S_OBSTACLE:
            return Decision(self.state, 0.45, False, 0.0, "LiDAR avoidance active")

        if self.state in (State.PERP_PARK, State.PARALLEL_PARK):
            return Decision(self.state, 0.25, False, 0.0, "follow calibrated parking route")

        if self.state == State.END_LANE:
            lane = max(-1, min(1, observation.allowed_lane))
            return Decision(
                self.state,
                0.4,
                False,
                lane * self.end_lane_offset_m,
                "follow allowed end lane" if lane else "end signal unknown",
            )

        self.state = State.ROUTE
        return Decision(self.state, self.default_speed_mps, False, 0.0, "route")
