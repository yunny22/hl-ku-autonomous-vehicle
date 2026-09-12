from hl_ku_core.mission import (
    LIGHT_GREEN,
    LIGHT_RED,
    MissionCoordinator,
    Observation,
    State,
)


def observation(now, zone="NORMAL", speed=0.0, **kwargs):
    return Observation(now_sec=now, zone=zone, route_s_m=kwargs.pop("route_s_m", 0.0), speed_mps=speed, **kwargs)


def test_not_armed_is_braked():
    coordinator = MissionCoordinator()
    decision = coordinator.update(observation(0.0))
    assert decision.state == State.INIT
    assert decision.brake


def test_hill_requires_full_hold_time():
    coordinator = MissionCoordinator(hill_stop_s_m=10.0, hill_hold_sec=3.2)
    coordinator.arm()
    stopped = coordinator.update(observation(1.0, "HILL", route_s_m=10.0))
    assert stopped.state == State.HILL_HOLD
    assert stopped.brake
    holding = coordinator.update(observation(4.0, "HILL", route_s_m=10.0))
    assert holding.state == State.HILL_HOLD
    launch = coordinator.update(observation(4.3, "HILL", route_s_m=10.0))
    assert launch.state == State.HILL_CLIMB
    assert not launch.brake


def test_traffic_red_stop_green_debounce():
    coordinator = MissionCoordinator(green_debounce_sec=0.35)
    coordinator.arm()
    red = coordinator.update(
        observation(
            1.0,
            "TRAFFIC",
            stop_line_detected=True,
            stop_line_distance_m=1.0,
            traffic_light=LIGHT_RED,
        )
    )
    assert red.state == State.TRAFFIC_WAIT
    first_green = coordinator.update(observation(2.0, "TRAFFIC", traffic_light=LIGHT_GREEN))
    assert first_green.brake
    go = coordinator.update(observation(2.4, "TRAFFIC", traffic_light=LIGHT_GREEN))
    assert go.state == State.ROUTE
    assert not go.brake


def test_dummy_must_stop_hold_and_clear():
    coordinator = MissionCoordinator(dummy_trigger_m=4.0, dummy_hold_sec=3.2)
    coordinator.arm()
    braking = coordinator.update(
        observation(1.0, "DUMMY", speed=0.5, obstacle_in_path=True, obstacle_distance_m=3.0)
    )
    assert braking.state == State.DUMMY_BRAKE
    stopped = coordinator.update(
        observation(2.0, "DUMMY", speed=0.0, obstacle_in_path=True, obstacle_distance_m=2.0)
    )
    assert stopped.state == State.DUMMY_HOLD
    wait = coordinator.update(observation(5.3, "DUMMY", speed=0.0, obstacle_in_path=True))
    assert wait.brake
    cleared = coordinator.update(observation(5.4, "DUMMY", speed=0.0, obstacle_in_path=False))
    assert cleared.state == State.ROUTE
