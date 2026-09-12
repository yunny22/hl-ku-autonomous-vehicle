from hl_ku_core.control import SpeedPiController, pure_pursuit_steering


def controller() -> SpeedPiController:
    return SpeedPiController(
        kp=0.1,
        ki=0.0,
        integral_limit=1.0,
        maximum_duty=0.8,
        deadband_speed_mps=0.03,
        forward_speeds=[0.2, 1.0],
        forward_duties=[0.15, 0.4],
        reverse_speeds=[0.2, 0.6],
        reverse_duties=[0.18, 0.35],
        nominal_voltage=22.2,
    )


def test_speed_controller_forward_reverse_and_stop():
    speed = controller()
    assert speed.update(0.6, 0.0, 22.2, 0.02) > 0.0
    assert speed.update(-0.4, 0.0, 22.2, 0.02) < 0.0
    assert speed.update(0.0, 0.0, 22.2, 0.02) == 0.0


def test_speed_controller_can_apply_a_forward_breakaway_duty():
    speed = controller()
    speed.minimum_forward_duty = 0.30
    assert speed.update(0.10, 0.0, 22.2, 0.02) == 0.30
    assert speed.update(0.0, 0.0, 22.2, 0.02) == 0.0


def test_pure_pursuit_steering_sign_and_limit():
    left = pure_pursuit_steering(0, 0, 0, 2, 1, 0.58, 0.4)
    right = pure_pursuit_steering(0, 0, 0, 2, -1, 0.58, 0.4)
    assert -0.4 <= left < 0.0
    assert 0.0 < right <= 0.4
