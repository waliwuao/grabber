from spear_locator.position_pid import PositionPid, PositionPidConfig


def test_pid_proportional_output_and_speed_limit():
    pid = PositionPid(
        PositionPidConfig(
            kp=2.0,
            ki=0.0,
            kd=0.0,
            maximum_speed_mps=0.05,
        )
    )
    assert pid.update(0.01, 0.1) == 0.02
    assert pid.update(0.10, 0.1) == 0.05
    assert pid.update(-0.10, 0.1) == -0.05


def test_pid_deadband_stops_and_resets_integral():
    pid = PositionPid(
        PositionPidConfig(
            kp=0.0,
            ki=1.0,
            kd=0.0,
            maximum_speed_mps=1.0,
            deadband_m=0.005,
        )
    )
    assert pid.update(0.02, 1.0) > 0.0
    assert pid.integral_m_s > 0.0
    assert pid.update(0.003, 1.0) == 0.0
    assert pid.integral_m_s == 0.0


def test_pid_derivative_uses_measurement_interval():
    pid = PositionPid(
        PositionPidConfig(
            kp=0.0,
            ki=0.0,
            kd=1.0,
            maximum_speed_mps=1.0,
            derivative_filter=0.0,
        )
    )
    assert pid.update(0.01, 0.0) == 0.0
    assert abs(pid.update(0.03, 0.5) - 0.04) < 1e-12
