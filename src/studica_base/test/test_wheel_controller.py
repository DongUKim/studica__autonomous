import pytest

from studica_base.wheel_controller import slew, ff_p_duty, clamp, sign, VEL_DEADBAND_MPS


def test_slew_limits_step_and_reaches_target():
    cur = 0.0
    step = 0.8 * 0.02  # 0.8 m/s^2 at 50 Hz
    cur = slew(cur, 1.0, step)
    assert cur == pytest.approx(step)
    cur = slew(cur, -1.0, step)
    assert cur == pytest.approx(0.0)
    assert slew(0.5, 0.505, step) == pytest.approx(0.505)   # 스텝 이내면 바로 도달
    assert slew(0.5, 0.5, step) == 0.5


def test_slew_converges_to_target():
    cur = 0.0
    for _ in range(100):
        cur = slew(cur, 0.3, 0.016)
    assert cur == pytest.approx(0.3)


def test_ff_p_deadband_gives_zero():
    assert ff_p_duty(0.0, 0.0, 0.05, 1.1, 0.4) == 0.0
    assert ff_p_duty(VEL_DEADBAND_MPS / 2, 0.0, 0.05, 1.1, 0.4) == 0.0
    # 목표가 0인데 측정 속도가 남아 있어도 P항이 역듀티를 내면 안 된다(정지 시 진동)
    assert ff_p_duty(0.0, 0.2, 0.05, 1.1, 0.4) == 0.0


def test_ff_p_formula_and_sign():
    ref, meas = 0.2, 0.1
    expected = 0.05 * 1 + 1.1 * 0.2 + 0.4 * (0.2 - 0.1)
    assert ff_p_duty(ref, meas, 0.05, 1.1, 0.4) == pytest.approx(expected)
    neg = ff_p_duty(-ref, -meas, 0.05, 1.1, 0.4)
    assert neg == pytest.approx(-expected)


def test_ff_p_clamps_to_max_duty():
    assert ff_p_duty(5.0, 0.0, 0.05, 1.1, 0.4) == 1.0
    assert ff_p_duty(-5.0, 0.0, 0.05, 1.1, 0.4) == -1.0
    assert ff_p_duty(5.0, 0.0, 0.05, 1.1, 0.4, max_duty=0.7) == 0.7


def test_helpers():
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.3, -1.0, 1.0) == 0.3
    assert sign(0.0) == 0.0 and sign(3.0) == 1.0 and sign(-3.0) == -1.0
