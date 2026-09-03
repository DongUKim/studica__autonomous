import math

import pytest

from studica_repeat.core.relative_move import RelativeMovePlanner

DT = 0.02


def simulate(dx, dy, speed=0.15, lateral_kick=0.0):
    p = RelativeMovePlanner(dx, dy, speed=speed)
    x = y = 0.0
    t = 0.0
    vmax = 0.0
    while t < 20:
        vx, vy, done = p.step(t, x, y)
        if done:
            break
        vmax = max(vmax, math.hypot(vx, vy))
        x += vx * DT
        y += (vy + lateral_kick) * DT
        t += DT
    return x, y, t, vmax


def test_rejects_beyond_limit():
    with pytest.raises(ValueError):
        RelativeMovePlanner(0.3, 0.1)


def test_reaches_target_forward():
    x, y, t, vmax = simulate(0.25, 0.0)
    assert math.isclose(x, 0.25, abs_tol=0.012)
    assert abs(y) < 0.005
    assert vmax <= 0.15 + 1e-9
    assert t < 5


def test_reaches_target_diagonal_backward():
    x, y, t, _ = simulate(-0.1, 0.15)
    assert math.hypot(x + 0.1, y - 0.15) < 0.012


def test_cross_correction_pulls_back_to_line():
    x, y, t, _ = simulate(0.25, 0.0, lateral_kick=0.01)
    assert abs(y) < 0.01


def test_zero_move_done_immediately():
    p = RelativeMovePlanner(0.0, 0.0)
    assert p.step(0.0, 0.0, 0.0) == (0.0, 0.0, True)
