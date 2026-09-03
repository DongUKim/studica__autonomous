import math

import pytest

from studica_base.omni_kinematics import (
    inverse_kinematics, forward_kinematics, wrap_angle, SQ3_2,
)

R = 0.120


@pytest.mark.parametrize('vx,vy,wz', [
    (0.3, 0.0, 0.0),
    (0.0, 0.2, 0.0),
    (0.0, 0.0, 1.0),
    (0.25, -0.1, 0.7),
    (-0.15, 0.05, -1.2),
    (0.0, 0.0, 0.0),
])
def test_inverse_forward_round_trip(vx, vy, wz):
    # 1 s 동안의 휠 이동량 = 휠 속도 → 정기구학이 원래 속도를 복원해야 한다
    w0, w1, w2 = inverse_kinematics(vx, vy, wz, R)
    dx, dy, dth = forward_kinematics(w0, w1, w2, R)
    assert dx == pytest.approx(vx, abs=1e-12)
    assert dy == pytest.approx(vy, abs=1e-12)
    assert dth == pytest.approx(wz, abs=1e-12)


def test_forward_drive_uses_front_wheels_only():
    # 전방 직진: 후방 휠(2)은 구동 방향이 vx와 수직이라 0
    w0, w1, w2 = inverse_kinematics(1.0, 0.0, 0.0, R)
    assert w0 == pytest.approx(SQ3_2)
    assert w1 == pytest.approx(-SQ3_2)
    assert w2 == pytest.approx(0.0)


def test_strafe_left_direction():
    # 좌측 병진(+vy): 전방 휠 둘 다 +0.5, 후방 휠 -1
    w0, w1, w2 = inverse_kinematics(0.0, 1.0, 0.0, R)
    assert (w0, w1, w2) == pytest.approx((0.5, 0.5, -1.0))


def test_pure_rotation_all_wheels_equal():
    wz = 2.0
    w = inverse_kinematics(0.0, 0.0, wz, R)
    assert all(wi == pytest.approx(R * wz) for wi in w)
    dx, dy, dth = forward_kinematics(*w, R)
    assert dx == pytest.approx(0.0, abs=1e-12)
    assert dy == pytest.approx(0.0, abs=1e-12)
    assert dth == pytest.approx(wz)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
