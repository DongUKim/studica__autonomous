import pytest

from studica_teleop.teleop_state import (
    ANG_PRESETS, LIN_MAX, LIN_MIN, LIN_PRESETS, TeleopState,
)


def test_defaults_mid_preset_and_locked():
    s = TeleopState()
    assert s.lin_speed == LIN_PRESETS[1]
    assert s.ang_speed == ANG_PRESETS[1]
    assert s.rotation_locked
    assert (s.vx, s.vy, s.wz) == (0.0, 0.0, 0.0)


def test_latching_translation():
    s = TeleopState()
    s.press('w')
    assert s.vx == pytest.approx(LIN_PRESETS[1])
    s.press('w')                       # 같은 키 반복은 유지
    assert s.vx == pytest.approx(LIN_PRESETS[1])
    s.press('s')                       # 반대 키는 반전
    assert s.vx == pytest.approx(-LIN_PRESETS[1])
    s.press('a')
    assert s.vy == pytest.approx(+LIN_PRESETS[1])   # 좌 = +y
    s.press('d')
    assert s.vy == pytest.approx(-LIN_PRESETS[1])
    s.press(' ')
    assert (s.vx, s.vy, s.wz) == (0.0, 0.0, 0.0)


def test_rotation_lock_lowercase_blocked_uppercase_allowed():
    s = TeleopState()
    consumed, msg = s.press('q')
    assert consumed and '잠금' in msg
    assert s.wz == 0.0
    s.press('Q')
    assert s.wz == pytest.approx(ANG_PRESETS[1])
    s.press('E')
    assert s.wz == pytest.approx(-ANG_PRESETS[1])


def test_unlock_toggle_allows_lowercase():
    s = TeleopState()
    s.press('l')
    assert not s.rotation_locked
    s.press('e')
    assert s.wz == pytest.approx(-ANG_PRESETS[1])
    s.press('l')
    assert s.rotation_locked


def test_preset_change_rescales_active_components():
    s = TeleopState()
    s.press('s')
    s.press('Q')
    s.press('3')
    assert s.vx == pytest.approx(-LIN_PRESETS[2])
    assert s.wz == pytest.approx(+ANG_PRESETS[2])
    assert s.vy == 0.0
    s.press('1')
    assert s.vx == pytest.approx(-LIN_PRESETS[0])


def test_fine_adjust_and_clamp():
    s = TeleopState()
    s.press('+')
    assert s.lin_speed == pytest.approx(LIN_PRESETS[1] * 1.1)
    s.press('-')
    assert s.lin_speed == pytest.approx(LIN_PRESETS[1])
    for _ in range(100):
        s.press('+')
    assert s.lin_speed == pytest.approx(LIN_MAX)
    for _ in range(200):
        s.press('-')
    assert s.lin_speed == pytest.approx(LIN_MIN)


def test_unknown_key_not_consumed():
    s = TeleopState()
    consumed, msg = s.press('k')
    assert not consumed and msg is None


def test_help_toggle():
    s = TeleopState()
    assert not s.help_visible
    s.press('h')
    assert s.help_visible
