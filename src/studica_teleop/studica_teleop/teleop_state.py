"""키보드 텔레옵의 래칭 상태 머신 (ROS 무의존).

키 → 목표속도 변환 규칙을 노드에서 분리해 pytest로 검증한다.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

LIN_PRESETS = (0.10, 0.20, 0.35)   # m/s 저/중/고
ANG_PRESETS = (0.4, 0.8, 1.2)      # rad/s 저/중/고
DEFAULT_PRESET = 1                 # 중속. 티칭 기본값

LIN_MIN, LIN_MAX = 0.02, 1.0
ANG_MIN, ANG_MAX = 0.1, 3.0
FINE_STEP = 1.1                    # +/- 키 1회당 10 %

MOTION_KEYS = frozenset('wsadQEqe')


class TeleopState:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.lin_speed = LIN_PRESETS[DEFAULT_PRESET]
        self.ang_speed = ANG_PRESETS[DEFAULT_PRESET]
        self.preset = DEFAULT_PRESET
        self.rotation_locked = True
        self.help_visible = False

    # ------------------------------------------------------------------ 속도
    def stop(self) -> None:
        self.vx = self.vy = self.wz = 0.0

    def set_speeds(self, lin: float, ang: float) -> None:
        """프리셋/미세조정으로 속도가 바뀌면 활성 성분을 새 속도로 다시 스케일한다 (방향 유지)."""
        self.lin_speed = min(LIN_MAX, max(LIN_MIN, lin))
        self.ang_speed = min(ANG_MAX, max(ANG_MIN, ang))
        self.vx = math.copysign(self.lin_speed, self.vx) if self.vx else 0.0
        self.vy = math.copysign(self.lin_speed, self.vy) if self.vy else 0.0
        self.wz = math.copysign(self.ang_speed, self.wz) if self.wz else 0.0

    def select_preset(self, idx: int) -> None:
        self.preset = idx
        self.set_speeds(LIN_PRESETS[idx], ANG_PRESETS[idx])

    # ------------------------------------------------------------------ 키 처리
    def press(self, ch: str) -> Tuple[bool, Optional[str]]:
        """키 하나를 적용한다. (소비 여부, 상태줄 메시지)."""
        if ch == 'w':
            self.vx = +self.lin_speed
        elif ch == 's':
            self.vx = -self.lin_speed
        elif ch == 'a':
            self.vy = +self.lin_speed      # 좌측 = +y (REP-103)
        elif ch == 'd':
            self.vy = -self.lin_speed
        elif ch in (' ', 'x'):
            self.stop()
            return True, '정지'
        elif ch in ('Q', 'E'):
            self.wz = self.ang_speed if ch == 'Q' else -self.ang_speed
        elif ch in ('q', 'e'):
            if self.rotation_locked:
                return True, '회전 잠금: Shift+Q/E 사용 (l 로 잠금 해제)'
            self.wz = self.ang_speed if ch == 'q' else -self.ang_speed
        elif ch == 'l':
            self.rotation_locked = not self.rotation_locked
            return True, '회전 잠금 ' + ('ON' if self.rotation_locked else 'OFF')
        elif ch in ('1', '2', '3'):
            self.select_preset(int(ch) - 1)
            return True, f'프리셋 {ch}: {self.lin_speed:.2f} m/s, {self.ang_speed:.2f} rad/s'
        elif ch in ('+', '='):
            self.set_speeds(self.lin_speed * FINE_STEP, self.ang_speed * FINE_STEP)
            return True, f'속도 +10%: {self.lin_speed:.2f} m/s, {self.ang_speed:.2f} rad/s'
        elif ch in ('-', '_'):
            self.set_speeds(self.lin_speed / FINE_STEP, self.ang_speed / FINE_STEP)
            return True, f'속도 -10%: {self.lin_speed:.2f} m/s, {self.ang_speed:.2f} rad/s'
        elif ch == 'h':
            self.help_visible = not self.help_visible
            return True, None
        else:
            return False, None
        return True, None

    @staticmethod
    def is_motion_key(ch: str) -> bool:
        return ch in MOTION_KEYS
