"""RelativeMove — 스테이션 후퇴·바코드 재정렬용 단거리 데드레코닝 이동 (플랜B §4.4).

플랜A의 "0.3 m 이내 개루프 허용" 경계 규칙을 계승: 그 이상은 거절한다.
프레임: 이동 시작 시점의 로봇 pose를 원점으로 하는 로컬 프레임(노드가 Anchor로 만든다).
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

DEFAULT_SPEED = 0.15
DEFAULT_ACCEL = 0.4
MAX_DIST_M = 0.3
END_TOL_M = 0.01
# 정지 마찰을 이기고 출발하기 위한 최저 명령 속도. 이보다 작으면 옴니가 아예 안 움직인다.
V_FLOOR = 0.03
# 직선 이탈 보정 게인 (1/s). 짧은 거리라 약하게만 잡는다.
K_CROSS = 1.5


class RelativeMovePlanner:
    def __init__(self, dx: float, dy: float, speed: float = DEFAULT_SPEED,
                 accel: float = DEFAULT_ACCEL, max_dist: float = MAX_DIST_M,
                 end_tol: float = END_TOL_M) -> None:
        self.dist = math.hypot(dx, dy)
        if self.dist > max_dist:
            raise ValueError(f'RelativeMove {self.dist:.3f} m > 허용 {max_dist:.2f} m')
        self.speed = speed if speed > 0 else DEFAULT_SPEED
        self.accel = accel
        self.end_tol = end_tol
        if self.dist > 0:
            self.ux, self.uy = dx / self.dist, dy / self.dist
        else:
            self.ux, self.uy = 1.0, 0.0
        self._t0: Optional[float] = None
        self.done = self.dist <= end_tol

    def step(self, t: float, xl: float, yl: float) -> Tuple[float, float, bool]:
        """현재 로컬 위치로 (vx, vy, done)을 낸다. 사다리꼴: 시간 램프업 + 남은 거리 감속."""
        if self.done:
            return 0.0, 0.0, True
        if self._t0 is None:
            self._t0 = t
        along = xl * self.ux + yl * self.uy
        remaining = self.dist - along
        if remaining <= self.end_tol:
            self.done = True
            return 0.0, 0.0, True
        v_ramp = V_FLOOR + self.accel * (t - self._t0)
        v_decel = math.sqrt(2.0 * self.accel * remaining)
        v = max(V_FLOOR, min(self.speed, v_ramp, v_decel))
        # 직선에서 벗어난 성분(좌측 +)을 되돌린다.
        cross = self.ux * yl - self.uy * xl
        corr = -K_CROSS * cross
        vx = v * self.ux + corr * (-self.uy)
        vy = v * self.uy + corr * self.ux
        return vx, vy, False
