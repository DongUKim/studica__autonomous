"""레그 앵커링 — 레그-로컬 프레임(기록 시작 pose = 원점, 헤딩 0)과 odom 프레임 사이의 강체변환.

플랜B §4.1: 재생 시작 시 현재 odom pose를 레그 원점에 맞춘다. 이후 체크포인트마다 새로
앵커되므로 오차가 경로 변형으로 누적되지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


def wrap(a: float) -> float:
    """각도를 (-pi, pi]로 접는다."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Anchor:
    x: float
    y: float
    th: float

    @classmethod
    def from_pose(cls, x: float, y: float, th: float) -> 'Anchor':
        return cls(float(x), float(y), float(th))

    def to_local(self, x: float, y: float, th: float) -> Tuple[float, float, float]:
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.th), math.sin(self.th)
        return (c * dx + s * dy, -s * dx + c * dy, wrap(th - self.th))

    def to_world(self, xl: float, yl: float, thl: float) -> Tuple[float, float, float]:
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + c * xl - s * yl, self.y + s * xl + c * yl, wrap(thl + self.th))
