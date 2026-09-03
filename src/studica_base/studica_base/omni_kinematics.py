"""3륜 옴니 기구학 (ROS 무의존).

휠 배치는 구코드 national_project ControlCore.cpp의 믹싱 행렬을 그대로 이식했다:
모터 0/1 = 전방 좌우(구동 방향 30°/150°), 모터 2 = 후방(270°). 전방에는 휠이 없다.
좌표 규약: vx 전방(+x), vy 좌측(+y), wz 반시계 +.
"""
import math
from typing import Tuple

SQ3_2 = 0.8660254037844386
SQ3 = 1.7320508075688772


def inverse_kinematics(vx: float, vy: float, wz: float, r: float) -> Tuple[float, float, float]:
    """로봇 속도 (m/s, rad/s) → 휠 선속도 [m/s] (w0, w1, w2)."""
    rot = r * wz
    w0 = vx * SQ3_2 + vy * 0.5 + rot
    w1 = -vx * SQ3_2 + vy * 0.5 + rot
    w2 = -vy + rot
    return (w0, w1, w2)


def forward_kinematics(d0: float, d1: float, d2: float, r: float) -> Tuple[float, float, float]:
    """휠 이동량 [m] (d0, d1, d2) → 로봇 프레임 이동 (dx, dy, dth).

    역기구학 행렬의 정확한 역행렬이므로 왕복 시 손실이 없다.
    dth는 IMU가 없을 때만 헤딩 대체용으로 쓴다(휠 슬립에 취약).
    """
    dx = (d0 - d1) / SQ3
    dy = (d0 + d1 - 2.0 * d2) / 3.0
    dth = (d0 + d1 + d2) / (3.0 * r)
    return (dx, dy, dth)


def wrap_angle(a: float) -> float:
    """각도를 (-pi, pi]로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))
