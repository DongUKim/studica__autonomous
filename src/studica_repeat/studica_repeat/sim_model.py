"""ROS 무의존 기구학 시뮬레이터 모델 — sim_node가 studica_control(HAL)을 대체할 때 쓰는 물리.

모델링 범위(플랜B §7: "노이즈 파라미터를 올려 강건성 확인"이 목적):
  - 듀티 → 휠 속도: FF 역모델 v = (|duty| − kS)/kV · sgn (베이스 FF+P의 정확한 역함수라
    베이스 게인이 맞으면 명령 속도가 그대로 나온다) + 1차 지연 τ
  - 슬립: 지면 이동량 = 휠 회전량 × (1 + N(0, slip_pct/100)). 엔코더는 휠 회전량을 재므로
    slip_pct = 0이면 엔코더 적분 = 실제 pose 가 된다.
  - IMU: 실제 yaw + 일정 드리프트(deg/min) + 각속도 노이즈
  - 거리센서: 장착 기하에서 레이캐스트한 최근접 벽 거리 + 가우시안 노이즈 + 센서 유효범위

정기구학은 studica_base.omni_kinematics 와 같은 식이다(SSoT: CONTRACT §0). colcon 없이도
테스트가 돌도록 여기서 다시 쓴다 — 한쪽을 바꾸면 다른 쪽도 같이 바꿀 것.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

SQ3 = math.sqrt(3.0)

# 센서 유효범위 (studica_control sharp/ultrasonic 컴포넌트와 동일)
PSD_MIN_M, PSD_MAX_M = 0.1, 0.8
US_MIN_M, US_MAX_M = 0.02, 4.0
RANGE_NOISE_SD_M = 0.003
GYRO_NOISE_SD_RADPS = 0.005


@dataclass(frozen=True)
class SensorMount:
    x: float        # 로봇 프레임 장착 위치 [m], 전방 +
    y: float        # 좌측 +
    yaw: float      # 조준 방향 [rad], 반시계 +, 0 = 전방
    kind: str       # 'psd' | 'us'


# 장착 오프셋 기본값 — 실측 전 임시치(tools/robot_geom.py도 전부 0.0 = 미실측)
DEFAULT_MOUNTS: Dict[str, SensorMount] = {
    'us_l': SensorMount(0.00, 0.10, math.radians(90.0), 'us'),
    'us_r': SensorMount(0.00, -0.10, math.radians(-90.0), 'us'),
    'psd_l': SensorMount(0.05, 0.10, math.radians(90.0), 'psd'),
    'psd_r': SensorMount(0.05, -0.10, math.radians(-90.0), 'psd'),
    'psd_f': SensorMount(0.12, 0.00, 0.0, 'psd'),
}

Wall = Tuple[float, float, float, float]   # (x1, y1, x2, y2)


def rectangle_walls(x0: float, y0: float, x1: float, y1: float) -> List[Wall]:
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def default_world() -> List[Wall]:
    """플랜A §2.3 경기장: 4.0×2.0 m + 판형 장애물 2개."""
    walls = rectangle_walls(0.0, 0.0, 4.0, 2.0)
    walls += rectangle_walls(0.66, 0.10, 0.76, 0.60)
    walls += rectangle_walls(2.558, 0.115, 2.658, 0.415)
    return walls


def ray_cast(px: float, py: float, ang: float, walls: Sequence[Wall]) -> float:
    """(px,py)에서 ang 방향으로 쏜 광선이 처음 만나는 벽까지 거리. 없으면 inf."""
    dx, dy = math.cos(ang), math.sin(ang)
    best = math.inf
    for (x1, y1, x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-12:
            continue
        # p + t·d = w1 + u·e
        wx, wy = x1 - px, y1 - py
        t = (wx * ey - wy * ex) / denom
        u = (wx * dy - wy * dx) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0 and t < best:
            best = t
    return best


def forward_kinematics(d0: float, d1: float, d2: float, r: float) -> Tuple[float, float, float]:
    dx = (d0 - d1) / SQ3
    dy = (d0 + d1 - 2.0 * d2) / 3.0
    dth = (d0 + d1 + d2) / (3.0 * r)
    return dx, dy, dth


@dataclass
class SimModel:
    r: float = 0.12
    k_s: float = 0.05
    k_v: float = 1.1
    tau_s: float = 0.1
    slip_pct: float = 2.0
    imu_drift_deg_per_min: float = 1.0
    walls: List[Wall] = field(default_factory=default_world)
    mounts: Dict[str, SensorMount] = field(default_factory=lambda: dict(DEFAULT_MOUNTS))
    range_noise_sd: float = RANGE_NOISE_SD_M
    gyro_noise_sd: float = GYRO_NOISE_SD_RADPS
    x: float = 0.32
    y: float = 0.32
    th: float = 0.0
    seed: Optional[int] = None

    def __post_init__(self) -> None:
        self.duty = [0.0, 0.0, 0.0]
        self.wheel_v = [0.0, 0.0, 0.0]
        self.enc = [0.0, 0.0, 0.0]
        self.wz = 0.0
        self.t = 0.0
        self.imu_drift = 0.0
        self._rng = np.random.default_rng(self.seed)

    # ------------------------------------------------------------------ 입력
    def set_duty(self, motor: int, duty: float) -> None:
        if 0 <= motor < 3 and math.isfinite(duty):
            self.duty[motor] = max(-1.0, min(1.0, duty))

    def _duty_to_speed(self, duty: float) -> float:
        mag = abs(duty)
        if mag <= self.k_s:
            return 0.0
        return math.copysign((mag - self.k_s) / self.k_v, duty)

    # ------------------------------------------------------------------ 적분
    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        alpha = dt / (self.tau_s + dt) if self.tau_s > 0 else 1.0
        disp_truth = [0.0, 0.0, 0.0]
        for i in range(3):
            target = self._duty_to_speed(self.duty[i])
            self.wheel_v[i] += alpha * (target - self.wheel_v[i])
            d_wheel = self.wheel_v[i] * dt
            self.enc[i] += d_wheel
            slip = self._rng.normal(0.0, self.slip_pct / 100.0) if self.slip_pct > 0 else 0.0
            disp_truth[i] = d_wheel * (1.0 + slip)
        dx, dy, dth = forward_kinematics(disp_truth[0], disp_truth[1], disp_truth[2], self.r)
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += c * dx - s * dy
        self.y += s * dx + c * dy
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
        self.wz = dth / dt
        self.imu_drift += math.radians(self.imu_drift_deg_per_min) / 60.0 * dt
        self.t += dt

    # ------------------------------------------------------------------ 출력
    def imu_yaw(self) -> float:
        return math.atan2(math.sin(self.th + self.imu_drift), math.cos(self.th + self.imu_drift))

    def imu_wz(self) -> float:
        noise = self._rng.normal(0.0, self.gyro_noise_sd) if self.gyro_noise_sd > 0 else 0.0
        return self.wz + noise

    def sense(self, noise: bool = True) -> Dict[str, float]:
        """채널별 거리 [m]. 유효범위 밖은 inf (studica_control 컴포넌트와 같은 규약)."""
        out: Dict[str, float] = {}
        c, s = math.cos(self.th), math.sin(self.th)
        for ch, m in self.mounts.items():
            px = self.x + c * m.x - s * m.y
            py = self.y + s * m.x + c * m.y
            d = ray_cast(px, py, self.th + m.yaw, self.walls)
            if noise and math.isfinite(d) and self.range_noise_sd > 0:
                d += self._rng.normal(0.0, self.range_noise_sd)
            lo, hi = (PSD_MIN_M, PSD_MAX_M) if m.kind == 'psd' else (US_MIN_M, US_MAX_M)
            out[ch] = d if lo <= d <= hi else math.inf
        return out


def load_walls(data: dict) -> List[Wall]:
    """sim_world.yaml 파싱: walls: [[x1,y1,x2,y2], ...], rects: [[x0,y0,x1,y1], ...]."""
    walls: List[Wall] = []
    for w in data.get('walls') or []:
        if len(w) != 4:
            raise ValueError(f'wall 항목은 [x1,y1,x2,y2] 4개여야 한다: {w}')
        walls.append(tuple(float(v) for v in w))  # type: ignore[arg-type]
    for r in data.get('rects') or []:
        if len(r) != 4:
            raise ValueError(f'rect 항목은 [x0,y0,x1,y1] 4개여야 한다: {r}')
        walls += rectangle_walls(*(float(v) for v in r))
    return walls
