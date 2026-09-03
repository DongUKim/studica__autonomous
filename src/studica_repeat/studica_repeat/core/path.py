"""레그 컴파일 — 기록 샘플을 재생기가 쓰는 T(폴리라인)/R(회전) 세그먼트로 변환.

플랜B §2.2 로드 시 컴파일: T는 호장 2 cm 간격 폴리라인 + θ(s) + v(s), R은 (시작 yaw, 종료 yaw, 방향).
유효 센서 서명은 s 인덱스로 정렬해 둔다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union

import numpy as np

from .anchor import wrap
from .leg_io import CHANNELS, Sample

DS_DEFAULT = 0.02
# 정지 상태 잡음으로 생긴 짧은 T 조각은 경로로 취급하지 않는다.
MIN_T_LENGTH_M = 0.03
# 헤딩락 흔들림 수준의 R은 회전 세그먼트가 아니다.
MIN_R_ANGLE_RAD = math.radians(2.0)
# 서명 조회 허용 반경: 이보다 먼 티칭 샘플은 "그 지점의 서명"이 아니다.
SIG_TOL_M = 0.05
V_SMOOTH_N = 5


@dataclass
class TSegment:
    seg_id: int
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    th: np.ndarray          # unwrap된 연속각
    v: np.ndarray
    signatures: Dict[str, Tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)

    @property
    def kind(self) -> str:
        return 'T'

    @property
    def label(self) -> str:
        return f'T{self.seg_id}'

    @property
    def s_end(self) -> float:
        return float(self.s[-1])

    @property
    def end_xy(self) -> Tuple[float, float]:
        return float(self.x[-1]), float(self.y[-1])

    def project(self, px: float, py: float) -> Tuple[float, float]:
        """점을 폴리라인에 사영 → (호장 s, 횡오차 e_lat). e_lat은 진행방향 좌측이 +."""
        ax, ay = self.x[:-1], self.y[:-1]
        dx, dy = self.x[1:] - ax, self.y[1:] - ay
        seg_len2 = dx * dx + dy * dy
        safe = np.where(seg_len2 > 0, seg_len2, 1.0)
        tpar = np.clip(((px - ax) * dx + (py - ay) * dy) / safe, 0.0, 1.0)
        tpar = np.where(seg_len2 > 0, tpar, 0.0)
        qx, qy = ax + tpar * dx, ay + tpar * dy
        d2 = (px - qx) ** 2 + (py - qy) ** 2
        i = int(np.argmin(d2))
        s = float(self.s[i] + tpar[i] * math.sqrt(seg_len2[i]))
        # 사영점이 꼭짓점이면 그 세그먼트 방향으로 좌/우를 판단한다 (길이 0 세그먼트는 앞뒤 방향 대체).
        ddx, ddy = float(dx[i]), float(dy[i])
        if ddx == 0.0 and ddy == 0.0:
            ddx, ddy = math.cos(self.th[i]), math.sin(self.th[i])
        norm = math.hypot(ddx, ddy)
        e_lat = (ddx * (py - qy[i]) - ddy * (px - qx[i])) / norm
        return s, float(e_lat)

    def theta_at(self, s: float) -> float:
        return float(np.interp(s, self.s, self.th))

    def v_at(self, s: float) -> float:
        return float(np.interp(s, self.s, self.v))

    def signature_at(self, channel: str, s: float) -> Optional[float]:
        """s 지점의 티칭 센서값. 근처(±SIG_TOL_M)에 유효 서명이 없으면 None.

        갭을 가로지르는 보간을 막기 위해 양쪽 이웃이 모두 가까울 때만 선형보간하고,
        한쪽만 가까우면 그 값을 쓴다.
        """
        sig = self.signatures.get(channel)
        if sig is None or len(sig[0]) == 0:
            return None
        ss, vals = sig
        j = int(np.searchsorted(ss, s))
        left = j - 1 if j - 1 >= 0 else None
        right = j if j < len(ss) else None
        dl = (s - ss[left]) if left is not None else math.inf
        dr = (ss[right] - s) if right is not None else math.inf
        if dl <= SIG_TOL_M and dr <= SIG_TOL_M:
            if dl + dr <= 0:
                return float(vals[left])
            return float(vals[left] + (vals[right] - vals[left]) * dl / (dl + dr))
        if dl <= SIG_TOL_M:
            return float(vals[left])
        if dr <= SIG_TOL_M:
            return float(vals[right])
        return None


@dataclass
class RSegment:
    seg_id: int
    start_yaw: float        # IMU 절대 yaw (기록 시)
    end_yaw: float
    direction: int          # +1 반시계, -1 시계
    start_th_local: float   # 레그-로컬 헤딩
    end_th_local: float
    end_yaw_rel: float      # end_yaw - 레그 첫 샘플 yaw (wrap). 재생 시 yaw_local과 비교

    @property
    def kind(self) -> str:
        return 'R'

    @property
    def label(self) -> str:
        return f'R{self.seg_id}'

    @property
    def delta(self) -> float:
        return wrap(self.end_yaw - self.start_yaw)


Segment = Union[TSegment, RSegment]


def _group_by_seg(samples: List[Sample]) -> List[List[Sample]]:
    groups: List[List[Sample]] = []
    for smp in samples:
        if groups and groups[-1][-1].seg_id == smp.seg_id and groups[-1][-1].seg == smp.seg:
            groups[-1].append(smp)
        else:
            groups.append([smp])
    return groups


def _moving_average(a: np.ndarray, n: int) -> np.ndarray:
    if len(a) < 2 or n <= 1:
        return a.copy()
    kernel = np.ones(n) / n
    padded = np.concatenate([np.full(n // 2, a[0]), a, np.full(n - 1 - n // 2, a[-1])])
    return np.convolve(padded, kernel, mode='valid')


def _dedupe_by_s(s: np.ndarray, *cols: np.ndarray) -> Tuple[np.ndarray, ...]:
    """같은 s(정지 구간)의 샘플은 첫 값만 남긴다 — np.interp는 xp가 증가해야 한다."""
    keep = np.concatenate([[True], np.diff(s) > 0])
    return (s[keep],) + tuple(c[keep] for c in cols)


def _compile_t(group: List[Sample], ds: float) -> Optional[TSegment]:
    xs = np.array([p.x for p in group], dtype=float)
    ys = np.array([p.y for p in group], dtype=float)
    ths = np.unwrap(np.array([p.th for p in group], dtype=float))
    vs = _moving_average(np.abs(np.array([p.v for p in group], dtype=float)), V_SMOOTH_N)
    step = np.hypot(np.diff(xs), np.diff(ys))
    s_raw = np.concatenate([[0.0], np.cumsum(step)])
    total = float(s_raw[-1])
    if total < MIN_T_LENGTH_M:
        return None
    s_u, x_u, y_u, th_u, v_u = _dedupe_by_s(s_raw, xs, ys, ths, vs)
    n = int(math.floor(total / ds))
    s_grid = np.arange(0, n + 1) * ds
    if total - s_grid[-1] > 1e-9:
        s_grid = np.append(s_grid, total)
    seg = TSegment(seg_id=group[0].seg_id,
                   s=s_grid,
                   x=np.interp(s_grid, s_u, x_u),
                   y=np.interp(s_grid, s_u, y_u),
                   th=np.interp(s_grid, s_u, th_u),
                   v=np.interp(s_grid, s_u, v_u))
    for ch in CHANNELS:
        pts = [(float(s_raw[i]), float(p.channel(ch))) for i, p in enumerate(group)
               if p.channel(ch) is not None]
        if not pts:
            continue
        arr = np.array(pts, dtype=float)
        # 정지 중 다중 샘플은 평균 하나로 합친다.
        uniq, inv = np.unique(arr[:, 0], return_inverse=True)
        vals = np.zeros(len(uniq))
        counts = np.zeros(len(uniq))
        np.add.at(vals, inv, arr[:, 1])
        np.add.at(counts, inv, 1.0)
        seg.signatures[ch] = (uniq, vals / counts)
    return seg


def _compile_r(group: List[Sample]) -> Optional[RSegment]:
    start_yaw, end_yaw = group[0].yaw, group[-1].yaw
    delta = wrap(end_yaw - start_yaw)
    if abs(delta) < MIN_R_ANGLE_RAD:
        return None
    return RSegment(seg_id=group[0].seg_id, start_yaw=start_yaw, end_yaw=end_yaw,
                    direction=1 if delta > 0 else -1,
                    start_th_local=group[0].th, end_th_local=group[-1].th,
                    end_yaw_rel=0.0)


def compile_leg(samples: List[Sample], ds: float = DS_DEFAULT) -> List[Segment]:
    if not samples:
        return []
    leg_start_yaw = samples[0].yaw
    out: List[Segment] = []
    for group in _group_by_seg(samples):
        if group[0].seg == 'R':
            r = _compile_r(group)
            if r is not None:
                r.end_yaw_rel = wrap(r.end_yaw - leg_start_yaw)
                out.append(r)
        else:
            t = _compile_t(group, ds)
            if t is not None:
                out.append(t)
    return out


def total_length(segments: List[Segment]) -> float:
    return float(sum(seg.s_end for seg in segments if isinstance(seg, TSegment)))
