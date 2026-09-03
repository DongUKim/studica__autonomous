"""재생 제어기 — 기록된 경로를 50 Hz 폐루프로 추종한다 (플랜B §4).

핵심: cmd_vel을 재생하는 것이 아니라 매 주기 오차로부터 새로 계산한다(개루프 재생은 발산).
진행변수는 시간이 아니라 호장 s.

프레임 규약(노드와의 계약):
  - (xl, yl, thl): odom pose를 Anchor.to_local로 바꾼 레그-로컬 pose.
  - yaw_abs_local: wrap(IMU yaw_now − 앵커 시점 IMU yaw). R 세그먼트는 이것과
    RSegment.end_yaw_rel(= 기록 종료 yaw − 기록 시작 yaw)을 비교한다. 즉 지그 출발이면
    기록 시작과 재생 시작의 절대 yaw 차이가 자동으로 오프셋된다.
  - sensors: {'us_l','us_r','psd_l','psd_r','psd_f'} → 게이트 통과한 현재 측정(m) 또는 None.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

from .anchor import wrap
from .leg_io import FRONT_CHANNELS, LEFT_CHANNELS, RIGHT_CHANNELS
from .path import RSegment, Segment, TSegment


@dataclass
class RepeatGains:
    k_lat: float = 1.5            # vy = -k_lat * e_lat  (1/s)
    k_th: float = 2.0             # wz = k_th * e_th     (1/s)
    k_yaw: float = 2.5            # R 세그먼트 yaw P 게인
    lookahead: float = 0.06       # 헤딩 목표를 앞서 보는 거리 (m)
    vmax: float = 0.35
    vmin: float = 0.05            # 말단 감속 하한 — 정지마찰로 멈추지 않게
    decel: float = 0.4            # v <= sqrt(2*decel*s_remain)
    end_tol: float = 0.02
    end_debounce_s: float = 0.2
    dev_lat: float = 0.08         # 이탈 판정 횡오차
    dev_debounce_s: float = 0.5
    wz_max: float = 1.2
    yaw_tol: float = math.radians(1.0)
    yaw_debounce_s: float = 0.3
    sig_k: float = 0.3            # 서명 보정 게인
    sig_res_max: float = 0.15     # 잔차 게이트
    sig_wz_max: float = 0.1       # 회전 중 서명 미반영
    sig_median_n: int = 5


@dataclass(frozen=True)
class StepResult:
    vx: float
    vy: float
    wz: float
    done: bool
    aborted: bool
    reason: str
    seg_label: str
    s: float
    s_end: float
    e_lat: float
    e_th: float
    lat_bias: float
    s_bias: float


_ZERO = (0.0, 0.0, 0.0)


class RepeatController:
    def __init__(self, segments: List[Segment], gains: Optional[RepeatGains] = None,
                 speed_scale: float = 1.0) -> None:
        self.segments = list(segments)
        self.g = gains or RepeatGains()
        self.speed_scale = speed_scale if speed_scale > 0 else 1.0
        self.idx = 0
        self.done = not self.segments
        self.aborted = False
        self.reason = ''
        self.lat_bias = 0.0
        self.s_bias = 0.0
        self._end_since: Optional[float] = None
        self._dev_since: Optional[float] = None
        self._yaw_since: Optional[float] = None
        self._sig_buf: Dict[str, Deque[float]] = {}

    # ------------------------------------------------------------------ 상태
    @property
    def current(self) -> Optional[Segment]:
        if self.done or self.aborted or self.idx >= len(self.segments):
            return None
        return self.segments[self.idx]

    def _advance(self) -> None:
        self.idx += 1
        self._end_since = self._dev_since = self._yaw_since = None
        self._sig_buf.clear()
        # 세그먼트가 바뀌면 횡/종 축의 의미가 달라지므로 보정량은 이월하지 않는다.
        self.lat_bias = self.s_bias = 0.0
        if self.idx >= len(self.segments):
            self.done = True

    def _abort(self, reason: str) -> None:
        self.aborted = True
        self.reason = reason

    def _result(self, cmd, seg_label: str, s: float, s_end: float,
                e_lat: float, e_th: float) -> StepResult:
        return StepResult(cmd[0], cmd[1], cmd[2], self.done, self.aborted, self.reason,
                          seg_label, s, s_end, e_lat, e_th, self.lat_bias, self.s_bias)

    # ------------------------------------------------------------------ 메인
    def step(self, t: float, xl: float, yl: float, thl: float, yaw_abs_local: float,
             wz_meas: float, sensors: Dict[str, Optional[float]]) -> StepResult:
        seg = self.current
        if seg is None:
            return self._result(_ZERO, '', 0.0, 0.0, 0.0, 0.0)
        if isinstance(seg, TSegment):
            return self._step_t(seg, t, xl, yl, thl, wz_meas, sensors)
        return self._step_r(seg, t, yaw_abs_local)

    # ------------------------------------------------------------------ T
    def _step_t(self, seg: TSegment, t: float, xl: float, yl: float, thl: float,
                wz_meas: float, sensors: Dict[str, Optional[float]]) -> StepResult:
        g = self.g
        s_raw, e_lat_raw = seg.project(xl, yl)
        s_now = min(s_raw + self.s_bias, seg.s_end)
        self._update_signatures(seg, s_now, e_lat_raw, wz_meas, sensors)
        s_now = min(s_raw + self.s_bias, seg.s_end)
        e_lat = e_lat_raw + self.lat_bias
        e_th = wrap(seg.theta_at(min(s_now + g.lookahead, seg.s_end)) - thl)

        # 이탈 감시
        if abs(e_lat) > g.dev_lat:
            self._dev_since = t if self._dev_since is None else self._dev_since
            if t - self._dev_since >= g.dev_debounce_s:
                self._abort(f'{seg.label}: lateral deviation {e_lat:.3f} m')
                return self._result(_ZERO, seg.label, s_now, seg.s_end, e_lat, e_th)
        else:
            self._dev_since = None

        # 종료 판정: 호장 끝 도달 + 종점 근접이 디바운스 동안 유지.
        # 종점 거리는 odom 좌표 그대로가 아니라 서명 보정이 반영된 (종방향, e_lat)로 잰다 —
        # 드리프트를 보정해 실제로는 경로 위에 있는데 odom만 틀어진 경우에도 끝나야 한다.
        s_remain = seg.s_end - s_now
        overshoot = self._overshoot(seg, xl, yl)
        end_dist = math.hypot(max(s_remain, overshoot), e_lat)
        at_end = s_now >= seg.s_end - 1e-9 and end_dist < g.end_tol
        if at_end:
            self._end_since = t if self._end_since is None else self._end_since
            if t - self._end_since >= g.end_debounce_s:
                self._advance()
                return self._result(_ZERO, seg.label, s_now, seg.s_end, e_lat, e_th)
        else:
            self._end_since = None

        vmax = g.vmax * self.speed_scale
        if s_remain > 0:
            v_prof = min(seg.v_at(s_now) * self.speed_scale, vmax)
            v_decel = math.sqrt(2.0 * g.decel * s_remain)
            vx = max(min(v_prof, v_decel), g.vmin)
        else:
            vx = 0.0
        vy = max(-vmax, min(vmax, -g.k_lat * e_lat))
        wz = max(-g.wz_max, min(g.wz_max, g.k_th * e_th))
        return self._result((vx, vy, wz), seg.label, s_now, seg.s_end, e_lat, e_th)

    @staticmethod
    def _overshoot(seg: TSegment, xl: float, yl: float) -> float:
        """종점을 지나친 종방향 거리(+). 사영은 s_end에서 클램프되므로 따로 잰다."""
        ex, ey = seg.end_xy
        dx, dy = math.cos(seg.th[-1]), math.sin(seg.th[-1])
        return max(0.0, (xl - ex) * dx + (yl - ey) * dy)

    def _update_signatures(self, seg: TSegment, s_now: float, e_lat_raw: float,
                           wz_meas: float, sensors: Dict[str, Optional[float]]) -> None:
        """드리프트 트리머: 티칭 서명 대비 잔차의 미디언으로 e_lat/s 추정을 저게인 갱신.

        측면 채널의 잔차는 그 자체가 "센서가 말하는 실제 e_lat"이므로 bias 목표 = 잔차 − odom e_lat.
        전방 채널의 잔차는 이미 s_bias가 반영된 s_now 기준이므로 증분으로 더한다.
        """
        g = self.g
        for ch, meas in sensors.items():
            taught = seg.signature_at(ch, s_now) if meas is not None else None
            if meas is None or taught is None:
                # 서명 연속성이 끊기면 미디언 창을 비운다 — 이전 구간 잔차와 섞이면 안 된다.
                self._sig_buf.pop(ch, None)
                continue
            buf = self._sig_buf.setdefault(ch, deque(maxlen=g.sig_median_n))
            buf.append(taught - meas)
            if len(buf) < g.sig_median_n or abs(wz_meas) > g.sig_wz_max:
                continue
            res = sorted(buf)[len(buf) // 2]
            if abs(res) > g.sig_res_max:
                continue
            # 좌측 벽이 가까워짐(taught > meas) = 로봇이 경로 좌측 = e_lat +.
            if ch in LEFT_CHANNELS:
                self.lat_bias += g.sig_k * ((res - e_lat_raw) - self.lat_bias)
            elif ch in RIGHT_CHANNELS:
                self.lat_bias += g.sig_k * ((-res - e_lat_raw) - self.lat_bias)
            elif ch in FRONT_CHANNELS:
                # 전방 벽이 가까워짐 = 티칭보다 앞서 있음 = s +.
                self.s_bias += g.sig_k * res

    # ------------------------------------------------------------------ R
    def _step_r(self, seg: RSegment, t: float, yaw_abs_local: float) -> StepResult:
        g = self.g
        err = wrap(seg.end_yaw_rel - yaw_abs_local)
        if abs(err) < g.yaw_tol:
            self._yaw_since = t if self._yaw_since is None else self._yaw_since
            if t - self._yaw_since >= g.yaw_debounce_s:
                self._advance()
                return self._result(_ZERO, seg.label, 0.0, 0.0, 0.0, err)
        else:
            self._yaw_since = None
        wz = max(-g.wz_max, min(g.wz_max, g.k_yaw * err))
        return self._result((0.0, 0.0, wz), seg.label, 0.0, 0.0, 0.0, err)
