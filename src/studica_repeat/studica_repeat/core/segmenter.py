"""온라인 세그먼트 분할기 — 텔레옵 기록 중 T(병진)/R(회전) 자동 판정과 회전 오염 게이트.

플랜B §2.1: |wz| > 0.1 rad/s 이고 병진속도 < 0.05 m/s 가 0.3 s 지속 → R, 그 외 T.
플랜B §2.2 규칙(3): R 세그먼트 중과 회전 종료 후 0.3 s 동안 모든 거리채널 무효.
벽시계 대신 호출자가 주는 타임스탬프만 쓴다(재생·회귀 테스트에서 결정적이어야 하므로).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

SEG_T = 'T'
SEG_R = 'R'


@dataclass(frozen=True)
class SegState:
    seg: str            # 'T' | 'R'
    seg_id: int         # T/R 전환마다 1 증가
    dist_valid: bool    # 거리센서 기록 허용 여부(회전 오염 게이트)
    mixed_warning: bool  # 병진+회전 동시 조작(운전 수칙 위반) 경고


class OnlineSegmenter:
    def __init__(self, wz_thresh: float = 0.1, v_thresh: float = 0.05,
                 dwell_s: float = 0.3, contam_hold_s: float = 0.3) -> None:
        self.wz_thresh = wz_thresh
        self.v_thresh = v_thresh
        self.dwell_s = dwell_s
        self.contam_hold_s = contam_hold_s
        self.seg = SEG_T
        self.seg_id = 0
        self._cand_since: Optional[float] = None   # 전환 후보 조건이 시작된 시각
        self._last_rot_t: Optional[float] = None   # 마지막으로 |wz|>thresh 였던 시각

    def update(self, t: float, v: float, wz: float) -> SegState:
        rotating = abs(wz) > self.wz_thresh
        slow = v < self.v_thresh
        if rotating:
            self._last_rot_t = t

        # 전환 후보: T에서는 "회전 중이며 병진 정지", R에서는 "회전 멈춤". dwell 동안 지속돼야 전환.
        candidate = (rotating and slow) if self.seg == SEG_T else (not rotating)
        if candidate:
            if self._cand_since is None:
                self._cand_since = t
            elif t - self._cand_since >= self.dwell_s:
                self.seg = SEG_R if self.seg == SEG_T else SEG_T
                self.seg_id += 1
                self._cand_since = None
        else:
            self._cand_since = None

        # 회전이 끝난 뒤에도 hold 시간 동안은 초음파/PSD 판독이 회전 잔상에 오염된 것으로 본다.
        settled = self._last_rot_t is None or (t - self._last_rot_t) >= self.contam_hold_s
        dist_valid = (self.seg == SEG_T) and (not rotating) and settled
        mixed = rotating and not slow
        return SegState(self.seg, self.seg_id, dist_valid, mixed)
