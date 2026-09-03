"""검증 모드 리포트 — tracking.csv 기록과 레그별 합격 판정 (플랜B §6)."""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# 합격 기준 초기값 (실기 튜닝으로 조정)
P95_LAT_PASS_M = 0.025
END_ERR_PASS_M = 0.015
END_ERR_PASS_WITH_TRACER_M = 0.025
# 진단 힌트 임계
BIAS_SIGN_FRACTION = 0.8          # 전 레그의 이 비율 이상이 같은 부호 편향이면 지그/IMU 의심
MEAN_LAT_BIAS_MIN_M = 0.010
SIG_BIAS_SUSPECT_M = 0.10         # 서명 보정량이 이만큼 크면 코트 변경 의심

TRACKING_COLUMNS = ('t', 'leg', 'seg', 's', 'e_lat', 'e_th', 'lat_bias', 's_bias')


@dataclass(frozen=True)
class TrackingRow:
    t: float
    leg: str
    seg: str
    s: float
    e_lat: float
    e_th: float
    lat_bias: float
    s_bias: float


def write_tracking_csv(path: str, rows: List[TrackingRow]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(TRACKING_COLUMNS)
        for r in rows:
            w.writerow([f'{r.t:.3f}', r.leg, r.seg, f'{r.s:.4f}', f'{r.e_lat:.4f}',
                        f'{r.e_th:.4f}', f'{r.lat_bias:.4f}', f'{r.s_bias:.4f}'])


def read_tracking_csv(path: str) -> List[TrackingRow]:
    out: List[TrackingRow] = []
    with open(path, 'r', newline='', encoding='utf-8') as f:
        for d in csv.DictReader(f):
            out.append(TrackingRow(float(d['t']), d['leg'], d['seg'], float(d['s']),
                                   float(d['e_lat']), float(d['e_th']),
                                   float(d['lat_bias']), float(d['s_bias'])))
    return out


def summarize(rows: List[TrackingRow], end_errors: Dict[str, float],
              durations: Dict[str, Tuple[float, float]],
              time_budget_s: Optional[float] = None,
              line_tracer: bool = False) -> Dict[str, Any]:
    """레그별 e_lat p95 / 종점 오차 / 소요시간을 집계하고 합격 여부와 진단 힌트를 낸다.

    rows의 e_lat은 T 세그먼트만 집계한다(R은 횡오차 정의가 없다).
    """
    end_pass = END_ERR_PASS_WITH_TRACER_M if line_tracer else END_ERR_PASS_M
    legs: Dict[str, Dict[str, Any]] = {}
    order: List[str] = []
    for r in rows:
        if r.leg not in legs:
            legs[r.leg] = {'e_lat': [], 'lat_bias': [], 's_bias': []}
            order.append(r.leg)
        if r.seg.startswith('T'):
            legs[r.leg]['e_lat'].append(abs(r.e_lat))
            legs[r.leg]['lat_bias'].append(r.lat_bias)
            legs[r.leg]['s_bias'].append(r.s_bias)
    for leg in end_errors:
        if leg not in legs:
            legs[leg] = {'e_lat': [], 'lat_bias': [], 's_bias': []}
            order.append(leg)

    per_leg: List[Dict[str, Any]] = []
    signed_means: List[float] = []
    total_repeat = 0.0
    for leg in order:
        d = legs[leg]
        e = np.array(d['e_lat'], dtype=float)
        p95 = float(np.percentile(e, 95)) if len(e) else 0.0
        end_err = end_errors.get(leg)
        rep_s, taught_s = durations.get(leg, (0.0, 0.0))
        total_repeat += rep_s
        sig_max = float(np.max(np.abs(d['lat_bias']))) if d['lat_bias'] else 0.0
        signed = [r.e_lat for r in rows if r.leg == leg and r.seg.startswith('T')]
        if signed:
            signed_means.append(float(np.mean(signed)))
        ok = p95 <= P95_LAT_PASS_M and (end_err is None or end_err <= end_pass)
        per_leg.append({'leg': leg, 'p95_lat': p95, 'end_err': end_err,
                        'repeat_s': rep_s, 'taught_s': taught_s,
                        'sig_bias_max': sig_max, 'pass': ok})

    all_pass = all(x['pass'] for x in per_leg) if per_leg else False
    budget_ok = time_budget_s is None or total_repeat <= time_budget_s
    hints = _hints(per_leg, signed_means)
    return {'legs': per_leg, 'all_pass': all_pass and budget_ok,
            'total_repeat_s': total_repeat, 'time_budget_s': time_budget_s,
            'budget_ok': budget_ok, 'hints': hints,
            'thresholds': {'p95_lat': P95_LAT_PASS_M, 'end_err': end_pass}}


def _hints(per_leg: List[Dict[str, Any]], signed_means: List[float]) -> List[str]:
    hints: List[str] = []
    failed = [x['leg'] for x in per_leg if not x['pass']]
    if failed and len(failed) < len(per_leg):
        hints.append('특정 레그만 불량 → 해당 레그 재티칭: ' + ', '.join(failed))
    if signed_means:
        sgn = np.sign(signed_means)
        dominant = max(float(np.mean(sgn > 0)), float(np.mean(sgn < 0)))
        if dominant >= BIAS_SIGN_FRACTION and abs(float(np.mean(signed_means))) >= MEAN_LAT_BIAS_MIN_M \
                and len(signed_means) >= 2:
            hints.append('전 레그 동일 방향 편향 → 출발 지그·IMU 캘리브레이션 점검')
    suspicious = [x['leg'] for x in per_leg if x['sig_bias_max'] >= SIG_BIAS_SUSPECT_M]
    if suspicious:
        hints.append('서명 잔차 급증 → 코트 변경 의심(재티칭 필요): ' + ', '.join(suspicious))
    return hints


def format_report(summary: Dict[str, Any]) -> str:
    th = summary['thresholds']
    lines = ['레그              e_lat p95   종점오차   소요/티칭(s)   서명보정max  판정',
             '-' * 78]
    for x in summary['legs']:
        end = f"{x['end_err']*1000:7.1f}mm" if x['end_err'] is not None else '     n/a '
        lines.append(f"{x['leg']:<16} {x['p95_lat']*1000:7.1f}mm  {end}  "
                     f"{x['repeat_s']:5.1f}/{x['taught_s']:5.1f}   "
                     f"{x['sig_bias_max']*1000:7.1f}mm  {'PASS' if x['pass'] else 'FAIL'}")
    lines.append('-' * 78)
    lines.append(f"기준: e_lat p95 ≤ {th['p95_lat']*1000:.0f} mm, 종점 ≤ {th['end_err']*1000:.0f} mm")
    if summary['time_budget_s'] is not None:
        lines.append(f"총 소요 {summary['total_repeat_s']:.1f} s / 예산 {summary['time_budget_s']:.1f} s "
                     f"→ {'OK' if summary['budget_ok'] else 'OVER'}")
    lines.append(f"종합: {'PASS' if summary['all_pass'] else 'FAIL'}")
    for h in summary['hints']:
        lines.append('힌트: ' + h)
    return '\n'.join(lines)
