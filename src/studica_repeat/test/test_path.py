import math

import numpy as np

from studica_repeat.core.leg_io import make_sample
from studica_repeat.core.path import RSegment, TSegment, compile_leg

DT = 0.02


def straight_leg(length=1.0, v=0.25, psd_l=None, yaw0=0.0):
    """+x 방향 직선 레그. psd_l은 (s_from, s_to, value)로 구간 서명을 준다."""
    samples = []
    t = 0.0
    x = 0.0
    while x <= length + 1e-9:
        raw = {}
        if psd_l and psd_l[0] <= x <= psd_l[1]:
            raw['psd_l'] = psd_l[2]
        samples.append(make_sample(t, 'T', 0, x, 0.0, 0.0, v, 0.0, [x, x, 0.0, 0.0], yaw0,
                                   raw, dist_valid=True))
        t += DT
        x += v * DT
    return samples


def test_straight_leg_compiles_to_one_t_segment():
    segs = compile_leg(straight_leg(1.0))
    assert len(segs) == 1 and isinstance(segs[0], TSegment)
    seg = segs[0]
    assert math.isclose(seg.s_end, 1.0, abs_tol=0.01)
    assert np.allclose(np.diff(seg.s)[:-1], 0.02)
    assert math.isclose(seg.end_xy[0], seg.s_end, abs_tol=1e-6)
    assert math.isclose(seg.theta_at(0.5), 0.0, abs_tol=1e-9)
    assert math.isclose(seg.v_at(0.5), 0.25, abs_tol=1e-6)


def test_projection_sign_and_s():
    seg = compile_leg(straight_leg(1.0))[0]
    s, e = seg.project(0.4, 0.03)
    assert math.isclose(s, 0.4, abs_tol=1e-6)
    assert math.isclose(e, 0.03, abs_tol=1e-9)     # 경로 좌측 → +
    s, e = seg.project(0.4, -0.05)
    assert math.isclose(e, -0.05, abs_tol=1e-9)
    s, e = seg.project(1.5, 0.0)                    # 종점 너머 → s 클램프
    assert math.isclose(s, seg.s_end, abs_tol=1e-9)


def test_projection_on_curved_path_uses_segment_interior():
    # 45도 대각선 경로: 꼭짓점 사이 점도 정확히 사영돼야 한다
    samples = []
    for i in range(60):
        d = i * 0.01
        samples.append(make_sample(i * DT, 'T', 0, d, d, math.pi / 4, 0.3, 0.0,
                                   [0, 0, 0, 0], 0.0, {}, True))
    seg = compile_leg(samples)[0]
    s, e = seg.project(0.205, 0.195)
    assert math.isclose(s, 0.2 * math.sqrt(2), abs_tol=2e-3)
    assert e < 0     # 대각선 우하단 = 경로 우측
    assert math.isclose(seg.theta_at(0.1), math.pi / 4, abs_tol=1e-9)


def test_signature_interp_and_gap():
    seg = compile_leg(straight_leg(1.0, psd_l=(0.3, 0.6, 0.15)))[0]
    assert 'psd_l' in seg.signatures
    assert math.isclose(seg.signature_at('psd_l', 0.45), 0.15, abs_tol=1e-9)
    assert seg.signature_at('psd_l', 0.1) is None          # 서명 없는 구간
    assert seg.signature_at('psd_l', 0.9) is None
    assert seg.signature_at('psd_l', 0.62) is not None     # 끝에서 5 cm 이내
    assert seg.signature_at('us_r', 0.45) is None          # 채널 없음


def test_l_shaped_leg_compiles_t_r_t():
    v = 0.25
    samples = []
    t = 0.0
    # T0: +x 0.5 m
    x = 0.0
    while x <= 0.5:
        samples.append(make_sample(t, 'T', 0, x, 0.0, 0.0, v, 0.0, [0] * 4, 0.0, {}, True))
        t += DT
        x += v * DT
    # R1: 제자리 +90도 회전 (yaw 0→pi/2)
    th = 0.0
    while th < math.pi / 2:
        samples.append(make_sample(t, 'R', 1, 0.5, 0.0, th, 0.0, 1.0, [0] * 4, th, {}, False))
        t += DT
        th += 1.0 * DT
    # T2: +y 0.4 m
    y = 0.0
    while y <= 0.4:
        samples.append(make_sample(t, 'T', 2, 0.5, y, math.pi / 2, v, 0.0, [0] * 4,
                                   math.pi / 2, {}, True))
        t += DT
        y += v * DT
    segs = compile_leg(samples)
    assert [s.kind for s in segs] == ['T', 'R', 'T']
    r = segs[1]
    assert isinstance(r, RSegment) and r.direction == 1
    assert math.isclose(r.end_yaw_rel, math.pi / 2, abs_tol=0.03)
    assert math.isclose(segs[2].theta_at(0.2), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(segs[2].s_end, 0.4, abs_tol=0.01)


def test_tiny_segments_dropped():
    samples = [make_sample(i * DT, 'T', 0, 0.001 * i, 0, 0, 0.0, 0.0, [0] * 4, 0, {}, True)
               for i in range(10)]
    samples += [make_sample(1 + i * DT, 'R', 1, 0.01, 0, 0, 0.0, 0.2, [0] * 4, 0.01 * i, {}, False)
                for i in range(3)]
    assert compile_leg(samples) == []
