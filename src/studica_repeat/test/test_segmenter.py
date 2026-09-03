from studica_repeat.core.segmenter import OnlineSegmenter

DT = 0.02


def run(seg, spec):
    """spec: [(duration_s, v, wz), ...] → 상태 리스트 [(t, state)]"""
    t = 0.0
    out = []
    for dur, v, wz in spec:
        n = int(round(dur / DT))
        for _ in range(n):
            out.append((t, seg.update(t, v, wz)))
            t += DT
    return out


def test_t_to_r_to_t_timing():
    seg = OnlineSegmenter()
    states = run(seg, [(1.0, 0.3, 0.0), (1.0, 0.0, 0.5), (1.0, 0.3, 0.0)])
    # 회전 시작 후 0.3 s 전에는 아직 T
    assert all(s.seg == 'T' for t, s in states if t < 1.0 + 0.29)
    # 0.3 s 이후 R, seg_id 1
    r_states = [s for t, s in states if 1.0 + 0.32 <= t < 2.0]
    assert r_states and all(s.seg == 'R' and s.seg_id == 1 for s in r_states)
    # 회전이 멈추고 0.3 s 후 다시 T, seg_id 2
    tail = [s for t, s in states if t >= 2.0 + 0.32]
    assert tail and all(s.seg == 'T' and s.seg_id == 2 for s in tail)


def test_contamination_hold_after_rotation():
    seg = OnlineSegmenter()
    states = run(seg, [(0.5, 0.2, 0.0), (0.6, 0.0, 0.4), (1.0, 0.2, 0.0)])
    assert all(s.dist_valid for t, s in states if t < 0.5)
    # 회전 중과 종료 후 0.3 s 이내는 무효
    assert all(not s.dist_valid for t, s in states if 0.5 <= t < 1.1 + 0.29)
    # 그 뒤 유효 복귀
    late = [s for t, s in states if t >= 1.1 + 0.32]
    assert late and all(s.dist_valid for s in late)


def test_mixed_warning_and_no_r_switch_when_translating():
    seg = OnlineSegmenter()
    states = run(seg, [(1.0, 0.3, 0.5)])
    assert all(s.mixed_warning for _, s in states)
    # 병진 중 회전은 R로 분류하지 않는다 (운전 수칙 위반, 경고만)
    assert all(s.seg == 'T' for _, s in states)
    assert all(not s.dist_valid for _, s in states)


def test_short_rotation_blip_does_not_switch():
    seg = OnlineSegmenter()
    states = run(seg, [(0.5, 0.0, 0.0), (0.2, 0.0, 0.5), (0.5, 0.0, 0.0)])
    assert all(s.seg == 'T' and s.seg_id == 0 for _, s in states)
