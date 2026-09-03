import math
import os

from studica_repeat.core import leg_io
from studica_repeat.core.leg_io import (IndexEntry, LegMeta, Sample, gate_psd, gate_us,
                                         leg_filename, load_index, make_sample, read_leg,
                                         rotate_versions, save_index, upsert_index_entry,
                                         write_leg)


def _samples(n=5):
    out = []
    for i in range(n):
        out.append(make_sample(t=i * 0.02, seg='T', seg_id=0, x=i * 0.01, y=0.0, th=0.0,
                               v=0.3, wz=0.0, enc=[0.1 * i, 0.1 * i, 0.1 * i, 0.0], yaw=0.01,
                               raw={'us_l': 0.4, 'us_r': float('inf'), 'psd_l': 0.15,
                                    'psd_r': 0.5, 'psd_f': float('nan')},
                               dist_valid=True, cmd=[0.3, 0.0, 0.0]))
    return out


def test_gates():
    assert gate_psd(0.15) == 0.15
    assert gate_psd(0.30) is None
    assert gate_psd(0.45) is None
    assert gate_psd(None) is None
    assert gate_psd(float('inf')) is None
    assert gate_psd(float('nan')) is None
    assert gate_us(0.85) == 0.85
    assert gate_us(0.86) is None
    assert gate_us(float('inf')) is None


def test_make_sample_applies_gates():
    s = _samples(1)[0]
    assert s.us_l == 0.4 and s.valid['us_l']
    assert s.us_r is None and not s.valid['us_r']
    assert s.psd_l == 0.15 and s.valid['psd_l']
    assert s.psd_r is None and not s.valid['psd_r']
    assert s.psd_f is None and not s.valid['psd_f']
    blocked = make_sample(0, 'R', 1, 0, 0, 0, 0, 0.5, [0, 0, 0, 0], 0,
                          {'us_l': 0.4, 'psd_l': 0.1}, dist_valid=False)
    assert blocked.us_l is None and not blocked.valid['us_l']


def test_jsonl_roundtrip(tmp_path):
    meta = LegMeta('N1', 'N2', '2026-09-03T00:00:00', 'abc', 50.0, 1, start_yaw=0.3)
    samples = _samples()
    path = os.path.join(tmp_path, leg_filename('N1', 'N2'))
    write_leg(path, meta, samples)
    m2, s2 = read_leg(path)
    assert m2 == meta
    assert len(s2) == len(samples)
    assert s2[0] == samples[0]
    assert s2[-1].x == samples[-1].x
    with open(path, encoding='utf-8') as f:
        first = f.readline()
    assert first.startswith('{"meta"')


def test_index_roundtrip_and_upsert(tmp_path):
    d = str(tmp_path)
    assert load_index(d) == []
    e1 = IndexEntry('N1', 'N2', 'N1__N2.jsonl', 1.2, 5.0, '2026-09-03', 1, 250)
    e2 = IndexEntry('N2', 'N3', 'N2__N3.jsonl', 0.8, 4.0, '2026-09-03', 1, 200, court='A')
    save_index(d, [e1, e2])
    loaded = load_index(d)
    assert [e.name for e in loaded] == ['N1__N2', 'N2__N3']
    assert loaded[1].court == 'A'
    e1b = IndexEntry('N1', 'N2', 'N1__N2.jsonl', 1.3, 5.5, '2026-09-04', 2, 260)
    merged = upsert_index_entry(loaded, e1b)
    assert len(merged) == 2
    assert leg_io.find_entry(merged, 'N1', 'N2').version == 2


def test_rotate_versions_keeps_three(tmp_path):
    d = str(tmp_path)
    meta = LegMeta('A', 'B', 'd')
    assert rotate_versions(d, 'A', 'B') == 1     # 첫 기록: 보존할 것 없음
    versions = []
    for _ in range(5):
        write_leg(os.path.join(d, leg_filename('A', 'B')), meta, _samples(2))
        versions.append(rotate_versions(d, 'A', 'B', keep=3))
    assert versions == [2, 3, 4, 5, 6]
    archived = sorted(f for f in os.listdir(d) if '.v' in f)
    assert archived == ['A__B.v3.jsonl', 'A__B.v4.jsonl', 'A__B.v5.jsonl']
    assert not os.path.exists(os.path.join(d, 'A__B.jsonl'))


def test_leg_length():
    samples = _samples(11)
    assert math.isclose(leg_io.leg_length_m(samples), 0.10, abs_tol=1e-9)
