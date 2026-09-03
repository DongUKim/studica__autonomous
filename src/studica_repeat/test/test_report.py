import os

from studica_repeat.core.report import (TrackingRow, format_report, read_tracking_csv,
                                        summarize, write_tracking_csv)


def rows_for(leg, e_lat, n=100, seg='T0', bias=0.0):
    return [TrackingRow(i * 0.02, leg, seg, i * 0.005, e_lat, 0.0, bias, 0.0) for i in range(n)]


def test_csv_roundtrip(tmp_path):
    rows = rows_for('A__B', 0.01, n=5) + rows_for('A__B', 0.0, n=2, seg='R1')
    path = os.path.join(tmp_path, 'tracking.csv')
    write_tracking_csv(path, rows)
    back = read_tracking_csv(path)
    assert len(back) == 7
    assert back[0].leg == 'A__B' and back[-1].seg == 'R1'
    assert abs(back[0].e_lat - 0.01) < 1e-9


def test_summary_pass_and_fail():
    rows = rows_for('A__B', 0.01) + rows_for('B__C', 0.04)
    summary = summarize(rows, {'A__B': 0.01, 'B__C': 0.01},
                        {'A__B': (5.0, 5.0), 'B__C': (6.0, 5.0)}, time_budget_s=20.0)
    legs = {x['leg']: x for x in summary['legs']}
    assert legs['A__B']['pass'] and not legs['B__C']['pass']
    assert not summary['all_pass']
    assert any('B__C' in h and '재티칭' in h for h in summary['hints'])
    text = format_report(summary)
    assert 'FAIL' in text and 'PASS' in text and 'B__C' in text


def test_summary_r_rows_ignored_and_budget():
    rows = rows_for('A__B', 0.005) + rows_for('A__B', 0.5, seg='R1')
    summary = summarize(rows, {'A__B': 0.005}, {'A__B': (30.0, 5.0)}, time_budget_s=20.0)
    assert summary['legs'][0]['pass']
    assert not summary['budget_ok'] and not summary['all_pass']


def test_hint_same_direction_bias():
    rows = rows_for('A__B', 0.02) + rows_for('B__C', 0.02) + rows_for('C__D', 0.02)
    summary = summarize(rows, {}, {})
    assert any('지그' in h for h in summary['hints'])


def test_hint_signature_bias():
    rows = rows_for('A__B', 0.005, bias=0.12)
    summary = summarize(rows, {}, {})
    assert any('코트 변경' in h for h in summary['hints'])
