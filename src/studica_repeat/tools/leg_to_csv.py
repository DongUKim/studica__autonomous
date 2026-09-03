#!/usr/bin/env python3
"""티칭 레그 JSONL → 엑셀 검토용 CSV 변환기 (플랜B §2.2).

사용: python leg_to_csv.py <leg.jsonl> [out.csv]
out.csv 생략 시 같은 이름의 .csv 로 저장.
"""
from __future__ import annotations

import csv
import os
import sys

# 오프라인 실행(ROS 미설치 PC) 지원: 패키지 루트를 경로에 추가
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from studica_repeat.core.leg_io import CHANNELS, read_leg  # noqa: E402

ENC_N = 4


def convert(src: str, dst: str) -> int:
    meta, samples = read_leg(src)
    header = (['t', 'seg', 'seg_id', 'x', 'y', 'th', 'v', 'wz']
              + [f'enc_{i}' for i in range(ENC_N)] + ['yaw']
              + list(CHANNELS) + ['cmd_vx', 'cmd_vy', 'cmd_wz']
              + [f'valid_{c}' for c in CHANNELS])
    with open(dst, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow([f'# leg {meta.from_node}->{meta.to_node} date={meta.date} '
                    f'rate_hz={meta.rate_hz} version={meta.version}'])
        w.writerow(header)
        for s in samples:
            enc = list(s.enc) + [''] * (ENC_N - len(s.enc))
            row = [s.t, s.seg, s.seg_id, s.x, s.y, s.th, s.v, s.wz] + enc[:ENC_N] + [s.yaw]
            row += ['' if getattr(s, c) is None else getattr(s, c) for c in CHANNELS]
            row += list(s.cmd) + [''] * (3 - len(s.cmd))
            row += [int(bool(s.valid.get(c, False))) for c in CHANNELS]
            w.writerow(row)
    return len(samples)


def main(argv: list) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    src = argv[1]
    dst = argv[2] if len(argv) > 2 else os.path.splitext(src)[0] + '.csv'
    n = convert(src, dst)
    print(f'{src} -> {dst} ({n} samples)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
