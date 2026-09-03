"""레그 파일 I/O — JSONL 샘플, index.yaml, 버전 보존, 센서 게이트.

플랜B §2.2 포맷. 파일 배치:
  <taught_legs>/index.yaml
  <taught_legs>/N1__N2.jsonl        최신
  <taught_legs>/N1__N2.vK.jsonl     이전 버전(최근 keep개 보존)
"""
from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Tuple

import yaml

CHANNELS: Tuple[str, ...] = ('us_l', 'us_r', 'psd_l', 'psd_r', 'psd_f')
LEFT_CHANNELS: Tuple[str, ...] = ('us_l', 'psd_l')
RIGHT_CHANNELS: Tuple[str, ...] = ('us_r', 'psd_r')
FRONT_CHANNELS: Tuple[str, ...] = ('psd_f',)

# 플랜B 규칙(2): PSD는 0.30 m 이상이면 장거리 편차 때문에 기록하지 않는다(사용자 요구).
PSD_MAX_M = 0.30
# 구코드 us_max_mm=850 — 초음파 물리 상한. 그 이상은 에코 미스로 본다.
US_MAX_M = 0.85

INDEX_FILE = 'index.yaml'
INDEX_VERSION = 1
LEG_META_VERSION = 1


def _finite(d: Optional[float]) -> bool:
    return d is not None and isinstance(d, (int, float)) and math.isfinite(d)


def gate_psd(d: Optional[float]) -> Optional[float]:
    """PSD 게이트: 유한하고 0.30 m 미만일 때만 값을 돌려준다."""
    if not _finite(d) or d >= PSD_MAX_M:
        return None
    return float(d)


def gate_us(d: Optional[float]) -> Optional[float]:
    """초음파 게이트: 유한하고 0.85 m 이하일 때만 값을 돌려준다."""
    if not _finite(d) or d > US_MAX_M:
        return None
    return float(d)


def gate_channel(channel: str, d: Optional[float]) -> Optional[float]:
    if channel in FRONT_CHANNELS or channel in ('psd_l', 'psd_r'):
        return gate_psd(d)
    return gate_us(d)


@dataclass
class LegMeta:
    from_node: str
    to_node: str
    date: str
    robot_cfg_hash: str = ''
    rate_hz: float = 50.0
    version: int = LEG_META_VERSION
    start_yaw: float = 0.0   # 레그 시작 시 IMU 절대 yaw — R 세그먼트 목표를 상대각으로 바꾸는 기준

    def to_dict(self) -> Dict[str, Any]:
        return {'from': self.from_node, 'to': self.to_node, 'date': self.date,
                'robot_cfg_hash': self.robot_cfg_hash, 'rate_hz': self.rate_hz,
                'version': self.version, 'start_yaw': self.start_yaw}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'LegMeta':
        return cls(from_node=str(d['from']), to_node=str(d['to']), date=str(d.get('date', '')),
                   robot_cfg_hash=str(d.get('robot_cfg_hash', '')),
                   rate_hz=float(d.get('rate_hz', 50.0)), version=int(d.get('version', 1)),
                   start_yaw=float(d.get('start_yaw', 0.0)))

    @property
    def name(self) -> str:
        return leg_name(self.from_node, self.to_node)


@dataclass
class Sample:
    t: float
    seg: str
    seg_id: int
    x: float
    y: float
    th: float
    v: float
    wz: float
    enc: List[float]
    yaw: float
    us_l: Optional[float] = None
    us_r: Optional[float] = None
    psd_l: Optional[float] = None
    psd_r: Optional[float] = None
    psd_f: Optional[float] = None
    cmd: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    valid: Dict[str, bool] = field(default_factory=dict)

    def channel(self, name: str) -> Optional[float]:
        return getattr(self, name) if self.valid.get(name, False) else None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'Sample':
        return cls(t=float(d['t']), seg=str(d['seg']), seg_id=int(d.get('seg_id', 0)),
                   x=float(d['x']), y=float(d['y']), th=float(d['th']),
                   v=float(d['v']), wz=float(d['wz']), enc=[float(e) for e in d.get('enc', [])],
                   yaw=float(d.get('yaw', 0.0)),
                   us_l=d.get('us_l'), us_r=d.get('us_r'), psd_l=d.get('psd_l'),
                   psd_r=d.get('psd_r'), psd_f=d.get('psd_f'),
                   cmd=[float(c) for c in d.get('cmd', [0.0, 0.0, 0.0])],
                   valid={k: bool(v) for k, v in d.get('valid', {}).items()})


def make_sample(t: float, seg: str, seg_id: int, x: float, y: float, th: float,
                v: float, wz: float, enc: List[float], yaw: float,
                raw: Dict[str, Optional[float]], dist_valid: bool,
                cmd: Optional[List[float]] = None) -> Sample:
    """원시 센서값에 채널 게이트와 회전 오염 게이트를 적용해 Sample을 만든다."""
    gated: Dict[str, Optional[float]] = {}
    valid: Dict[str, bool] = {}
    for ch in CHANNELS:
        val = gate_channel(ch, raw.get(ch)) if dist_valid else None
        gated[ch] = val
        valid[ch] = val is not None
    return Sample(t=t, seg=seg, seg_id=seg_id, x=x, y=y, th=th, v=v, wz=wz,
                  enc=list(enc), yaw=yaw, cmd=list(cmd) if cmd else [0.0, 0.0, 0.0],
                  valid=valid, **gated)


def leg_name(from_node: str, to_node: str) -> str:
    return f'{from_node}__{to_node}'


def leg_filename(from_node: str, to_node: str) -> str:
    return leg_name(from_node, to_node) + '.jsonl'


def write_leg(path: str, meta: LegMeta, samples: List[Sample]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(json.dumps({'meta': meta.to_dict()}, ensure_ascii=False) + '\n')
        for s in samples:
            f.write(json.dumps(s.to_dict(), ensure_ascii=False) + '\n')


def read_leg(path: str) -> Tuple[LegMeta, List[Sample]]:
    meta: Optional[LegMeta] = None
    samples: List[Sample] = []
    with open(path, 'r', encoding='utf-8') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if 'meta' in d:
                meta = LegMeta.from_dict(d['meta'])
                continue
            samples.append(Sample.from_dict(d))
    if meta is None:
        raise ValueError(f'{path}: 첫 행에 meta가 없다')
    return meta, samples


@dataclass
class IndexEntry:
    from_node: str
    to_node: str
    file: str
    length_m: float
    duration_s: float
    recorded: str
    version: int
    samples: int
    court: str = ''

    @property
    def name(self) -> str:
        return leg_name(self.from_node, self.to_node)

    def to_dict(self) -> Dict[str, Any]:
        return {'from': self.from_node, 'to': self.to_node, 'file': self.file,
                'length_m': round(float(self.length_m), 4),
                'duration_s': round(float(self.duration_s), 3),
                'recorded': self.recorded, 'version': int(self.version),
                'samples': int(self.samples), 'court': self.court}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'IndexEntry':
        return cls(from_node=str(d['from']), to_node=str(d['to']), file=str(d['file']),
                   length_m=float(d.get('length_m', 0.0)), duration_s=float(d.get('duration_s', 0.0)),
                   recorded=str(d.get('recorded', '')), version=int(d.get('version', 1)),
                   samples=int(d.get('samples', 0)), court=str(d.get('court', '') or ''))


def load_index(legs_dir: str) -> List[IndexEntry]:
    path = os.path.join(legs_dir, INDEX_FILE)
    if not os.path.exists(path):
        return []
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return [IndexEntry.from_dict(e) for e in (data.get('legs') or [])]


def save_index(legs_dir: str, entries: List[IndexEntry]) -> None:
    os.makedirs(legs_dir, exist_ok=True)
    data = {'version': INDEX_VERSION, 'legs': [e.to_dict() for e in entries]}
    with open(os.path.join(legs_dir, INDEX_FILE), 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def upsert_index_entry(entries: List[IndexEntry], entry: IndexEntry) -> List[IndexEntry]:
    out = [e for e in entries if not (e.from_node == entry.from_node and e.to_node == entry.to_node)]
    out.append(entry)
    return out


def find_entry(entries: List[IndexEntry], from_node: str, to_node: str) -> Optional[IndexEntry]:
    for e in entries:
        if e.from_node == from_node and e.to_node == to_node:
            return e
    return None


def _archived_versions(legs_dir: str, from_node: str, to_node: str) -> List[Tuple[int, str]]:
    pat = re.compile(r'^' + re.escape(leg_name(from_node, to_node)) + r'\.v(\d+)\.jsonl$')
    found: List[Tuple[int, str]] = []
    if not os.path.isdir(legs_dir):
        return found
    for fn in os.listdir(legs_dir):
        m = pat.match(fn)
        if m:
            found.append((int(m.group(1)), os.path.join(legs_dir, fn)))
    return sorted(found)


def rotate_versions(legs_dir: str, from_node: str, to_node: str, keep: int = 3) -> int:
    """현재 최신 파일을 .vK.jsonl로 보존하고, 새 기록이 가질 버전 번호를 돌려준다.

    K = 기존 보존본 최대 번호 + 1. 보존본은 최신 keep개만 남긴다(디스크·혼동 방지).
    최신 파일이 없으면 1을 돌려준다(첫 기록).
    """
    current = os.path.join(legs_dir, leg_filename(from_node, to_node))
    archived = _archived_versions(legs_dir, from_node, to_node)
    if not os.path.exists(current):
        return (archived[-1][0] + 1) if archived else 1
    k = (archived[-1][0] + 1) if archived else 1
    os.replace(current, os.path.join(legs_dir, f'{leg_name(from_node, to_node)}.v{k}.jsonl'))
    if keep > 0:
        for _ver, path in _archived_versions(legs_dir, from_node, to_node)[:-keep]:
            os.remove(path)
    return k + 1


def leg_length_m(samples: List[Sample]) -> float:
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total
