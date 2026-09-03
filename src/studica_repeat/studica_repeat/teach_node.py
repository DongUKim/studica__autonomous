#!/usr/bin/env python3
"""기록기(teach 모드) — 키보드 텔레옵을 확장해 레그를 JSONL로 기록한다 (플랜B §3).

실행: ros2 run studica_repeat teach_node --ros-args -p mission:=mission_a
      (tty 필요. 베이스·HAL은 `ros2 launch studica_repeat teach.launch.py`로 먼저 띄운다)

기록 대상은 cmd_vel이 아니라 추정기(오도메트리) pose 궤적 + 센서 서명이다. 재생은 이 궤적을
폐루프로 추종한다(플랜B §1). 기록 중에는 read_line(텍스트 입력)을 쓰지 않는다 — 입력 프롬프트는
기록 시작 전([ 키)과 폐기 확인(ESC, 정지 후)에서만 뜬다.

키: [ 기록 시작(from/to 입력)  ] 종료·저장  ESC 폐기  r 같은 레그 재기록
서비스: /repeat/record_start (RecordStart), /repeat/record_stop (RecordStop)
"""
from __future__ import annotations

import datetime
import math
import os
import re
import time
from typing import Dict, List, Optional

import rclpy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64MultiArray

from studica_interfaces.srv import RecordStart, RecordStop
from studica_repeat.core.anchor import Anchor
from studica_repeat.core.leg_io import (CHANNELS, IndexEntry, LegMeta, Sample, leg_filename,
                                        leg_length_m, load_index, make_sample, rotate_versions,
                                        save_index, upsert_index_entry, write_leg)
from studica_repeat.core.segmenter import OnlineSegmenter, SegState
from studica_teleop.keyboard_teleop import KeyboardTeleop
from studica_teleop.terminal import ESC

RANGE_PARAMS: Dict[str, str] = {
    'us_l': 'ultrasonic_left_topic',
    'us_r': 'ultrasonic_right_topic',
    'psd_l': 'psd_left_topic',
    'psd_r': 'psd_right_topic',
    'psd_f': 'psd_front_topic',
}
RANGE_DEFAULTS: Dict[str, str] = {
    'us_l': '/ultrasonic_left/range',
    'us_r': '/ultrasonic_right/range',
    'psd_l': '/psd_left/range',
    'psd_r': '/psd_right/range',
    'psd_f': '/psd_front/range',
}
# 기록 시작·종료는 정지 상태에서(운전 수칙 3). 오도메트리 속도가 이보다 크면 거절
STOPPED_V_MPS = 0.02
SENSOR_STALE_S = 0.3
NODE_NAME_RE = re.compile(r'^[A-Za-z0-9_\-.]+$')
MIN_SAMPLES_TO_SAVE = 10

RULES_LINES = [
    '수칙 1) 레그는 병진 위주로 — 회전은 꼭 필요한 곳에서만 (Shift+Q/E)',
    '수칙 2) 보정 존(벽 근처 직선)은 벽에서 15~25 cm 유지 — PSD 30 cm 게이트 안이어야 서명 기록',
    '수칙 3) 시작·종료는 정지 상태에서. 종료 지점 = 다음 레그 시작 지점',
    '수칙 4) 티칭 속도가 재생 상한 — 최종 티칭은 목표 경기 속도로',
]


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TeachNode(KeyboardTeleop):
    def __init__(self) -> None:
        super().__init__('teach_node')
        d = self.declare_parameter
        d('mission', 'mission_a')
        d('missions_dir', '~/studica_missions')
        d('sample_rate_hz', 50.0)
        d('imu_topic', '/imu')
        d('encoders_topic', '/base/encoders')
        for ch, pname in RANGE_PARAMS.items():
            d(pname, RANGE_DEFAULTS[ch])

        self.mission = str(self.get_parameter('mission').value)
        self.legs_dir = os.path.join(os.path.expanduser(str(self.get_parameter('missions_dir').value)),
                                     self.mission, 'taught_legs')
        self.rate_hz = float(self.get_parameter('sample_rate_hz').value)

        self.last_imu: Optional[Imu] = None
        self.last_enc: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.last_range: Dict[str, float] = {}
        self.last_range_t: Dict[str, float] = {}

        self.create_subscription(Imu, str(self.get_parameter('imu_topic').value), self._on_imu, 10)
        self.create_subscription(Float64MultiArray, str(self.get_parameter('encoders_topic').value),
                                 self._on_enc, 10)
        for ch, pname in RANGE_PARAMS.items():
            self.create_subscription(Range, str(self.get_parameter(pname).value),
                                     lambda msg, ch=ch: self._on_range(ch, msg), 10)
        self.create_service(RecordStart, '/repeat/record_start', self._srv_start)
        self.create_service(RecordStop, '/repeat/record_stop', self._srv_stop)

        # 기록 상태
        self.recording = False
        self.from_node = ''
        self.to_node = ''
        self.last_pair: Optional[tuple] = None
        self.samples: List[Sample] = []
        self.anchor: Optional[Anchor] = None
        self.segmenter = OnlineSegmenter()
        self.seg_state: Optional[SegState] = None
        self.t0 = 0.0
        self.start_yaw = 0.0
        self.arc_len = 0.0
        self._last_xy: Optional[tuple] = None
        self._last_sample: Optional[Sample] = None
        self._mixed_warned = False

        self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_sample)
        os.makedirs(self.legs_dir, exist_ok=True)
        self._leg_count = len(load_index(self.legs_dir))   # HUD용 캐시 — 저장 때만 갱신
        self.get_logger().info(f'teach_node: mission={self.mission} legs_dir={self.legs_dir}')

    # ------------------------------------------------------------------ 구독
    def _on_imu(self, msg: Imu) -> None:
        self.last_imu = msg

    def _on_enc(self, msg: Float64MultiArray) -> None:
        vals = [float(v) for v in msg.data][:4]
        while len(vals) < 4:
            vals.append(0.0)
        self.last_enc = vals

    def _on_range(self, ch: str, msg: Range) -> None:
        self.last_range[ch] = float(msg.range)
        self.last_range_t[ch] = time.monotonic()

    # ------------------------------------------------------------------ 상태 조회
    def _odom_pose(self) -> Optional[tuple]:
        if self.last_odom is None:
            return None
        p = self.last_odom.pose.pose
        return (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _odom_speed(self) -> float:
        if self.last_odom is None:
            return 0.0
        tw = self.last_odom.twist.twist
        return math.hypot(tw.linear.x, tw.linear.y)

    def _wz_meas(self) -> float:
        if self.last_imu is not None:
            return float(self.last_imu.angular_velocity.z)
        if self.last_odom is not None:
            return float(self.last_odom.twist.twist.angular.z)
        return 0.0

    def _imu_yaw(self) -> float:
        if self.last_imu is not None:
            return _yaw_from_quat(self.last_imu.orientation)
        pose = self._odom_pose()
        return pose[2] if pose else 0.0

    def _raw_sensors(self) -> Dict[str, Optional[float]]:
        now = time.monotonic()
        out: Dict[str, Optional[float]] = {}
        for ch in CHANNELS:
            t = self.last_range_t.get(ch)
            out[ch] = self.last_range[ch] if (t is not None and now - t < SENSOR_STALE_S) else None
        return out

    def _is_stopped(self) -> bool:
        cmd_zero = abs(self.vx_cmd) < 1e-6 and abs(self.vy_cmd) < 1e-6 and abs(self.wz_cmd) < 1e-6
        return cmd_zero and self._odom_speed() < STOPPED_V_MPS

    def _known_nodes(self) -> List[str]:
        names = set()
        for e in load_index(self.legs_dir):
            names.add(e.from_node)
            names.add(e.to_node)
        graph_file = os.path.join(os.path.dirname(self.legs_dir), 'graph.yaml')
        if os.path.exists(graph_file):
            try:
                with open(graph_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                names.update(str(n) for n in (data.get('nodes') or []))
            except Exception as e:  # graph.yaml 오류는 티칭을 막지 않는다 — 상태줄로만 알림
                self.set_status(f'graph.yaml 읽기 실패: {e}')
        return sorted(names)

    # ------------------------------------------------------------------ 기록 제어
    def start_recording(self, from_node: str, to_node: str) -> tuple:
        """(성공 여부, 메시지). 키와 서비스가 공용."""
        if self.recording:
            return False, '이미 기록 중'
        for n in (from_node, to_node):
            if not NODE_NAME_RE.match(n):
                return False, f'체크포인트 이름 부적합: {n!r} (영숫자 _ - . 만)'
        if from_node == to_node:
            return False, 'from 과 to 가 같다'
        pose = self._odom_pose()
        if pose is None:
            return False, '/odom 수신 없음 — 베이스가 떠 있는지 확인'
        if not self._is_stopped():
            return False, '정지 상태에서만 시작 가능 (space)'
        self.from_node, self.to_node = from_node, to_node
        self.last_pair = (from_node, to_node)
        self.samples = []
        self.anchor = Anchor.from_pose(*pose)
        self.segmenter = OnlineSegmenter()
        self.seg_state = None
        self.t0 = time.monotonic()
        self.start_yaw = self._imu_yaw()
        self.arc_len = 0.0
        self._last_xy = None
        self._last_sample = None
        self._mixed_warned = False
        self.recording = True
        return True, f'REC {from_node} → {to_node}'

    def stop_recording(self, save: bool) -> tuple:
        """(성공 여부, 메시지, 저장 경로)."""
        if not self.recording:
            return False, '기록 중이 아님', ''
        self.recording = False
        if not save:
            n = len(self.samples)
            self.samples = []
            return True, f'폐기됨 ({n} 샘플)', ''
        if len(self.samples) < MIN_SAMPLES_TO_SAVE:
            n = len(self.samples)
            self.samples = []
            return False, f'샘플 부족({n}) — 저장 안 함', ''
        try:
            path = self._save()
        except Exception as e:
            self.get_logger().error(f'레그 저장 실패: {e}')
            return False, f'저장 실패: {e}', ''
        return True, f'저장: {os.path.basename(path)} ({len(self.samples)} 샘플, {self.arc_len:.2f} m)', path

    def _save(self) -> str:
        version = rotate_versions(self.legs_dir, self.from_node, self.to_node)
        now = datetime.datetime.now().isoformat(timespec='seconds')
        meta = LegMeta(from_node=self.from_node, to_node=self.to_node, date=now,
                       rate_hz=self.rate_hz, version=version, start_yaw=self.start_yaw)
        path = os.path.join(self.legs_dir, leg_filename(self.from_node, self.to_node))
        write_leg(path, meta, self.samples)
        entry = IndexEntry(from_node=self.from_node, to_node=self.to_node,
                           file=os.path.basename(path), length_m=leg_length_m(self.samples),
                           duration_s=self.samples[-1].t, recorded=now, version=version,
                           samples=len(self.samples))
        entries = upsert_index_entry(load_index(self.legs_dir), entry)
        save_index(self.legs_dir, entries)
        self._leg_count = len(entries)
        self.get_logger().info(f'leg saved: {path} v{version}')
        return path

    # ------------------------------------------------------------------ 샘플링 (50 Hz)
    def _on_sample(self) -> None:
        if not self.recording or self.anchor is None:
            return
        pose = self._odom_pose()
        if pose is None:
            return
        t = time.monotonic() - self.t0
        xl, yl, thl = self.anchor.to_local(*pose)
        v = self._odom_speed()
        wz = self._wz_meas()
        st = self.segmenter.update(t, v, wz)
        self.seg_state = st
        if st.mixed_warning and not self._mixed_warned:
            self._mixed_warned = True
            self.set_status('경고: 병진+회전 동시 조작 — 거리 서명 기록 중단됨')
        sample = make_sample(t=t, seg=st.seg, seg_id=st.seg_id, x=xl, y=yl, th=thl, v=v, wz=wz,
                             enc=self.last_enc, yaw=self._imu_yaw(), raw=self._raw_sensors(),
                             dist_valid=st.dist_valid, cmd=[self.vx_cmd, self.vy_cmd, self.wz_cmd])
        if self._last_xy is not None:
            self.arc_len += math.hypot(xl - self._last_xy[0], yl - self._last_xy[1])
        self._last_xy = (xl, yl)
        self._last_sample = sample
        self.samples.append(sample)

    # ------------------------------------------------------------------ 키
    def on_key(self, ch: str) -> bool:
        if ch == '[':
            self._key_start(self.last_pair)
            return True
        if ch == ']':
            ok, msg, _ = self.stop_recording(save=True)
            self.set_status(msg)
            return True
        if ch == 'r':
            if self.last_pair is None:
                self.set_status('재기록할 레그 없음 — 먼저 [ 로 기록')
            else:
                self._key_start(self.last_pair, use_last=True)
            return True
        if ch == ESC:
            if not self.recording:
                self.set_status('기록 중이 아님')
                return True
            self.stop()   # 폐기 확인 전에 로봇부터 세운다
            ans = self.read_line('기록을 폐기할까요? (y/N) > ')
            if ans is not None and ans.strip().lower() == 'y':
                _, msg, _ = self.stop_recording(save=False)
                self.set_status(msg)
            else:
                self.set_status('폐기 취소 — 기록 계속')
            return True
        return super().on_key(ch)

    def _key_start(self, default_pair: Optional[tuple], use_last: bool = False) -> None:
        if self.recording:
            self.set_status('이미 기록 중 — ] 저장 또는 ESC 폐기')
            return
        if not self._is_stopped():
            self.set_status('정지 상태에서만 시작 가능 (space)')
            return
        if use_last and default_pair is not None:
            from_node, to_node = default_pair
        else:
            known = ', '.join(self._known_nodes()) or '(없음)'
            self.set_status(f'알려진 체크포인트: {known}')
            from_node = self._prompt_node('from', default_pair[0] if default_pair else '')
            if from_node is None:
                self.set_status('시작 취소')
                return
            to_node = self._prompt_node('to', default_pair[1] if default_pair else '')
            if to_node is None:
                self.set_status('시작 취소')
                return
        ok, msg = self.start_recording(from_node, to_node)
        self.set_status(msg)

    def _prompt_node(self, label: str, default: str) -> Optional[str]:
        hint = f' [{default}]' if default else ''
        ans = self.read_line(f'{label}{hint} > ')
        if ans is None:
            return None
        ans = ans.strip()
        return ans or default or None

    # ------------------------------------------------------------------ 서비스
    def _srv_start(self, req: RecordStart.Request, res: RecordStart.Response) -> RecordStart.Response:
        res.success, res.message = self.start_recording(req.from_node.strip(), req.to_node.strip())
        self.set_status(res.message)
        return res

    def _srv_stop(self, req: RecordStop.Request, res: RecordStop.Response) -> RecordStop.Response:
        res.success, res.message, res.path = self.stop_recording(bool(req.save))
        self.set_status(res.message)
        return res

    # ------------------------------------------------------------------ HUD
    def extra_hud_lines(self) -> List[str]:
        lines = ['-- teach: [ 시작  ] 저장  ESC 폐기  r 재기록 --']
        if self.recording:
            st = self.seg_state
            seg = f'{st.seg}{st.seg_id}' if st else '-'
            gate = 'OK' if (st and st.dist_valid) else '차단(회전)'
            s = self._last_sample
            ch_flags = ' '.join(
                f'{ch}:{"O" if (s and s.valid.get(ch)) else "x"}' for ch in CHANNELS) if s else '-'
            mixed = '  !! 병진+회전 동시' if (st and st.mixed_warning) else ''
            lines.append(f'REC  {self.from_node} → {self.to_node}   seg {seg}   호장 {self.arc_len:.2f} m'
                         f'   샘플 {len(self.samples)}{mixed}')
            lines.append(f'서명 게이트 {gate}   채널 {ch_flags}')
        else:
            last = f'{self.last_pair[0]} → {self.last_pair[1]}' if self.last_pair else '-'
            lines.append(f'대기  mission={self.mission}  마지막 레그 {last}  레그 수 {self._leg_count}')
            raw = self._raw_sensors()
            lines.append('센서 ' + ' '.join(
                f'{ch}:{(raw[ch] if raw[ch] is not None and math.isfinite(raw[ch]) else float("nan")):.2f}'
                for ch in CHANNELS))
        lines += RULES_LINES
        return lines


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachNode()
    try:
        node.run()
    finally:
        if node.recording:
            node.get_logger().warn('종료 시 기록 중이었음 — 저장하지 않고 폐기')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
