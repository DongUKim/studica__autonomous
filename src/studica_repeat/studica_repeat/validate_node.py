#!/usr/bin/env python3
"""검증 모드 (--mode validate_repeat 상당) — 레그를 연속 재생하며 tracking.csv + 리포트를 만든다 (플랜B §6).

repeat_node가 떠 있어야 한다. 레그마다 2노드 FollowRoute 골을 보내고, /repeat/tracking(50 Hz)의
행을 모아 e_lat p95 / 종점 오차 / 서명 보정량 / 소요시간을 집계한다. 연속 재생은 "레그 종점 =
다음 레그 시작점"(운전 수칙 3)을 전제한다 — 순서가 이어지지 않는 레그 목록이면 사람이 로봇을
옮겨 놓아야 하며, 그 경우 legs 파라미터로 이어지는 부분집합만 지정할 것.

실행: ros2 run studica_repeat validate_node --ros-args -p mission:=mission_a -p legs:=all
종료 코드: 전 레그 합격 0, 아니면 1.
"""
from __future__ import annotations

import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from studica_interfaces.action import FollowRoute
from studica_repeat.core.leg_io import IndexEntry, load_index
from studica_repeat.core.report import TrackingRow, format_report, summarize, write_tracking_csv

SERVER_WAIT_S = 10.0
RESULT_TIMEOUT_MARGIN = 3.0     # 티칭 소요의 3배 + 10 s 를 넘기면 실패로 본다
RESULT_TIMEOUT_BASE_S = 10.0


class ValidateNode(Node):
    def __init__(self) -> None:
        super().__init__('validate_node')
        d = self.declare_parameter
        d('mission', 'mission_a')
        d('missions_dir', '~/studica_missions')
        d('legs', 'all')
        d('out_dir', '')
        d('speed_scale', 1.0)
        d('time_budget_s', 0.0)
        d('line_tracer', False)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.mission = str(p('mission'))
        mission_dir = os.path.join(os.path.expanduser(str(p('missions_dir'))), self.mission)
        self.legs_dir = os.path.join(mission_dir, 'taught_legs')
        self.out_dir = str(p('out_dir')) or os.path.join(mission_dir, 'validation')
        self.legs_spec = str(p('legs'))
        self.speed_scale = float(p('speed_scale'))
        budget = float(p('time_budget_s'))
        self.time_budget = budget if budget > 0 else None
        self.line_tracer = bool(p('line_tracer'))

        self.rows: List[TrackingRow] = []
        self.last_row: Dict[str, dict] = {}
        self._t_offset = 0.0
        self._current_leg = ''
        self.create_subscription(String, '/repeat/tracking', self._on_tracking, 50)
        self.client = ActionClient(self, FollowRoute, '/nav/follow_route')

    # ------------------------------------------------------------------ 레그 선택
    def select_legs(self) -> List[IndexEntry]:
        entries = load_index(self.legs_dir)
        if not entries:
            raise RuntimeError(f'{self.legs_dir}: index.yaml에 레그가 없다')
        if self.legs_spec.strip().lower() == 'all':
            return entries
        by_name = {e.name: e for e in entries}
        chosen: List[IndexEntry] = []
        for name in (n.strip() for n in self.legs_spec.split(',') if n.strip()):
            if name not in by_name:
                raise RuntimeError(f'레그 {name} 이 index에 없다 (있는 레그: {", ".join(by_name)})')
            chosen.append(by_name[name])
        return chosen

    # ------------------------------------------------------------------ 트래킹 수집
    def _on_tracking(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get('leg') != self._current_leg:
            return
        self.rows.append(TrackingRow(t=float(d['t']) + self._t_offset, leg=str(d['leg']), seg=str(d['seg']),
                                     s=float(d['s']), e_lat=float(d['e_lat']), e_th=float(d['e_th']),
                                     lat_bias=float(d['lat_bias']), s_bias=float(d['s_bias'])))
        if str(d['seg']).startswith('T'):
            self.last_row[str(d['leg'])] = d

    # ------------------------------------------------------------------ 레그 1개 재생
    def run_leg(self, entry: IndexEntry) -> Tuple[bool, str, float]:
        """(성공, 메시지, 소요 s)."""
        goal = FollowRoute.Goal()
        goal.nodes = [entry.from_node, entry.to_node]
        goal.speed_scale = self.speed_scale
        self._current_leg = entry.name
        t0 = time.monotonic()
        send_fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=SERVER_WAIT_S)
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return False, 'goal rejected', 0.0
        res_fut = gh.get_result_async()
        timeout = RESULT_TIMEOUT_BASE_S + RESULT_TIMEOUT_MARGIN * max(entry.duration_s, 1.0) / max(self.speed_scale, 0.1)
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        elapsed = time.monotonic() - t0
        if not res_fut.done():
            gh.cancel_goal_async()
            return False, f'timeout after {elapsed:.1f} s', elapsed
        result = res_fut.result().result
        self._t_offset += elapsed
        return bool(result.success), str(result.message), elapsed

    def end_error(self, leg: str) -> Optional[float]:
        d = self.last_row.get(leg)
        if d is None:
            return None
        remain = max(0.0, float(d['s_end']) - float(d['s']))
        return (remain ** 2 + float(d['e_lat']) ** 2) ** 0.5

    # ------------------------------------------------------------------ 전체
    def run(self) -> int:
        legs = self.select_legs()
        self.get_logger().info(f'validate: {len(legs)} legs, speed_scale {self.speed_scale}')
        if not self.client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            self.get_logger().error('/nav/follow_route 서버 없음 — repeat.launch.py 가 떠 있는지 확인')
            return 2
        end_errors: Dict[str, float] = {}
        durations: Dict[str, Tuple[float, float]] = {}
        failures: List[str] = []
        for e in legs:
            ok, msg, elapsed = self.run_leg(e)
            self.get_logger().info(f'{e.name}: {"OK" if ok else "FAIL"} {msg} ({elapsed:.1f} s / taught {e.duration_s:.1f} s)')
            durations[e.name] = (elapsed, e.duration_s)
            err = self.end_error(e.name)
            if err is not None:
                end_errors[e.name] = err
            if not ok:
                failures.append(f'{e.name}: {msg}')
                break   # 실패한 레그 뒤는 시작점이 보장되지 않는다
        os.makedirs(self.out_dir, exist_ok=True)
        write_tracking_csv(os.path.join(self.out_dir, 'tracking.csv'), self.rows)
        summary = summarize(self.rows, end_errors, durations, self.time_budget, self.line_tracer)
        text = format_report(summary)
        if failures:
            text += '\n실패 레그:\n' + '\n'.join('  ' + f for f in failures) + '\n'
        with open(os.path.join(self.out_dir, 'report.txt'), 'w', encoding='utf-8') as f:
            f.write(text)
        self.get_logger().info('\n' + text)
        self.get_logger().info(f'written: {self.out_dir}/tracking.csv, report.txt')
        return 0 if (summary.get('all_pass') and not failures) else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidateNode()
    code = 1
    try:
        code = node.run()
    except (KeyboardInterrupt, RuntimeError) as e:
        if not isinstance(e, KeyboardInterrupt):
            node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
