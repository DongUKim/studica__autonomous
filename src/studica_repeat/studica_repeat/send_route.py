#!/usr/bin/env python3
"""액션 클라이언트 CLI — 재생기에 경로/상대이동 골을 보낸다.

  ros2 run studica_repeat send_route N1 N2 N3 [--speed 1.0]
  ros2 run studica_repeat send_route --relative 0.2 -0.1 [--speed 0.15]

피드백을 한 줄씩 출력하고, 결과 성공이면 종료 코드 0, 아니면 1.
"""
from __future__ import annotations

import argparse
import sys
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from studica_interfaces.action import FollowRoute, RelativeMove

SERVER_WAIT_S = 10.0


def _parse(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog='send_route', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('nodes', nargs='*', help='체크포인트 열 (FollowRoute)')
    ap.add_argument('--relative', nargs=2, type=float, metavar=('DX', 'DY'),
                    help='RelativeMove [m], 로봇 프레임 (전방 +, 좌측 +), 합성 0.3 m 이내')
    ap.add_argument('--speed', type=float, default=0.0,
                    help='FollowRoute: speed_scale(기본 1.0) / RelativeMove: m/s(기본 0.15)')
    ns = ap.parse_args(argv)
    if ns.relative is None and len(ns.nodes) < 2:
        ap.error('체크포인트 2개 이상 또는 --relative DX DY 가 필요하다')
    return ns


class SendRoute(Node):
    def __init__(self) -> None:
        super().__init__('send_route')

    def _send(self, client: ActionClient, goal, on_feedback):
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            print(f'액션 서버 없음: {client._action_name}', file=sys.stderr)
            return None
        fut = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print('골 거절됨 (다른 잡 실행 중?)', file=sys.stderr)
            return None
        res_fut = gh.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, res_fut)
        except KeyboardInterrupt:
            print('취소 요청', file=sys.stderr)
            cancel = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=2.0)
            return None
        return res_fut.result().result

    def follow(self, nodes: List[str], speed_scale: float) -> int:
        goal = FollowRoute.Goal()
        goal.nodes = nodes
        goal.speed_scale = float(speed_scale)

        def fb(msg):
            f = msg.feedback
            print(f'{f.leg:<16} {f.segment:<4} s {f.s:6.3f}/{f.s_end:6.3f}  '
                  f'e_lat {f.e_lat:+.3f}  e_th {f.e_th:+.3f}')

        res = self._send(ActionClient(self, FollowRoute, '/nav/follow_route'), goal, fb)
        if res is None:
            return 1
        print(f'result: success={res.success} reached={res.reached_node!r} message={res.message!r}')
        return 0 if res.success else 1

    def relative(self, dx: float, dy: float, speed: float) -> int:
        goal = RelativeMove.Goal()
        goal.dx, goal.dy, goal.speed = float(dx), float(dy), float(speed)

        def fb(msg):
            print(f'remaining {msg.feedback.remaining:.3f} m')

        res = self._send(ActionClient(self, RelativeMove, '/nav/relative_move'), goal, fb)
        if res is None:
            return 1
        print(f'result: success={res.success} message={res.message!r}')
        return 0 if res.success else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    ns = _parse(remove_ros_args(sys.argv)[1:])
    node = SendRoute()
    code = 1
    try:
        if ns.relative is not None:
            code = node.relative(ns.relative[0], ns.relative[1], ns.speed)
        else:
            code = node.follow(ns.nodes, ns.speed)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
