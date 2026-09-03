#!/usr/bin/env python3
"""재생 제어기 노드 (repeat 모드) — 기록된 레그를 50 Hz 폐루프로 추종한다 (플랜B §4, §5).

액션: /nav/follow_route (FollowRoute) — 체크포인트 열을 받아 index.yaml의 엣지 레그를 순차 재생.
      /nav/relative_move (RelativeMove) — 0.3 m 이내 데드레코닝 이동.
발행: /cmd_vel (Twist), /repeat/tracking (String, JSON 1행/틱; validate_node가 소비)

스레딩: 제어 타이머·구독은 한 콜백 그룹(상호배제)에서 돌고, 액션 execute 콜백은 별도(재진입)
그룹에서 Event를 기다리기만 한다. 제어 상태는 타이머 스레드만 만진다 — execute 쪽은 잡(job)을
넣고 결과를 읽을 뿐이다. sleep 기반 제어는 없다(사용자 요구).
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String

from studica_interfaces.action import FollowRoute, RelativeMove
from studica_repeat.core.anchor import Anchor, wrap
from studica_repeat.core.controller import RepeatController, RepeatGains, StepResult
from studica_repeat.core.graph import LegGraph
from studica_repeat.core.leg_io import CHANNELS, IndexEntry, gate_channel, load_index, read_leg
from studica_repeat.core.path import compile_leg
from studica_repeat.core.relative_move import RelativeMovePlanner

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
SENSOR_STALE_S = 0.3
FEEDBACK_DIV = 5            # 50 Hz / 5 = 10 Hz 피드백
JOB_WAIT_S = 0.1            # execute 콜백이 취소 요청을 확인하는 주기
ODOM_STALE_S = 0.5
# RelativeMove 중 헤딩 유지 게인 (짧은 데드레코닝이라 약하게)
REL_K_TH = 2.0
REL_WZ_MAX = 0.6


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class Job:
    kind: str                       # 'follow' | 'relative'
    goal_handle: object
    finished: threading.Event = field(default_factory=threading.Event)
    cancel_requested: bool = False
    cancelled: bool = False
    success: bool = False
    message: str = ''
    t_start: float = 0.0
    tick: int = 0
    # follow 전용
    nodes: List[str] = field(default_factory=list)
    entries: List[IndexEntry] = field(default_factory=list)
    leg_idx: int = -1
    controller: Optional[RepeatController] = None
    anchor: Optional[Anchor] = None
    yaw_at_anchor: float = 0.0
    reached_node: str = ''
    speed_scale: float = 1.0
    # relative 전용
    planner: Optional[RelativeMovePlanner] = None


class RepeatNode(Node):
    def __init__(self) -> None:
        super().__init__('repeat_node')
        d = self.declare_parameter
        d('mission', 'mission_a')
        d('missions_dir', '~/studica_missions')
        d('control_rate_hz', 50.0)
        d('cmd_vel_topic', '/cmd_vel')
        d('odom_topic', '/odom')
        d('imu_topic', '/imu')
        d('publish_tracking', True)
        for ch, pname in RANGE_PARAMS.items():
            d(pname, RANGE_DEFAULTS[ch])
        g = RepeatGains()
        d('k_lat', g.k_lat)
        d('k_th', g.k_th)
        d('k_yaw', g.k_yaw)
        d('lookahead', g.lookahead)
        d('vmax', g.vmax)
        d('vmin', g.vmin)
        d('decel', g.decel)
        d('end_tol', g.end_tol)
        d('dev_lat', g.dev_lat)
        d('wz_max', g.wz_max)
        d('sig_k', g.sig_k)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.mission = str(p('mission'))
        self.legs_dir = os.path.join(os.path.expanduser(str(p('missions_dir'))), self.mission, 'taught_legs')
        self.gains = RepeatGains(
            k_lat=float(p('k_lat')), k_th=float(p('k_th')), k_yaw=float(p('k_yaw')),
            lookahead=float(p('lookahead')), vmax=float(p('vmax')), vmin=float(p('vmin')),
            decel=float(p('decel')), end_tol=float(p('end_tol')), dev_lat=float(p('dev_lat')),
            wz_max=float(p('wz_max')), sig_k=float(p('sig_k')))
        self.publish_tracking = bool(p('publish_tracking'))

        self.ctrl_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        self._job_lock = threading.Lock()
        self._job: Optional[Job] = None

        self.last_odom: Optional[Odometry] = None
        self.last_odom_t = 0.0
        self.last_imu: Optional[Imu] = None
        self.last_range: Dict[str, float] = {}
        self.last_range_t: Dict[str, float] = {}

        self.create_subscription(Odometry, str(p('odom_topic')), self._on_odom, 10,
                                 callback_group=self.ctrl_group)
        self.create_subscription(Imu, str(p('imu_topic')), self._on_imu, 10,
                                 callback_group=self.ctrl_group)
        for ch, pname in RANGE_PARAMS.items():
            self.create_subscription(Range, str(p(pname)), lambda msg, ch=ch: self._on_range(ch, msg), 10,
                                     callback_group=self.ctrl_group)
        self.cmd_pub = self.create_publisher(Twist, str(p('cmd_vel_topic')), 1)
        self.track_pub = self.create_publisher(String, '/repeat/tracking', 10)

        self.follow_server = ActionServer(
            self, FollowRoute, '/nav/follow_route', execute_callback=self._exec_follow,
            goal_callback=self._goal_cb, cancel_callback=self._cancel_cb,
            callback_group=self.action_group)
        self.relative_server = ActionServer(
            self, RelativeMove, '/nav/relative_move', execute_callback=self._exec_relative,
            goal_callback=self._goal_cb, cancel_callback=self._cancel_cb,
            callback_group=self.action_group)

        self.dt = 1.0 / max(1.0, float(p('control_rate_hz')))
        self.create_timer(self.dt, self._tick, callback_group=self.ctrl_group)
        self.get_logger().info(f'repeat_node: mission={self.mission} legs_dir={self.legs_dir} '
                               f'legs={len(load_index(self.legs_dir))}')

    # ------------------------------------------------------------------ 구독
    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_t = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        self.last_imu = msg

    def _on_range(self, ch: str, msg: Range) -> None:
        self.last_range[ch] = float(msg.range)
        self.last_range_t[ch] = time.monotonic()

    def _pose(self) -> Optional[tuple]:
        if self.last_odom is None or time.monotonic() - self.last_odom_t > ODOM_STALE_S:
            return None
        pp = self.last_odom.pose.pose
        return (pp.position.x, pp.position.y, _yaw_from_quat(pp.orientation))

    def _imu_yaw(self, fallback: float) -> float:
        return _yaw_from_quat(self.last_imu.orientation) if self.last_imu is not None else fallback

    def _wz_meas(self) -> float:
        if self.last_imu is not None:
            return float(self.last_imu.angular_velocity.z)
        return float(self.last_odom.twist.twist.angular.z) if self.last_odom is not None else 0.0

    def _sensors(self) -> Dict[str, Optional[float]]:
        now = time.monotonic()
        out: Dict[str, Optional[float]] = {}
        for ch in CHANNELS:
            t = self.last_range_t.get(ch)
            raw = self.last_range[ch] if (t is not None and now - t < SENSOR_STALE_S) else None
            out[ch] = gate_channel(ch, raw)
        return out

    # ------------------------------------------------------------------ 액션 공통
    def _goal_cb(self, _goal_request) -> GoalResponse:
        with self._job_lock:
            if self._job is not None:
                self.get_logger().warn('goal rejected: another job is active')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _run_job(self, job: Job) -> Job:
        """잡을 타이머에 넘기고 끝날 때까지 기다린다(execute 스레드)."""
        with self._job_lock:
            if self._job is not None:
                job.success = False
                job.message = 'busy'
                job.finished.set()
                return job
            job.t_start = time.monotonic()
            self._job = job
        while not job.finished.wait(JOB_WAIT_S):
            if job.goal_handle.is_cancel_requested:
                job.cancel_requested = True
        return job

    def _finish_job(self, job: Job, success: bool, message: str, cancelled: bool = False) -> None:
        job.success, job.message, job.cancelled = success, message, cancelled
        self._publish_cmd(0.0, 0.0, 0.0)
        with self._job_lock:
            if self._job is job:
                self._job = None
        job.finished.set()
        (self.get_logger().info if success else self.get_logger().warn)(f'{job.kind}: {message}')

    # ------------------------------------------------------------------ FollowRoute
    def _exec_follow(self, goal_handle):
        req = goal_handle.request
        nodes = [str(n) for n in req.nodes]
        result = FollowRoute.Result()
        if len(nodes) < 2:
            result.success, result.message = False, 'need at least 2 nodes'
            goal_handle.abort()
            return result
        entries = load_index(self.legs_dir)
        graph = LegGraph(entries)
        missing = graph.missing_edges(nodes)
        if missing:
            a, b = missing[0]
            result.success, result.message = False, f'untaught edge {a}->{b}'
            goal_handle.abort()
            return result
        job = Job(kind='follow', goal_handle=goal_handle, nodes=nodes,
                  entries=[graph.find(a, b) for a, b in zip(nodes, nodes[1:])],
                  speed_scale=float(req.speed_scale) if req.speed_scale > 0 else 1.0)
        job = self._run_job(job)
        result.success, result.message, result.reached_node = job.success, job.message, job.reached_node
        if job.cancelled:
            goal_handle.canceled()
        elif job.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _start_leg(self, job: Job) -> bool:
        """다음 레그를 현재 pose에 앵커링해 제어기를 만든다. 실패하면 잡을 종료하고 False."""
        job.leg_idx += 1
        if job.leg_idx >= len(job.entries):
            job.reached_node = job.nodes[-1]
            self._finish_job(job, True, f'route done at {job.reached_node}')
            return False
        entry = job.entries[job.leg_idx]
        pose = self._pose()
        if pose is None:
            self._finish_job(job, False, 'odom not available')
            return False
        try:
            _meta, samples = read_leg(os.path.join(self.legs_dir, entry.file))
            segments = compile_leg(samples)
        except Exception as e:
            self._finish_job(job, False, f'leg {entry.name} load failed: {e}')
            return False
        job.anchor = Anchor.from_pose(*pose)
        job.yaw_at_anchor = self._imu_yaw(pose[2])
        job.controller = RepeatController(segments, self.gains, job.speed_scale)
        self.get_logger().info(f'leg {entry.name}: {len(segments)} segments, anchored at '
                               f'({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} deg)')
        if job.controller.done:
            self.get_logger().warn(f'leg {entry.name}: no segments — skipping')
            job.reached_node = entry.to_node
            return self._start_leg(job)
        return True

    def _tick_follow(self, job: Job) -> None:
        if job.controller is None and not self._start_leg(job):
            return
        pose = self._pose()
        if pose is None:
            self._finish_job(job, False, 'odom lost')
            return
        assert job.anchor is not None and job.controller is not None
        xl, yl, thl = job.anchor.to_local(*pose)
        yaw_local = wrap(self._imu_yaw(pose[2]) - job.yaw_at_anchor)
        t = time.monotonic() - job.t_start
        r = job.controller.step(t, xl, yl, thl, yaw_local, self._wz_meas(), self._sensors())
        entry = job.entries[job.leg_idx]
        self._publish_tracking(t, entry.name, r)
        if r.aborted:
            self._finish_job(job, False, f'leg {entry.name} aborted: {r.reason}')
            return
        if r.done:
            job.reached_node = entry.to_node
            self._publish_cmd(0.0, 0.0, 0.0)
            job.controller = None          # 다음 틱에 새 레그를 현재 pose에 재앵커
            return
        self._publish_cmd(r.vx, r.vy, r.wz)
        if job.tick % FEEDBACK_DIV == 0:
            fb = FollowRoute.Feedback()
            fb.leg, fb.segment = entry.name, r.seg_label
            fb.s, fb.s_end, fb.e_lat, fb.e_th = float(r.s), float(r.s_end), float(r.e_lat), float(r.e_th)
            job.goal_handle.publish_feedback(fb)

    # ------------------------------------------------------------------ RelativeMove
    def _exec_relative(self, goal_handle):
        req = goal_handle.request
        result = RelativeMove.Result()
        try:
            planner = RelativeMovePlanner(float(req.dx), float(req.dy),
                                          speed=float(req.speed) if req.speed > 0 else 0.15)
        except ValueError as e:
            result.success, result.message = False, str(e)
            goal_handle.abort()
            return result
        job = self._run_job(Job(kind='relative', goal_handle=goal_handle, planner=planner))
        result.success, result.message = job.success, job.message
        if job.cancelled:
            goal_handle.canceled()
        elif job.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _tick_relative(self, job: Job) -> None:
        assert job.planner is not None
        pose = self._pose()
        if pose is None:
            self._finish_job(job, False, 'odom not available')
            return
        if job.anchor is None:
            job.anchor = Anchor.from_pose(*pose)
        xl, yl, thl = job.anchor.to_local(*pose)
        vx, vy, done = job.planner.step(time.monotonic() - job.t_start, xl, yl)
        if done:
            self._finish_job(job, True, f'relative move done ({job.planner.dist:.3f} m)')
            return
        wz = max(-REL_WZ_MAX, min(REL_WZ_MAX, REL_K_TH * wrap(-thl)))
        self._publish_cmd(vx, vy, wz)
        if job.tick % FEEDBACK_DIV == 0:
            fb = RelativeMove.Feedback()
            along = xl * job.planner.ux + yl * job.planner.uy
            fb.remaining = float(max(0.0, job.planner.dist - along))
            job.goal_handle.publish_feedback(fb)

    # ------------------------------------------------------------------ 타이머
    def _tick(self) -> None:
        job = self._job
        if job is None:
            return
        job.tick += 1
        if job.cancel_requested:
            self._finish_job(job, False, 'cancelled', cancelled=True)
            return
        if job.kind == 'follow':
            self._tick_follow(job)
        else:
            self._tick_relative(job)

    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(msg)

    def _publish_tracking(self, t: float, leg: str, r: StepResult) -> None:
        if not self.publish_tracking:
            return
        row = {'t': round(t, 3), 'leg': leg, 'seg': r.seg_label, 's': round(r.s, 4),
               's_end': round(r.s_end, 4), 'e_lat': round(r.e_lat, 4), 'e_th': round(r.e_th, 4),
               'lat_bias': round(r.lat_bias, 4), 's_bias': round(r.s_bias, 4),
               'vx': round(r.vx, 3), 'vy': round(r.vy, 3), 'wz': round(r.wz, 3),
               'done': r.done, 'aborted': r.aborted}
        self.track_pub.publish(String(data=json.dumps(row)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RepeatNode()
    executor = MultiThreadedExecutor(num_threads=4)  # execute 대기 1 + 타이머 1 + goal/cancel 처리 여유
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
