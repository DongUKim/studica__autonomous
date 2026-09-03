#!/usr/bin/env python3
"""키보드 텔레옵 노드 — /cmd_vel(geometry_msgs/Twist)을 래칭 방식으로 발행한다.

실행: ros2 run studica_teleop keyboard_teleop   (tty 필요 — ssh 접속 시 `ssh -t`)
확장: studica_repeat의 teach_node가 KeyboardTeleop을 상속해 on_key / extra_hud_lines /
      on_tick 을 덮어쓴다. 키 처리 순서는 서브클래스 on_key → 기본 키맵.

터미널 원시 입력은 키 "떼짐"을 알 수 없으므로 래칭(누르면 유지, space/x로 정지)이 기본이다.
hold_mode 파라미터를 켜면 key_timeout_s 동안 키가 없을 때 자동 정지한다.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from studica_control.srv import SetData

from studica_teleop.hud import Hud
from studica_teleop.terminal import ESC, KeyReader, RawTerminal
from studica_teleop.teleop_state import ANG_PRESETS, LIN_PRESETS, TeleopState

HUD_RATE_HZ = 10.0
KEY_POLL_S = 0.02
STATUS_TTL_S = 5.0          # 상태줄 메시지 유지 시간

HELP_LINES = [
    '  w/s 전진·후진  a/d 좌·우 이동  space/x 정지',
    '  Shift+Q / Shift+E 회전(반시계/시계)  l 회전 잠금 토글',
    '  1/2/3 속도 프리셋 저/중/고  +/- 속도 ±10%',
    '  z IMU yaw 영점 + 오도메트리 리셋  h 도움말  Ctrl-C 종료',
]


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KeyboardTeleop(Node):
    def __init__(self, node_name: str = 'keyboard_teleop'):
        super().__init__(node_name)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('hold_mode', False)
        self.declare_parameter('key_timeout_s', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_service', '/imu/get_imu_data')
        self.declare_parameter('reset_odom_service', '/base/reset_odom')

        self._publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._hold_mode = bool(self.get_parameter('hold_mode').value)
        self._key_timeout = float(self.get_parameter('key_timeout_s').value)

        self._state = TeleopState()
        self.last_odom: Optional[Odometry] = None
        self._status_text = ''
        self._status_t = 0.0
        self._hold_deadline: Optional[float] = None
        self._running = False

        self._cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 1)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self._imu_cli = self.create_client(SetData, self.get_parameter('imu_service').value)
        self._reset_cli = self.create_client(Trigger, self.get_parameter('reset_odom_service').value)

        self._hud = Hud(lines_fixed=8)
        self._reader = KeyReader()
        self._pub_timer = self.create_timer(1.0 / max(1.0, self._publish_rate), self._on_pub_timer)

    # ------------------------------------------------------------------ 계약 속성
    @property
    def vx_cmd(self) -> float:
        return self._state.vx

    @vx_cmd.setter
    def vx_cmd(self, v: float) -> None:
        self._state.vx = float(v)

    @property
    def vy_cmd(self) -> float:
        return self._state.vy

    @vy_cmd.setter
    def vy_cmd(self, v: float) -> None:
        self._state.vy = float(v)

    @property
    def wz_cmd(self) -> float:
        return self._state.wz

    @wz_cmd.setter
    def wz_cmd(self, v: float) -> None:
        self._state.wz = float(v)

    @property
    def lin_speed(self) -> float:
        return self._state.lin_speed

    @lin_speed.setter
    def lin_speed(self, v: float) -> None:
        self._state.set_speeds(float(v), self._state.ang_speed)

    @property
    def ang_speed(self) -> float:
        return self._state.ang_speed

    @ang_speed.setter
    def ang_speed(self, v: float) -> None:
        self._state.set_speeds(self._state.lin_speed, float(v))

    @property
    def rotation_locked(self) -> bool:
        return self._state.rotation_locked

    @rotation_locked.setter
    def rotation_locked(self, v: bool) -> None:
        self._state.rotation_locked = bool(v)

    # ------------------------------------------------------------------ 확장점
    def on_key(self, ch: str) -> bool:
        """서브클래스가 먼저 처리한다. True를 돌려주면 기본 키맵을 건너뛴다."""
        return False

    def extra_hud_lines(self) -> List[str]:
        return []

    def on_tick(self) -> None:
        """발행 타이머(publish_rate_hz)마다 호출."""

    # ------------------------------------------------------------------ 유틸
    def set_status(self, text: str) -> None:
        self._status_text = text
        self._status_t = time.monotonic()

    def stop(self) -> None:
        self._state.stop()

    def read_line(self, prompt: str) -> Optional[str]:
        """raw 모드 안에서 한 줄을 입력받는다. ESC/Ctrl-C면 None. HUD 아래 줄에 그린다."""
        buf: List[str] = []
        self._hud.write_below(prompt)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                ch = self._reader.read_key(KEY_POLL_S)
                if ch is None:
                    continue
                if ch == ESC or ch == '\x03':
                    return None
                if ch in ('\r', '\n'):
                    return ''.join(buf)
                if ch in ('\x7f', '\b'):
                    if buf:
                        buf.pop()
                elif ch.isprintable():
                    buf.append(ch)
                self._hud.write_below(prompt + ''.join(buf))
            return None
        except KeyboardInterrupt:
            return None
        finally:
            self._hud.clear_below()

    # ------------------------------------------------------------------ 내부
    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _on_pub_timer(self) -> None:
        if self._hold_mode and self._hold_deadline is not None and time.monotonic() > self._hold_deadline:
            self._state.stop()
            self._hold_deadline = None
        self.on_tick()
        self._publish_cmd()

    def _publish_cmd(self) -> None:
        msg = Twist()
        msg.linear.x = self._state.vx
        msg.linear.y = self._state.vy
        msg.angular.z = self._state.wz
        self._cmd_pub.publish(msg)

    def _dispatch_key(self, ch: str) -> None:
        if self.on_key(ch):
            return
        if ch == 'z':
            self._zero_yaw()
            return
        consumed, msg = self._state.press(ch)
        if consumed and self._hold_mode and TeleopState.is_motion_key(ch):
            self._hold_deadline = time.monotonic() + self._key_timeout
        if msg:
            self.set_status(msg)

    def _zero_yaw(self) -> None:
        # wait_for_service로 블로킹하지 않는다 — 준비 안 됐으면 상태줄에만 알린다
        if self._imu_cli.service_is_ready():
            req = SetData.Request()
            req.params = 'zero_yaw'
            fut = self._imu_cli.call_async(req)
            fut.add_done_callback(lambda f: self._on_srv_done('imu zero_yaw', f))
        else:
            self.set_status('IMU 서비스 없음 (/imu/get_imu_data)')
        if self._reset_cli.service_is_ready():
            fut = self._reset_cli.call_async(Trigger.Request())
            fut.add_done_callback(lambda f: self._on_srv_done('reset_odom', f))
        else:
            self.set_status('베이스 서비스 없음 (/base/reset_odom)')

    def _on_srv_done(self, name: str, fut) -> None:
        try:
            res = fut.result()
        except Exception as e:  # 서비스 예외는 상태줄로 노출
            self.set_status(f'{name} 실패: {e}')
            return
        if res is None:
            self.set_status(f'{name}: 응답 없음')
        elif getattr(res, 'success', False):
            self.set_status(f'{name}: {res.message}')
        else:
            self.set_status(f'{name} 실패: {res.message}')

    def _hud_lines(self) -> List[str]:
        s = self._state
        lock = '잠금' if s.rotation_locked else '해제'
        lines = [
            f'== {self.get_name()} ==  프리셋 {s.preset + 1} '
            f'(lin {s.lin_speed:.2f} m/s, ang {s.ang_speed:.2f} rad/s)  회전 {lock}',
            f'목표  vx {s.vx:+.2f}  vy {s.vy:+.2f}  wz {s.wz:+.2f}',
        ]
        if self.last_odom is not None:
            p = self.last_odom.pose.pose
            yaw = math.degrees(_yaw_from_quat(p.orientation))
            lines.append(f'odom  x {p.position.x:+.3f}  y {p.position.y:+.3f}  yaw {yaw:+.1f} deg')
        else:
            lines.append('odom  (수신 없음)')
        if time.monotonic() - self._status_t < STATUS_TTL_S:
            lines.append('상태  ' + self._status_text)
        else:
            lines.append('상태  -')
        if s.help_visible:
            lines += HELP_LINES
        else:
            lines.append('  h 도움말')
        lines += self.extra_hud_lines()
        return lines

    def run(self) -> None:
        self._running = True
        next_hud = 0.0
        try:
            with RawTerminal():
                while rclpy.ok() and self._running:
                    rclpy.spin_once(self, timeout_sec=0)
                    ch = self._reader.read_key(KEY_POLL_S)
                    if ch is not None:
                        self._dispatch_key(ch)
                    now = time.monotonic()
                    if now >= next_hud:
                        self._hud.render(self._hud_lines())
                        next_hud = now + 1.0 / HUD_RATE_HZ
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown_motion()

    def _shutdown_motion(self) -> None:
        """종료 시 정지 명령을 여러 번 보내 베이스가 확실히 받게 한다."""
        self.stop()
        try:
            for _ in range(3):
                self._publish_cmd()
                rclpy.spin_once(self, timeout_sec=0.02)
        except Exception:
            pass
        print()

    def request_exit(self) -> None:
        self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
