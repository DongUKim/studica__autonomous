#!/usr/bin/env python3
"""studica_base base_node — cmd_vel → Titan 3륜 옴니 모터, 데드레코닝 오도메트리, EMS 가드.

단일 50 Hz 타이머만 사용한다(스레드·sleep 없음). 구독 콜백은 값 저장만 하고
모든 계산·발행은 타이머에서 한다 — 구코드의 다중 스레드 race 사고를 원천 회피.

토픽
  구독  /cmd_vel (geometry_msgs/Twist)
        /titan0/m_{0..3}/encoder (std_msgs/Float64, m)
        /imu (sensor_msgs/Imu)
        ems_topic (std_msgs/Bool)
  발행  /titan0/m_{0..2}/cmd (std_msgs/Float64, 듀티)
        /odom (nav_msgs/Odometry)
        /base/encoders (Float64MultiArray [m0..m3] m)
        /base/wheel_vel (Float64MultiArray [3] m/s)
        /base/ems (std_msgs/Bool)
서비스 /base/reset_odom (std_srvs/Trigger)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray, Bool
from std_srvs.srv import Trigger

from studica_base.omni_kinematics import (
    inverse_kinematics, forward_kinematics, wrap_angle,
)
from studica_base.wheel_controller import slew, ff_p_duty, clamp

NUM_TITAN_MOTORS = 4
DRIVE_MOTORS = (0, 1, 2)
# 헤딩 홀드 진입 판정 — 램프 후 wz 명령이 사실상 0인지
HOLD_ENGAGE_EPS = 1e-3


def yaw_from_quaternion(q) -> float:
    # tf_transformations 의존을 피하기 위한 직접 계산 (Z축 yaw만 필요)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _finite(v: float) -> bool:
    return v == v and not math.isinf(v)


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')
        self._declare_params()
        self._read_params()

        # --- 명령 상태 ---
        self.req_vx = 0.0
        self.req_vy = 0.0
        self.req_wz = 0.0
        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.cur_wz = 0.0
        self.last_cmd_time = None
        self.watchdog_tripped = False
        self.hold_active = False
        self.hold_target = 0.0

        # --- 엔코더 상태 (m) ---
        self.enc = [float('nan')] * NUM_TITAN_MOTORS
        self.prev_enc = [float('nan')] * NUM_TITAN_MOTORS
        self.wheel_vel = [0.0, 0.0, 0.0]
        self.vel_win_t = None
        self.vel_win_enc = [float('nan')] * 3

        # --- IMU 상태 ---
        self.imu_yaw = None          # 최신 IMU yaw (rad, 절대)
        self.imu_prev_yaw = None
        self.imu_wz = 0.0
        self.imu_warned = False
        self.imu_yaw_ref = 0.0       # reset_odom 시점의 IMU yaw

        # --- 오도메트리 ---
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0           # 연속각 (랩 없음)
        self.meas_vx = 0.0
        self.meas_vy = 0.0
        self.meas_wz = 0.0

        # --- EMS ---
        self.ems_pressed = False
        self.ems_raw = None
        self.last_ems_log_t = 0.0

        self._setup_io()
        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self.on_tick)
        self.get_logger().info(
            'base_node ready: titan=%s R=%.3f m kS=%.3f kV=%.3f kP=%.3f heading_hold=%s ems=%s(%s==%s)'
            % (self.titan_name, self.wheel_base_r, self.k_s, self.k_v, self.k_p,
               self.heading_hold, self.ems_enabled, self.ems_topic, self.ems_pressed_value))

    # ------------------------------------------------------------------ params
    def _declare_params(self):
        d = self.declare_parameter
        d('titan_name', 'titan0')
        d('wheel_base_r_m', 0.120)
        d('enc_scale', 1.0)
        d('k_s', 0.05)
        d('k_v', 1.1)
        d('k_p', 0.4)
        d('max_duty', 1.0)
        d('max_lin_mps', 0.5)
        d('max_ang_radps', 1.5)
        d('slew_lin_mps2', 0.8)
        d('slew_ang_radps2', 6.28)
        d('cmd_timeout_s', 0.3)
        d('control_rate_hz', 50)
        d('vel_window_s', 0.05)
        d('use_imu_yaw', True)
        d('imu_topic', '/imu')
        d('cmd_vel_topic', '/cmd_vel')
        d('odom_topic', '/odom')
        d('odom_frame', 'odom')
        d('base_frame', 'base_link')
        d('publish_tf', False)
        d('heading_hold', True)
        d('hold_kp', 3.0)
        d('hold_w_max', 1.0)
        d('ems_enabled', True)
        d('ems_topic', '/titan0/m_1/limit_rev')
        d('ems_pressed_value', False)

    def _read_params(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.titan_name = g('titan_name')
        self.wheel_base_r = float(g('wheel_base_r_m'))
        self.enc_scale = float(g('enc_scale'))
        self.k_s = float(g('k_s'))
        self.k_v = float(g('k_v'))
        self.k_p = float(g('k_p'))
        self.max_duty = float(g('max_duty'))
        self.max_lin = float(g('max_lin_mps'))
        self.max_ang = float(g('max_ang_radps'))
        self.slew_lin = float(g('slew_lin_mps2'))
        self.slew_ang = float(g('slew_ang_radps2'))
        self.cmd_timeout = float(g('cmd_timeout_s'))
        self.control_rate_hz = int(g('control_rate_hz'))
        self.vel_window = float(g('vel_window_s'))
        self.use_imu_yaw = bool(g('use_imu_yaw'))
        self.imu_topic = g('imu_topic')
        self.cmd_vel_topic = g('cmd_vel_topic')
        self.odom_topic = g('odom_topic')
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.publish_tf = bool(g('publish_tf'))
        self.heading_hold = bool(g('heading_hold'))
        self.hold_kp = float(g('hold_kp'))
        self.hold_w_max = float(g('hold_w_max'))
        self.ems_enabled = bool(g('ems_enabled'))
        self.ems_topic = g('ems_topic')
        self.ems_pressed_value = bool(g('ems_pressed_value'))
        if self.wheel_base_r <= 0.0 or self.control_rate_hz <= 0:
            raise ValueError('wheel_base_r_m and control_rate_hz must be positive')

    # ------------------------------------------------------------------ io
    def _setup_io(self):
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 1)
        for i in range(NUM_TITAN_MOTORS):
            self.create_subscription(
                Float64, '/%s/m_%d/encoder' % (self.titan_name, i),
                lambda msg, i=i: self.on_encoder(i, msg), 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, sensor_qos)
        if self.ems_enabled:
            self.create_subscription(Bool, self.ems_topic, self.on_ems, 10)

        self.cmd_pubs = [
            self.create_publisher(Float64, '/%s/m_%d/cmd' % (self.titan_name, i), 1)
            for i in DRIVE_MOTORS
        ]
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.enc_pub = self.create_publisher(Float64MultiArray, '/base/encoders', 10)
        self.wheel_vel_pub = self.create_publisher(Float64MultiArray, '/base/wheel_vel', 10)
        self.ems_pub = self.create_publisher(Bool, '/base/ems', 10)
        self.create_service(Trigger, '/base/reset_odom', self.on_reset_odom)

        self.tf_broadcaster = None
        if self.publish_tf:
            # tf2_ros가 없는 환경에서도 노드는 떠야 하므로 필요할 때만 임포트
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)

    # ------------------------------------------------------------------ callbacks (저장만)
    def on_cmd_vel(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if not (_finite(vx) and _finite(vy) and _finite(wz)):
            self.get_logger().warn('cmd_vel with NaN/inf ignored')
            return
        self.req_vx = vx
        self.req_vy = vy
        self.req_wz = wz
        self.last_cmd_time = self.now_s()

    def on_encoder(self, i: int, msg: Float64):
        v = msg.data
        if not _finite(v):
            return
        self.enc[i] = v * self.enc_scale

    def on_imu(self, msg: Imu):
        yaw = yaw_from_quaternion(msg.orientation)
        if not _finite(yaw):
            return
        self.imu_yaw = yaw
        self.imu_wz = msg.angular_velocity.z if _finite(msg.angular_velocity.z) else 0.0

    def on_ems(self, msg: Bool):
        self.ems_raw = msg.data
        pressed = (msg.data == self.ems_pressed_value)
        if pressed != self.ems_pressed:
            self.ems_pressed = pressed
            self.get_logger().warn('EMS %s' % ('ENGAGED — motors forced to 0' if pressed else 'released'))
            self.ems_pub.publish(Bool(data=pressed))

    def on_reset_odom(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        if self.imu_yaw is not None:
            self.imu_yaw_ref = self.imu_yaw
            self.imu_prev_yaw = self.imu_yaw
        self.hold_target = 0.0
        self.hold_active = False
        response.success = True
        response.message = 'odom reset (x=y=heading=0, imu ref=%.3f)' % self.imu_yaw_ref
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ tick
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tick(self):
        t = self.now_s()
        self._update_odometry(t)
        self._update_command(t)
        self._publish_state(t)

    def _update_odometry(self, t: float):
        # 첫 샘플은 prev 초기화만 (적분하면 0에서 현재값까지 점프)
        for i in range(NUM_TITAN_MOTORS):
            if not _finite(self.prev_enc[i]) and _finite(self.enc[i]):
                self.prev_enc[i] = self.enc[i]
        d = []
        for i in DRIVE_MOTORS:
            if _finite(self.enc[i]) and _finite(self.prev_enc[i]):
                d.append(self.enc[i] - self.prev_enc[i])
                self.prev_enc[i] = self.enc[i]
            else:
                d.append(0.0)
        dx_b, dy_b, dth_enc = forward_kinematics(d[0], d[1], d[2], self.wheel_base_r)

        # 헤딩: IMU 연속각 우선. IMU가 없으면 엔코더 dth로 대체(경고 1회)
        if self.use_imu_yaw and self.imu_yaw is not None:
            if self.imu_prev_yaw is None:
                self.imu_prev_yaw = self.imu_yaw
                self.imu_yaw_ref = self.imu_yaw
            self.heading += wrap_angle(self.imu_yaw - self.imu_prev_yaw)
            self.imu_prev_yaw = self.imu_yaw
            self.meas_wz = self.imu_wz
        else:
            if self.use_imu_yaw and not self.imu_warned:
                self.get_logger().warn('IMU not received — heading falls back to encoder dth')
                self.imu_warned = True
            self.heading += dth_enc
            self.meas_wz = dth_enc / self.dt

        h = self.heading
        self.x += dx_b * math.cos(h) - dy_b * math.sin(h)
        self.y += dx_b * math.sin(h) + dy_b * math.cos(h)

        # 휠 속도 측정: 50 ms 창 — 20 Hz 엔코더 양자화 노이즈 완화
        if self.vel_win_t is None:
            if all(_finite(self.enc[i]) for i in DRIVE_MOTORS):
                self.vel_win_t = t
                self.vel_win_enc = [self.enc[i] for i in DRIVE_MOTORS]
        elif t - self.vel_win_t >= self.vel_window:
            span = t - self.vel_win_t
            for k, i in enumerate(DRIVE_MOTORS):
                if _finite(self.enc[i]):
                    self.wheel_vel[k] = (self.enc[i] - self.vel_win_enc[k]) / span
                    self.vel_win_enc[k] = self.enc[i]
            self.vel_win_t = t
            # 측정 twist는 틱당 차분(20 Hz 엔코더라 틱마다 0/스파이크 반복)이 아니라
            # 50 ms 창 휠 속도에서 구한다 — 기록기의 T/R 판정과 HUD가 이 값을 쓴다
            self.meas_vx, self.meas_vy, _ = forward_kinematics(
                self.wheel_vel[0], self.wheel_vel[1], self.wheel_vel[2], self.wheel_base_r)

    def _update_command(self, t: float):
        # 워치독: 무수신 시 목표 0 — 급정지가 아니라 슬루 램프로 감속(옴니 슬립 방지)
        timed_out = (self.last_cmd_time is None) or (t - self.last_cmd_time > self.cmd_timeout)
        if timed_out and not self.watchdog_tripped and self.last_cmd_time is not None:
            self.get_logger().warn('cmd_vel watchdog: timeout -> ramp to stop')
        elif not timed_out and self.watchdog_tripped:
            self.get_logger().info('cmd_vel watchdog: recovered')
        self.watchdog_tripped = timed_out

        if timed_out:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx = clamp(self.req_vx, -self.max_lin, self.max_lin)
            vy = clamp(self.req_vy, -self.max_lin, self.max_lin)
            wz = clamp(self.req_wz, -self.max_ang, self.max_ang)

        step_lin = self.slew_lin * self.dt
        step_ang = self.slew_ang * self.dt
        self.cur_vx = slew(self.cur_vx, vx, step_lin)
        self.cur_vy = slew(self.cur_vy, vy, step_lin)
        self.cur_wz = slew(self.cur_wz, wz, step_ang)

        wz_out = self.cur_wz
        if self.heading_hold:
            if abs(self.cur_wz) < HOLD_ENGAGE_EPS:
                # 램프가 끝난 시점의 헤딩을 1회만 래치 — 매 틱 갱신하면 락이 무의미
                if not self.hold_active:
                    self.hold_active = True
                    self.hold_target = self.heading
                err = self.hold_target - self.heading
                wz_out = clamp(self.hold_kp * err, -self.hold_w_max, self.hold_w_max)
            else:
                self.hold_active = False

        refs = inverse_kinematics(self.cur_vx, self.cur_vy, wz_out, self.wheel_base_r)
        duties = []
        for k in range(3):
            duty = ff_p_duty(refs[k], self.wheel_vel[k], self.k_s, self.k_v, self.k_p, self.max_duty)
            duties.append(duty)

        if self.ems_enabled and self.ems_pressed:
            duties = [0.0, 0.0, 0.0]
            if t - self.last_ems_log_t >= 1.0:
                self.get_logger().warn('EMS engaged: drive duty forced to 0')
                self.last_ems_log_t = t

        for k in range(3):
            self.cmd_pubs[k].publish(Float64(data=float(duties[k])))

    def _publish_state(self, t: float):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion_from_yaw(wrap_angle(self.heading))
        odom.twist.twist.linear.x = self.meas_vx
        odom.twist.twist.linear.y = self.meas_vy
        odom.twist.twist.angular.z = self.meas_wz
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        enc_msg = Float64MultiArray()
        enc_msg.data = [self.enc[i] if _finite(self.enc[i]) else 0.0 for i in range(NUM_TITAN_MOTORS)]
        self.enc_pub.publish(enc_msg)

        wv = Float64MultiArray()
        wv.data = list(self.wheel_vel)
        self.wheel_vel_pub.publish(wv)

        if self.ems_enabled and t - getattr(self, '_last_ems_pub_t', 0.0) >= 1.0:
            self._last_ems_pub_t = t
            self.ems_pub.publish(Bool(data=self.ems_pressed))

    def stop_motors(self):
        for p in self.cmd_pubs:
            p.publish(Float64(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 반드시 듀티 0 — Titan 워치독이 있어도 명시적으로 멈춘다
        try:
            node.stop_motors()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
