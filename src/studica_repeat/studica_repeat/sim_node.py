#!/usr/bin/env python3
"""기구학 시뮬레이터 노드 — studica_control(HAL)을 토픽 수준에서 대체한다.

base_node는 수정 없이 그대로 돌고, 텔레옵 → 티칭 → 재생 → 검증 전 사이클을 실기 없이 돌릴 수 있다
(플랜B §7). 물리는 sim_model.SimModel(ROS 무의존)에 있다.

구독: /titan0/m_{0..2}/cmd (Float64 듀티)
발행: /titan0/m_{0..3}/encoder (Float64, m), /titan0/m_1/limit_rev (Bool, EMS 단자),
      /imu (sensor_msgs/Imu), /<sensor>/range ×5 (sensor_msgs/Range), /sim/ground_truth (Odometry)
"""
from __future__ import annotations

import math
import os
from typing import Dict

import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float64

from studica_repeat.sim_model import DEFAULT_MOUNTS, SensorMount, SimModel, default_world, load_walls

# 채널 → 발행 토픽 (studica_base/config/params.yaml 의 센서 이름과 SSoT)
RANGE_TOPICS: Dict[str, str] = {
    'us_l': '/ultrasonic_left/range',
    'us_r': '/ultrasonic_right/range',
    'psd_l': '/psd_left/range',
    'psd_r': '/psd_right/range',
    'psd_f': '/psd_front/range',
}
US_FOV_RAD = math.radians(25.0)
PSD_FOV_RAD = math.radians(5.0)


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__('sim_node')
        d = self.declare_parameter
        d('rate_hz', 50.0)
        d('titan_name', 'titan0')
        d('imu_topic', '/imu')
        d('wheel_base_r_m', 0.12)
        d('k_s', 0.05)
        d('k_v', 1.1)
        d('tau_s', 0.1)
        d('slip_pct', 2.0)
        d('imu_drift_deg_per_min', 1.0)
        d('start_x', 0.32)
        d('start_y', 0.32)
        d('start_th', 0.0)
        d('ems_pressed', False)
        d('world_file', '')
        d('seed', -1)
        for ch, m in DEFAULT_MOUNTS.items():
            d(f'sensor.{ch}.x', m.x)
            d(f'sensor.{ch}.y', m.y)
            d(f'sensor.{ch}.yaw_deg', math.degrees(m.yaw))

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        mounts = {ch: SensorMount(float(p(f'sensor.{ch}.x')), float(p(f'sensor.{ch}.y')),
                                  math.radians(float(p(f'sensor.{ch}.yaw_deg'))), m.kind)
                  for ch, m in DEFAULT_MOUNTS.items()}
        seed = int(p('seed'))
        self.model = SimModel(
            r=float(p('wheel_base_r_m')), k_s=float(p('k_s')), k_v=float(p('k_v')),
            tau_s=float(p('tau_s')), slip_pct=float(p('slip_pct')),
            imu_drift_deg_per_min=float(p('imu_drift_deg_per_min')),
            walls=self._load_world(str(p('world_file'))), mounts=mounts,
            x=float(p('start_x')), y=float(p('start_y')), th=float(p('start_th')),
            seed=None if seed < 0 else seed)
        self.ems_pressed = bool(p('ems_pressed'))
        titan = str(p('titan_name'))
        self.dt = 1.0 / max(1.0, float(p('rate_hz')))

        for i in range(3):
            self.create_subscription(Float64, f'/{titan}/m_{i}/cmd',
                                     lambda msg, i=i: self.model.set_duty(i, float(msg.data)), 1)
        self.enc_pubs = [self.create_publisher(Float64, f'/{titan}/m_{i}/encoder', 10) for i in range(4)]
        # EMS = motor1 Low 단자(limit_rev). 눌림 = raw false (studica_base README의 가정과 동일)
        self.ems_pub = self.create_publisher(Bool, f'/{titan}/m_1/limit_rev', 10)
        self.imu_pub = self.create_publisher(Imu, str(p('imu_topic')), 10)
        self.range_pubs = {ch: self.create_publisher(Range, topic, 10) for ch, topic in RANGE_TOPICS.items()}
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            'sim ready: start (%.2f, %.2f, %.1f deg), slip %.1f%%, imu drift %.2f deg/min, walls %d'
            % (self.model.x, self.model.y, math.degrees(self.model.th), self.model.slip_pct,
               self.model.imu_drift_deg_per_min, len(self.model.walls)))

    def _load_world(self, world_file: str):
        if not world_file:
            try:
                from ament_index_python.packages import get_package_share_directory
                world_file = os.path.join(get_package_share_directory('studica_repeat'),
                                          'config', 'sim_world.yaml')
            except Exception:  # 설치 전(소스 실행)에는 기본 월드로 대체
                world_file = ''
        if world_file and os.path.exists(world_file):
            with open(world_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            walls = load_walls(data)
            if walls:
                return walls
            self.get_logger().warn(f'{world_file}: 벽이 없어 기본 월드 사용')
        elif world_file:
            self.get_logger().warn(f'{world_file} 없음 — 기본 월드 사용')
        return default_world()

    def _tick(self) -> None:
        m = self.model
        m.step(self.dt)
        now = self.get_clock().now().to_msg()

        for i in range(3):
            self.enc_pubs[i].publish(Float64(data=float(m.enc[i])))
        self.enc_pubs[3].publish(Float64(data=0.0))
        self.ems_pub.publish(Bool(data=not self.ems_pressed))

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.orientation = _quat_from_yaw(m.imu_yaw())
        imu.angular_velocity.z = float(m.imu_wz())
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        for ch, dist in m.sense().items():
            mount = m.mounts[ch]
            r = Range()
            r.header.stamp = now
            r.header.frame_id = f'{ch}_link'
            if mount.kind == 'psd':
                r.radiation_type = Range.INFRARED
                r.field_of_view = PSD_FOV_RAD
                r.min_range, r.max_range = 0.1, 0.8
            else:
                r.radiation_type = Range.ULTRASOUND
                r.field_of_view = US_FOV_RAD
                r.min_range, r.max_range = 0.02, 4.0
            r.range = float(dist)
            self.range_pubs[ch].publish(r)

        gt = Odometry()
        gt.header.stamp = now
        gt.header.frame_id = 'world'
        gt.child_frame_id = 'base_link'
        gt.pose.pose.position.x = float(m.x)
        gt.pose.pose.position.y = float(m.y)
        gt.pose.pose.orientation = _quat_from_yaw(m.th)
        gt.twist.twist.angular.z = float(m.wz)
        self.truth_pub.publish(gt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
