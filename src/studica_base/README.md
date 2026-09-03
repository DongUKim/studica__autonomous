# studica_base — cmd_vel → 3륜 옴니 베이스 드라이버

`geometry_msgs/Twist`를 받아 Titan Quad 주행 모터 3개의 듀티를 내고, 엔코더 + navX IMU로
데드레코닝 오도메트리(`/odom`)를 발행한다. `studica_control`(C++ HAL)은 그대로 두고 그 위에 얹는
Python 노드다. 기구학·게인은 구코드 `national_project`(ControlCore.cpp, params.txt)에서 이식했다.

## 실행 (VMX-pi 셸, 워크스페이스 루트)

```bash
colcon build --packages-select studica_interfaces studica_base
source install/setup.bash
ros2 launch studica_base base.launch.py                       # 실기
ros2 launch studica_base base.launch.py sim:=true             # 하드웨어 대신 studica_repeat sim_node
ros2 launch studica_base base.launch.py heading_hold:=false   # 플랜B 재생 시 (제어기가 wz를 직접 냄)
```

`base.launch.py`는 `config/params.yaml`을 `studica_control/studica_launch.py`에 넘겨 HAL을 띄운 뒤
`base_node`를 실행한다. HAL은 pigpio 때문에 `sudo`로 뜬다(`scripts/setup_permissions.sh` 선행).

## 토픽 · 서비스

| 방향 | 이름 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/cmd_vel` | geometry_msgs/Twist | vx 전방, vy 좌측 [m/s], wz 반시계 [rad/s] |
| 구독 | `/titan0/m_{0..3}/encoder` | std_msgs/Float64 | 거리 [m] (params.yaml dist_per_tick 적용값) |
| 구독 | `/imu` | sensor_msgs/Imu | 헤딩(yaw) 소스 |
| 구독 | `/titan0/m_1/limit_rev` | std_msgs/Bool | EMS (파라미터로 변경 가능) |
| 발행 | `/titan0/m_{0..2}/cmd` | std_msgs/Float64 | 듀티 -1..1 |
| 발행 | `/odom` | nav_msgs/Odometry | pose(x, y, yaw), twist = 측정 속도 |
| 발행 | `/base/encoders` | std_msgs/Float64MultiArray | [m0..m3] m, 50 Hz |
| 발행 | `/base/wheel_vel` | std_msgs/Float64MultiArray | 주행 휠 3개 측정 속도 [m/s] |
| 발행 | `/base/ems` | std_msgs/Bool | EMS 눌림 상태 (변화 시 + 1 Hz) |
| 서비스 | `/base/reset_odom` | std_srvs/Trigger | pose 0, 헤딩 기준을 현재 IMU yaw로 |

## 동작 요약 (50 Hz 단일 타이머)

1. 워치독: `/cmd_vel` 0.3 s 무수신 → 목표 0. **급정지가 아니라 슬루 램프로 감속**한다
   (옴니 급제동은 슬립으로 오도메트리를 오염시킨다 — 구코드 결정 계승).
2. 클램프(`max_lin_mps`, `max_ang_radps`) → 슬루 램프(0.8 m/s², 6.28 rad/s²).
3. `heading_hold`: 램프 후 wz 명령이 0이면 그 순간의 헤딩을 1회 래치해 P 제어로 유지.
4. 역기구학 → 휠 목표 속도 → FF+P(`duty = kS·sgn + kV·ref + kP·(ref − meas)`) → 듀티 발행.
5. EMS 눌림이면 듀티 0 강제 + 1 Hz 경고.
6. 오도메트리: 엔코더 차분 → 정기구학 → IMU 연속각으로 회전해 누적. IMU 미수신이면 엔코더 dth로 대체(경고 1회).

기구학(`studica_base/omni_kinematics.py`):

```
w0 =  vx·√3/2 + vy/2 + R·wz      dx  = (d0 − d1)/√3
w1 = −vx·√3/2 + vy/2 + R·wz      dy  = (d0 + d1 − 2·d2)/3
w2 = −vy       + R·wz            dth = (d0 + d1 + d2)/(3R)
```

모터 0/1 = 전방 좌우, 2 = 후방. 전방에는 휠이 없다.

## 파라미터 (`base_node`)

| 이름 | 기본 | 의미 |
|---|---|---|
| `titan_name` | `titan0` | studica_control 티탄 이름 |
| `wheel_base_r_m` | 0.120 | 중심~휠 거리 **(미실측)** |
| `enc_scale` | 1.0 | encoder 토픽값 × enc_scale = m |
| `k_s`, `k_v`, `k_p` | 0.05, 1.1, 0.4 | FF+P 게인 (duty, duty/(m/s), duty/(m/s)) |
| `max_duty` | 1.0 | 듀티 클램프 |
| `max_lin_mps`, `max_ang_radps` | 0.5, 1.5 | 명령 클램프 |
| `slew_lin_mps2`, `slew_ang_radps2` | 0.8, 6.28 | 가감속 상한 |
| `cmd_timeout_s` | 0.3 | 워치독 |
| `control_rate_hz` | 50 | 제어·발행 주기 |
| `vel_window_s` | 0.05 | 휠 속도 측정 창 (20 Hz 엔코더 양자화 완화) |
| `use_imu_yaw` | true | 헤딩 소스 IMU |
| `imu_topic`, `cmd_vel_topic`, `odom_topic` | `/imu`, `/cmd_vel`, `/odom` | |
| `odom_frame`, `base_frame`, `publish_tf` | odom, base_link, false | tf2_ros는 publish_tf일 때만 임포트 |
| `heading_hold`, `hold_kp`, `hold_w_max` | true, 3.0, 1.0 | 헤딩 유지 |
| `ems_enabled`, `ems_topic`, `ems_pressed_value` | true, `/titan0/m_1/limit_rev`, false | 아래 EMS 참고 |

## 같이 고쳐야 깨지지 않는 값 (SSoT)

- `config/params.yaml`의 `titan0.m_N.dist_per_tick`(0.00021889 m/tick) ↔ `base_node`의 `enc_scale`(1.0).
  dist_per_tick을 바꾸면 encoder 토픽 단위가 바뀌므로 enc_scale로 다시 m로 맞춘다.
- `wheel_base_r_m`은 역기구학(회전 항)과 정기구학(엔코더 dth) 양쪽에 쓰인다. 하나의 파라미터라 자동 일치.
- 센서 이름(`ultrasonic_left/right`, `psd_left/right/front`) ↔ `studica_repeat` teach_node의 센서 토픽 파라미터.

## EMS 가정 (실기에서 확인 필요)

pin_setting.md: EMS 스위치는 Titan **motor 1의 Low 단자**. studica_control은 이를
`/titan0/m_1/limit_rev`(direction 1)로 발행한다고 **가정**했고, 구코드(`Hal.cpp`)가 리밋 단자를
반전(`!GetLimitSwitch`)해서 읽었으므로 눌림 = raw `false`로 두었다(`ems_pressed_value: false`).

- 증상 A: 아무 것도 안 눌렀는데 "EMS ENGAGED" 로그 + 모터 정지 → `ems_pressed_value:=true`.
- 증상 B: `limit_rev`에 반응이 없고 `limit_fwd`가 바뀜 → `ems_topic:=/titan0/m_1/limit_fwd`.
- 배선 확인: `ros2 topic echo /titan0/m_1/limit_rev`를 띄운 채 EMS를 눌러 본다.
- 잘못된 가정은 "모터가 안 도는" 안전한 쪽으로 실패하도록 두었다.

## 캘리브레이션 절차 (실기, 바퀴를 띄운 뒤 → 바닥)

1. **모터 방향**: `ros2 topic pub -1 /titan0/m_0/cmd std_msgs/Float64 "{data: 0.15}"` 순서로 0/1/2를 각각 돌려
   전방 직진 시 0 = 정회전, 1 = 역회전, 2 = 정지가 되는지 본다. 다르면 `params.yaml`의 `invert_motor`.
   엔코더 부호는 `/titan0/m_N/encoder`가 듀티 + 일 때 증가해야 한다. 아니면 `invert_encoder`.
2. **1 m 직진**: 텔레옵으로 정확히 1 m 전진 후 `/odom`의 x 오차 → `enc_scale` 보정
   (`enc_scale_new = enc_scale × 실측/odom`).
3. **360° 회전**: IMU 없이(`use_imu_yaw:=false`) 제자리 1회전 후 odom yaw가 2π에서 벗어난 비율만큼
   `wheel_base_r_m` 보정 (`R_new = R × odom_yaw/2π`). IMU를 쓸 때는 회전 명령 wz 대비 IMU 각속도로 확인.
4. **kS/kV 특성화**: 듀티 계단(0.10→0.60, 각 2 s)을 `/titan0/m_N/cmd`로 주며 `/base/wheel_vel` 기록.
   속도 0이 되는 듀티 = kS, 기울기(duty per m/s) = kV. 구코드는 kV가 0.0011 vs 0.00187 duty/(mm/s)로
   1.7배 불일치 상태였다 — 실측이 판정 근거.
5. `heading_hold`가 진동하면 `hold_kp`를 낮춘다.

## 테스트 (호스트, ROS 불필요)

```bash
python -m pytest src/studica_base/test -q
```
