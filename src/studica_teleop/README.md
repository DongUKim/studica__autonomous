# studica_teleop — 키보드 텔레옵

`/cmd_vel`(geometry_msgs/Twist)을 래칭 방식으로 발행해 `studica_base`를 조종한다.
플랜B 기록기(`studica_repeat teach_node`)는 이 패키지의 `KeyboardTeleop` 클래스를 상속해 키를 확장한다.

## 실행

```bash
# VMX-pi 셸, 워크스페이스 루트에서 (studica_base base.launch.py가 떠 있어야 로봇이 움직인다)
source install/setup.bash
ros2 run studica_teleop keyboard_teleop
```

- **tty가 필요하다.** ssh로 붙을 때는 `ssh -t user@vmx` 로 접속한다. 파이프/백그라운드 실행 불가.
- Linux 전용(termios). Windows에서는 단위 테스트만 가능.
- 종료는 `Ctrl-C`. 종료 시 정지 명령을 3회 발행하고 터미널을 복구한다.

## 키맵 (래칭: 누르면 유지, space/x 로 정지)

| 키 | 동작 |
|---|---|
| `w` / `s` | 전진 / 후진 (`vx = ±lin`) |
| `a` / `d` | 좌 / 우 이동 (`vy = ±lin`, 좌 = +y) |
| `space`, `x` | 전부 정지 |
| `Shift+Q` / `Shift+E` | 반시계 / 시계 회전 (`wz = ±ang`) |
| `q` / `e` | 회전 잠금 상태면 무시(상태줄 안내). `l`로 잠금 해제 시 동작 |
| `l` | 회전 잠금 토글 (기본 잠금 — 병진/회전 분리 유도, 플랜B §3.2) |
| `1` / `2` / `3` | 속도 프리셋 저/중/고 = 0.10/0.20/0.35 m/s, 0.4/0.8/1.2 rad/s (기본 2) |
| `+` / `-` | 현재 속도 ±10 % (0.02~1.0 m/s, 0.1~3.0 rad/s 클램프) |
| `z` | IMU yaw 영점(`/imu/get_imu_data` params=`zero_yaw`) + `/base/reset_odom` |
| `h` | 도움말 토글 |
| `Ctrl-C` | 종료 |

프리셋을 바꾸면 이미 움직이는 성분은 방향을 유지한 채 새 속도로 바뀐다.
방향키 등 ESC 시퀀스는 무시된다. 단독 `ESC`는 서브클래스(기록기 폐기)에 전달된다.

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `publish_rate_hz` | 20.0 | `/cmd_vel` 발행 주기. 베이스 워치독(0.3 s)보다 충분히 빨라야 한다 |
| `hold_mode` | false | true면 `key_timeout_s` 동안 키가 없을 때 자동 정지 |
| `key_timeout_s` | 0.5 | hold_mode 타임아웃 |
| `cmd_vel_topic` | `/cmd_vel` | |
| `odom_topic` | `/odom` | HUD 표시용 |
| `imu_service` | `/imu/get_imu_data` | |
| `reset_odom_service` | `/base/reset_odom` | |

## HUD

프리셋·목표속도·odom pose(x, y, yaw deg)·회전 잠금·상태줄을 제자리에서 갱신한다(화면 clear 없음).
서브클래스가 `extra_hud_lines()`로 줄을 덧붙일 수 있다.

## 확장 (studica_repeat 등)

```python
from studica_teleop.keyboard_teleop import KeyboardTeleop

class TeachNode(KeyboardTeleop):
    def on_key(self, ch):          # True를 돌려주면 기본 키맵을 건너뛴다
        if ch == '[':
            target = self.read_line('to> ')   # raw 모드 안에서 한 줄 입력, ESC면 None
            ...
            return True
        return super().on_key(ch)

    def extra_hud_lines(self):
        return ['레그 A->B  세그 T  s=1.23 m']

    def on_tick(self):             # publish_rate_hz 마다
        ...
```

공개 속성: `vx_cmd`, `vy_cmd`, `wz_cmd`, `lin_speed`, `ang_speed`, `rotation_locked`, `last_odom`.
유틸: `set_status(text)`, `stop()`, `read_line(prompt)`, `run()`.

## 테스트 (ROS 무의존)

```bash
# 개발 PC, 저장소 루트에서
python -m pytest src/studica_teleop/test -q
```
`terminal.py`(ESC 시퀀스 접기), `hud.py`(폭 계산·제자리 갱신), `teleop_state.py`(래칭·프리셋)만 검증한다. 노드 자체는 실기에서 확인한다.
