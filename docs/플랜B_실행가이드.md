# 플랜B (주행기록 Teach & Repeat) 실행 가이드

VMX-pi + Titan Quad 3륜 옴니 로봇을 ROS2(Humble, Python)로 **키보드 텔레옵 → 경로 기록 → 경로 재생**하는 절차.
학생용 원클릭 스크립트는 `scripts/` 에 있고, 내부 구조는 각 패키지 README 를 본다.

| 패키지 | 역할 |
|---|---|
| `src/studica_control` | Studica 공식 C++ HAL. Titan·IMU·초음파·PSD 드라이버 (수정하지 않음) |
| `src/studica_interfaces` | 서비스·액션 정의 (RecordStart/Stop, FollowRoute, RelativeMove) |
| `src/studica_base` | `/cmd_vel` → 옴니 역기구학 → 모터 듀티, 엔코더+IMU 데드레코닝 `/odom`, EMS 가드, 하드웨어 `params.yaml` |
| `src/studica_teleop` | 키보드 텔레옵 (확장 가능한 `KeyboardTeleop` 클래스) |
| `src/studica_repeat` | 플랜B: 기록기(teach_node) · 재생기(repeat_node) · 검증(validate_node) · 시뮬레이터(sim_node) · ROS 무의존 코어 + pytest |

## 1. 최초 1회 준비 (VMX-pi 셸)

```bash
cd ~/studica_ros2                      # 이 저장소를 clone 한 위치
bash scripts/setup_permissions.sh      # pigpio 용 sudo 규칙 (studica_control 제공)
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

- 빌드 후 `install/setup.bash` 가 생기면 `scripts/*.py` 가 이를 **자동 소싱**하므로 학생은 `source` 를 몰라도 된다.
- 하드웨어 핀 배치는 `src/studica_base/config/params.yaml` 한 곳에서만 바꾼다 (근거: 루트 `pin_setting.md`).

## 2. 학생용 실행 스크립트

모두 `python3 scripts/<이름>.py` 로 실행하고 **Ctrl-C 한 번**으로 전부 종료된다.
하드웨어·베이스 launch 는 백그라운드로 뜨고 로그는 `~/studica_logs/*.log` 에 쌓인다 (문제 생기면 이 파일을 본다).

### 2.1 텔레옵

```bash
python3 scripts/teleop.py          # 실기
python3 scripts/teleop.py --sim    # 하드웨어 없이 시뮬레이터
```

| 키 | 동작 |
|---|---|
| `W / S` | 전진 / 후진 (래칭 — 한 번 누르면 계속 감) |
| `A / D` | 좌 / 우 횡이동 |
| `Shift+Q / Shift+E` | 반시계 / 시계 회전 (소문자 q/e 는 잠금 — 병진·회전 분리 유도) |
| `Space` 또는 `X` | 정지 |
| `1 / 2 / 3` | 속도 저 0.10 / 중 0.20 / 고 0.35 m/s (회전 0.4 / 0.8 / 1.2 rad/s) |
| `+ / -` | 현재 속도 ±10 % |
| `L` | 회전 잠금 토글 |
| `Z` | IMU yaw 영점 + 오도메트리 리셋 |
| `H` | 도움말 |

### 2.2 경로 기록 (티칭)

```bash
python3 scripts/teach.py mission_a          # 실기, 미션 폴더 mission_a
python3 scripts/teach.py mission_a --sim
```

텔레옵 키에 더해:

| 키 | 동작 |
|---|---|
| `[` | 기록 시작 — from / to 체크포인트 이름 입력 (정지 상태에서만) |
| `]` | 기록 종료·저장 (이전 버전은 `.vN.jsonl` 로 3개까지 보존) |
| `ESC` | 기록 폐기 |
| `R` | 직전 레그 재기록 |

저장 위치: `~/studica_missions/<mission>/taught_legs/<from>__<to>.jsonl` + `index.yaml`.
체크포인트 이름 목록을 미리 두고 싶으면 `~/studica_missions/<mission>/graph.yaml` 에 `nodes: [ST, P1, P2, ...]` 를 적는다.

**운전 수칙 (HUD 에도 표시)**
1. 레그는 정지 상태에서 시작·종료. 종료 지점 = 다음 레그의 시작 지점.
2. 가급적 병진만으로 주행. 회전은 스테이션 정렬 등 꼭 필요한 곳에서만 (회전 중엔 거리센서 기록이 자동 무효화됨).
3. 벽 근처 직선 구간은 **벽에서 15~25 cm** 유지 — PSD 30 cm 게이트 안에 들어와야 서명이 기록되고, 재생 때 드리프트 보정 앵커가 된다.
4. 티칭 속도가 재생 상한이다. 최종 티칭은 목표 속도의 80 % 정도로.

### 2.3 경로 재생 (자율주행)

```bash
python3 scripts/repeat.py mission_a --list            # 티칭된 레그 확인
python3 scripts/repeat.py mission_a ST P1 P2          # ST→P1→P2 순서로 재생
python3 scripts/repeat.py mission_a ST P1 --speed 0.8 # 티칭 속도의 80 %
python3 scripts/repeat.py mission_a --validate        # 전 레그 연속 재생 + 검증 리포트
```

- 로봇을 **첫 체크포인트의 티칭 시작 자세**에 놓고 시작한다 (거치 지그 권장, ±2 mm / ±0.5°).
- 연속 쌍 중 티칭되지 않은 엣지가 있으면 출발 전에 `untaught edge A->B` 로 거절된다.
- 재생 중 횡오차가 8 cm 를 0.5 s 넘으면 정지하고 실패를 반환한다.
- 검증 리포트: `~/studica_missions/<mission>/validation/tracking.csv`, `report.txt` (레그별 e_lat p95 ≤ 25 mm, 종점 오차 ≤ 15 mm 가 합격 기준).

## 3. 실기 투입 전 캘리브레이션 (순서대로, `src/studica_base/README.md` 상세)

1. **모터 방향**: 텔레옵 `W` 로 전진하는지. 반대로 가는 휠은 `params.yaml` 의 `invert_motor`.
2. **엔코더 부호**: `/odom` 의 x 가 전진 시 증가하는지. 아니면 `invert_encoder`.
3. **1 m 직진**: 줄자와 `/odom` 비교 → `dist_per_tick` (기본 0.00021889 m/tick) 미세 조정.
4. **360° 회전**: `/odom` yaw 와 IMU 가 일치하는지 → `wheel_base_r_m` (기본 0.120) 조정.
5. **속도 특성화**: 듀티 계단 주행으로 `k_s`, `k_v` 재산출 (구코드 기본값 0.05 / 1.1).

EMS 는 Titan 모터1 Low 단자(`/titan0/m_1/limit_rev`, 눌림 = false)로 가정했다. 모터가 전혀 안 돌고 로그에 `EMS engaged` 가 계속 뜨면 `ems_pressed_value` 를 뒤집는다.

## 4. 시뮬레이터로 전 사이클 연습

하드웨어 없이 `--sim` 으로 티칭 → 재생 → 검증을 그대로 할 수 있다. 시뮬레이터는 슬립 2 %, IMU 드리프트 1°/min 을 기본으로 넣어 서명 보정이 실제로 동작하는지 볼 수 있게 했다 (`src/studica_repeat/config/sim_world.yaml` 에 벽·장애물).

## 5. 검증 명령 (개발자용)

```bash
# ROS 없이 돌아가는 코어 테스트 (PC/Pi 어디서나)
python3 -m pytest src/studica_base/test src/studica_teleop/test src/studica_repeat/test -q
# 티칭 파일을 엑셀용 CSV 로
python3 src/studica_repeat/tools/leg_to_csv.py ~/studica_missions/mission_a/taught_legs/ST__P1.jsonl
```

## 6. 한계 (플랜B 문서 §9 요약)

- 절대 좌표 리셋이 없다. 서명 보정은 벽 근처(PSD 30 cm 이내)에서만 작동하고, 긴 개활 구간의 드리프트는 남는다.
- 출발 자세 재현성이 전 레그 정확도를 좌우한다 — 지그 없이는 신뢰하지 말 것.
- 코트가 바뀌면 영향 엣지를 재티칭한다 (엣지당 2~3분).
