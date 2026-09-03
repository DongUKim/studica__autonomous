# studica_repeat — 플랜B 주행기록(Teach & Repeat) 네비게이션

키보드 텔레옵으로 레그(체크포인트 사이 1회 이동)를 **기록**하고, 기록된 **경로(pose 궤적)와 센서 서명**을
50 Hz 폐루프 경로추종으로 **재생**한다. cmd_vel을 그대로 다시 내보내는 개루프 재생이 아니다 —
cmd_vel은 재생 시 매 주기 오차로부터 새로 계산돼 `/cmd_vel`로 발행된다 (`플랜B_주행기록_네비게이션.md` §1).

| 노드 | 역할 |
|---|---|
| `teach_node` | `studica_teleop.KeyboardTeleop` 확장 기록기. `[` `]` ESC `r` 키 + HUD + `/repeat/record_*` 서비스 |
| `repeat_node` | `/nav/follow_route`(FollowRoute), `/nav/relative_move`(RelativeMove) 액션 서버. `/repeat/tracking` 발행 |
| `validate_node` | 레그 연속 재생 → `tracking.csv`, `report.txt` (e_lat p95 / 종점 오차 / 힌트) |
| `sim_node` | studica_control(HAL)을 토픽 수준에서 대체하는 기구학 시뮬레이터 (슬립·IMU 드리프트·레이캐스트 센서) |
| `send_route` | 액션 클라이언트 CLI |

ROS 무의존 코어는 `studica_repeat/core/` (세그먼트 분할, JSONL/index I/O, 폴리라인 사영, 앵커링,
재생 제어기, RelativeMove, 그래프, 리포트), `studica_repeat/sim_model.py` (시뮬 물리). `test/`의 pytest는
colcon 없이 `python -m pytest src/studica_repeat/test` 로 돈다.

## 파일 배치

```
<missions_dir>/<mission>/                (기본 ~/studica_missions/mission_a)
├── graph.yaml                           # 선택: nodes: [ST, N1, N2, ...]  — 티칭 프롬프트에 후보로 표시
├── taught_legs/
│   ├── index.yaml                       # 레그 목록: from, to, file, length_m, duration_s, recorded, version, samples
│   ├── N1__N2.jsonl                     # 최신 (첫 행 meta, 이후 50 Hz 샘플)
│   └── N1__N2.v1.jsonl                  # 이전 버전 (최근 3개 보존)
└── validation/
    ├── tracking.csv                     # t, leg, seg, s, e_lat, e_th, lat_bias, s_bias
    └── report.txt
```

샘플 1행(단위 m, rad; `x,y,th`는 레그 시작 pose 원점의 레그-로컬, `yaw`는 IMU 절대):
```json
{"t": 1.24, "seg": "T", "seg_id": 0, "x": 0.412, "y": 0.008, "th": 0.004, "v": 0.31, "wz": 0.0,
 "enc": [1.032, 1.029, 1.03, 0.0], "yaw": 0.004, "us_l": 0.412, "us_r": null, "psd_l": 0.148,
 "psd_r": null, "psd_f": null, "cmd": [0.3, 0.0, 0.0],
 "valid": {"us_l": true, "us_r": false, "psd_l": true, "psd_r": false, "psd_f": false}}
```
게이트: PSD ≥ 0.30 m → null(사용자 요구), 초음파 > 0.85 m → null, `|wz| > 0.1 rad/s` 중 + 종료 후 0.3 s는
전 채널 무효(회전 오염). 엑셀 검토용 변환: `python3 tools/leg_to_csv.py N1__N2.jsonl`.

## 1. 티칭 (teach)

터미널 1 — 베이스 스택 (실기는 `sim:=true` 생략):
```bash
ros2 launch studica_repeat teach.launch.py mission:=mission_a sim:=true
```
터미널 2 — 기록기 (tty 필요, ssh는 `ssh -t`):
```bash
ros2 run studica_repeat teach_node --ros-args -p mission:=mission_a
```

키 (텔레옵 키맵은 `studica_teleop/README.md`):

| 키 | 동작 |
|---|---|
| `[` | 기록 시작. `from >` `to >` 입력(알려진 체크포인트가 상태줄에 표시). **정지 상태에서만** |
| `]` | 종료·저장 (이전 파일은 `.vK.jsonl`로 보존, index 갱신) |
| `ESC` | 로봇 정지 후 폐기 확인 `y` |
| `r` | 마지막 from→to 로 재기록 |
| `Shift+Q/E` | 회전 (소문자 q/e는 기본 잠금 — 병진/회전 분리 유도) |

HUD: 레그 from→to, seg `T0`/`R1`, 경과 호장, 샘플 수, 채널별 유효 플래그(PSD가 30 cm 게이트 안인지),
회전 오염 차단 상태, 병진+회전 동시 조작 경고, 운전 수칙 4줄.

서비스(자동화·테스트): `/repeat/record_start {from_node, to_node}`, `/repeat/record_stop {save}`.

**운전 수칙** (플랜B §3.3): ① 병진 위주, 회전은 꼭 필요한 곳만 ② 보정 존은 벽에서 15~25 cm ③ 시작·종료는
정지 상태, 종료점 = 다음 레그 시작점 ④ 티칭 속도가 재생 상한 — 최종 티칭은 목표 경기 속도로.

## 2. 재생 (repeat)

```bash
ros2 launch studica_repeat repeat.launch.py mission:=mission_a sim:=true
ros2 run studica_repeat send_route N1 N2 N3            # 연속 엣지 레그 재생
ros2 run studica_repeat send_route --relative 0.2 0.0  # 0.3 m 이내 데드레코닝 이동
```
`repeat.launch.py`는 베이스의 `heading_hold`를 끈다 — T 세그먼트의 wz는 재생 제어기가 낸다.

동작 (플랜B §4): 레그 시작 시 현재 odom pose에 강체변환으로 앵커링 → T 세그먼트는 사영(s, e_lat) 기반
`vx = min(v(s), vmax)`(말단 √(2·a·s_remain) 감속), `vy = −k_lat·e_lat`, `wz = k_th·e_th` → R 세그먼트는
IMU yaw P 제어(거리센서 미사용) → 종점 도달·디바운스 후 다음 세그먼트/레그. `|e_lat| > 8 cm` 0.5 s → abort.
서명 보정은 티칭 유효 서명이 있는 s 구간에서만, 회전 중에는 미반영. 미티칭 엣지는 즉시 abort
(`untaught edge A->B`).

게인은 `config/repeat_params.yaml`. 액션 정의는 `studica_interfaces`.

## 3. 검증 (validate)

```bash
ros2 launch studica_repeat validate.launch.py mission:=mission_a sim:=true [legs:=N1__N2,N2__N3]
```
전 레그(또는 지정 목록)를 순서대로 재생하며 `/repeat/tracking`을 모아 `validation/tracking.csv`,
`report.txt`를 쓴다. 합격 초기값: 레그별 e_lat p95 ≤ 25 mm, 종점 오차 ≤ 15 mm(라인트레이서 병용 시 25 mm).
힌트: 특정 레그만 불량 → 재티칭 / 전 레그 같은 방향 편향 → 지그·IMU / 서명 잔차 급증 → 코트 변경.
연속 재생은 "레그 종점 = 다음 레그 시작점"을 전제한다 — 이어지지 않는 목록이면 `legs:=`로 부분 지정.

## 4. 시뮬레이션

`sim:=true`면 `studica_base/base.launch.py`가 HAL 대신 `sim_node`를 띄운다. base_node는 수정 없이 그대로 돈다.
파라미터: `slip_pct`(2.0), `imu_drift_deg_per_min`(1.0), `tau_s`, `k_s`/`k_v`(베이스 FF와 동일), `start_x/y/th`,
`world_file`(`config/sim_world.yaml`), `sensor.<ch>.x/y/yaw_deg` 장착 오프셋, `ems_pressed`.
강건성 확인은 노이즈를 올려서: `-p slip_pct:=5.0 -p imu_drift_deg_per_min:=3.0`.

시뮬 한 사이클: `teach.launch sim:=true` → `teach_node`로 레그 2~3개 티칭 → `validate.launch sim:=true` → 리포트.

## 5. 튜닝 메모

- `k_lat`가 낮으면 횡오차 수렴이 느리고, 높으면 옴니 횡슬립으로 진동한다. sim에서 e_lat p95를 보며 조정.
- `vmin`은 정지마찰(베이스 `k_s`)보다 충분히 큰 속도여야 말단에서 멈추지 않는다.
- `end_tol`을 줄이면 종점 정밀도는 오르나 디바운스 실패로 "끝나지 않는" 레그가 생긴다.
- 서명 보정(`sig_k`)은 벽 근처 직선에서만 작동한다. 개활 구간의 드리프트는 그대로 남는다.

## 6. 한계 (플랜B §9, 정직한 전제)

1. 절대 좌표 리셋이 없다 — 서명 보정은 벽 근처에서만. 긴 개활 구간은 라인트레이서 병용이 사실상 필수.
2. 출발 재현성은 스테이션 거치 지그(±2 mm/±0.5°)에 달렸다. 지그 없으면 전 레그가 함께 틀어진다.
3. 코트 변경 시 영향 엣지 재티칭 (엣지당 2~3분). 평가코트 2개면 `missions_dir`를 코트별로 분리.
4. 배터리·적재 변화는 경로추종이 흡수하지만 속도 상한 여유 20%를 남겨 티칭할 것.
5. 센서 장착 오프셋(`sim_node`, `tools/robot_geom.py`)은 미실측 임시치 — 실기와 sim의 서명 절대값은 다르다.
