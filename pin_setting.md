# 핀 설정 정리 — Project_1st_mission (VMX-pi + Titan Quad)

> 출처: `Project_1st_mission__260810.zip` (`Project_1st_mission_last/src/main/include/Constants.h`,
> `Motor.cpp`, `Sensor.cpp` 기준)

## 1. DC 모터 (Titan Quad, CAN)

TitanQuad 생성자: `{CAN ID, encoder CPR, Titan 채널}` → 모든 모터가 CAN ID 42를 공유하고 채널 0~3으로 구분된다.

| motor[] 인덱스 | Titan 채널 | 용도 | 반전(Inverted) | 내장 엔코더 핀 (A/B) |
|---|---|---|---|---|
| motor[0] | 0 | 주행 (3륜 옴니 중 1) | X | DIO 0 / 1 |
| motor[1] | 1 | 주행 (3륜 옴니 중 1) | X | DIO 2 / 3 |
| motor[2] | 2 | 주행 (3륜 옴니 중 1) | X | DIO 4 / 5 |
| motor[3] | 3 | 상하축(OMS Z, 리프트) | O | DIO 6 / 7 |

- 주행 모터는 3개(`ve_l[3]` 등, `Move.cpp`)로 3륜 옴니 드라이브 구성.
- Titan 리밋 스위치 단자(모터 보드의 Low 단자를 디지털 입력처럼 사용):
  - `OMS_Z_LIMIT` = Titan motor 3의 Low 단자 → 상하축 리밋 스위치
  - `EMS_LIMIT` = Titan motor 1의 Low 단자 → 비상정지(EMS) 스위치

## 2. 서보모터 (studica::Servo, 연속회전 서보 5개)

VMX HighCurrentDIO 대역(15~19)에 연결. `studica::Servo` 생성자에는 `핀 - 12`로 전달되어 채널 3~7로 매핑됨.

| 인덱스 (SV_*) | 이름 | VMX 핀 |
|---|---|---|
| SV_GOLF_GRIPPER (0) | 골프공 그리퍼 | 15 |
| SV_PALETTE_GRIPPER (1) | 팔레트 그리퍼 | 16 |
| SV_CAMERA (2) | 카메라 | 17 |
| SV_X_AXIS (3) | X축 | 18 |
| SV_Y_AXIS (4) | Y축 | 19 |

## 3. 센서

| 센서 | 종류 | 핀 |
|---|---|---|
| 초음파(왼쪽) | frc::Ultrasonic | Trigger 20 / Echo 11 |
| 초음파(오른쪽) | frc::Ultrasonic | Trigger 21 / Echo 8 |
| IR·PSD(왼쪽) | frc::AnalogInput | 1 |
| IR·PSD(오른쪽) | frc::AnalogInput | 2 |
| IR·PSD(앞쪽) | frc::AnalogInput | 3 |
| 자이로(navX) | AHRS | SPI MXP 포트 (핀 번호 없음) |
| 스위치1 | frc::DigitalInput | 9 |
| 스위치2 | frc::DigitalInput | 10 |

## 4. 기타 출력

| 항목 | 핀 |
|---|---|
| LED 초록 | 13 |
| LED 빨강 | 14 |

## 5. 핀 대역별 요약

- **AnalogInput (0~3)**: PSD 1(왼쪽) / 2(오른쪽) / 3(앞쪽)
- **FlexDIO (8~11)**: 스위치1=9, 스위치2=10, 초음파 echo → 왼쪽=11, 오른쪽=8
- **HighCurrentDIO (12~21, 출력 전용)**: LED 초록=13, LED 빨강=14, 서보 5개=15~19, 초음파 trigger → 왼쪽=20, 오른쪽=21
- **내장 Encoder DIO (0~7)**: motor0=0/1, motor1=2/3, motor2=4/5, motor3(OMS Z)=6/7
- **Titan 리밋 단자**: EMS=motor1 Low, OMS Z 리밋=motor3 Low
- **SPI (MXP)**: navX 자이로

> ※ `Constants.h` 주석 기준으로, echo 핀 8번은 과거 EMS 자리였으나 EMS는 Titan 리밋 단자로 이전됨.
