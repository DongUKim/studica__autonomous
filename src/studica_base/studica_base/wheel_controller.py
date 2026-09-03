"""휠 속도 FF+P 제어와 슬루 램프 (ROS 무의존).

게인은 구코드 national_project params.txt의 mm 단위 값을 m 단위로 환산한 것이다
(kV 0.0011 duty/(mm/s) → 1.1 duty/(m/s)). kS/kV/kP는 실기 특성화(벤치 1)로 갱신할 것.
"""

# 이 속도 이하의 목표는 정지로 간주 — kS 정지마찰항이 미세 목표에서 모터를 떨게 하는 것을 막는다
VEL_DEADBAND_MPS = 0.001


def sign(v: float) -> float:
    if v > 0.0:
        return 1.0
    if v < 0.0:
        return -1.0
    return 0.0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def slew(cur: float, target: float, max_step: float) -> float:
    """cur를 target 쪽으로 최대 max_step만큼만 이동. 옴니 급변속은 슬립으로 오도메트리를 오염시킨다."""
    if target > cur + max_step:
        return cur + max_step
    if target < cur - max_step:
        return cur - max_step
    return target


def ff_p_duty(ref: float, meas: float, k_s: float, k_v: float, k_p: float,
              max_duty: float = 1.0) -> float:
    """목표 휠 속도 ref, 측정 meas [m/s] → 듀티 [-max_duty, max_duty]."""
    if abs(ref) < VEL_DEADBAND_MPS:
        return 0.0
    duty = k_s * sign(ref) + k_v * ref + k_p * (ref - meas)
    return clamp(duty, -max_duty, max_duty)
