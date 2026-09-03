import math

from studica_repeat.sim_model import (SimModel, default_world, forward_kinematics, load_walls,
                                      ray_cast, rectangle_walls)


def test_ray_cast_rectangle():
    walls = rectangle_walls(0.0, 0.0, 4.0, 2.0)
    assert math.isclose(ray_cast(1.0, 1.0, 0.0, walls), 3.0)
    assert math.isclose(ray_cast(1.0, 1.0, math.pi / 2, walls), 1.0)
    assert math.isclose(ray_cast(1.0, 1.0, math.pi, walls), 1.0)
    # 벽과 평행한 광선은 그 벽을 무시하고 맞은편 벽을 본다
    assert math.isclose(ray_cast(1.0, 0.0, 0.0, walls), 3.0)


def test_sense_applies_mounts_and_limits():
    m = SimModel(walls=rectangle_walls(0.0, 0.0, 4.0, 2.0), x=1.0, y=0.25, th=0.0,
                 slip_pct=0.0, seed=0)
    d = m.sense(noise=False)
    # psd_r: 장착 (0.05,-0.10) → y=0.15, 아래벽까지 0.15 (유효범위 안)
    assert math.isclose(d['psd_r'], 0.15, abs_tol=1e-9)
    # psd_l: 위벽까지 1.65 → PSD 상한 0.8 초과 → inf
    assert math.isinf(d['psd_l'])
    # us_l: (0,0.10) → 위벽까지 1.65, 초음파는 유효
    assert math.isclose(d['us_l'], 1.65, abs_tol=1e-9)
    # psd_f: 오른쪽 벽까지 2.88 → inf
    assert math.isinf(d['psd_f'])


def test_duty_mix_moves_forward_without_slip():
    m = SimModel(slip_pct=0.0, imu_drift_deg_per_min=0.0, gyro_noise_sd=0.0, seed=0,
                 x=0.0, y=0.0, th=0.0)
    # 전진 믹싱: w0 = +, w1 = −, w2 = 0
    m.set_duty(0, 0.5)
    m.set_duty(1, -0.5)
    m.set_duty(2, 0.0)
    for _ in range(100):
        m.step(0.02)
    assert m.x > 0.3
    assert abs(m.y) < 1e-6
    assert abs(m.th) < 1e-6
    # 정상상태 휠 속도 = (0.5-0.05)/1.1
    assert math.isclose(m.wheel_v[0], (0.5 - 0.05) / 1.1, rel_tol=1e-2)


def test_encoder_integration_equals_truth_without_slip():
    m = SimModel(slip_pct=0.0, imu_drift_deg_per_min=0.0, gyro_noise_sd=0.0, seed=0,
                 x=0.0, y=0.0, th=0.0)
    m.set_duty(0, 0.4)
    m.set_duty(1, 0.2)
    m.set_duty(2, -0.3)
    x = y = th = 0.0
    prev = list(m.enc)
    for _ in range(200):
        m.step(0.02)
        d = [m.enc[i] - prev[i] for i in range(3)]
        prev = list(m.enc)
        dx, dy, dth = forward_kinematics(d[0], d[1], d[2], m.r)
        c, s = math.cos(th), math.sin(th)
        x += c * dx - s * dy
        y += s * dx + c * dy
        th += dth
    assert math.isclose(x, m.x, abs_tol=1e-3)
    assert math.isclose(y, m.y, abs_tol=1e-3)
    assert math.isclose(math.atan2(math.sin(th), math.cos(th)), m.th, abs_tol=1e-3)


def test_slip_makes_truth_differ():
    m = SimModel(slip_pct=5.0, seed=1, x=0.0, y=0.0, th=0.0)
    m.set_duty(0, 0.5)
    m.set_duty(1, -0.5)
    for _ in range(200):
        m.step(0.02)
    dx, _, _ = forward_kinematics(m.enc[0], m.enc[1], m.enc[2], m.r)
    assert abs(dx - m.x) > 1e-4


def test_deadband_below_ks():
    m = SimModel(slip_pct=0.0, seed=0)
    m.set_duty(0, 0.04)
    for _ in range(50):
        m.step(0.02)
    assert m.enc[0] == 0.0


def test_load_walls_and_default_world():
    walls = load_walls({'walls': [[0, 0, 1, 0]], 'rects': [[0, 0, 1, 1]]})
    assert len(walls) == 5
    assert len(default_world()) == 12
