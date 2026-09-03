import math

from studica_repeat.core.anchor import wrap
from studica_repeat.core.controller import RepeatController, RepeatGains
from studica_repeat.core.leg_io import make_sample
from studica_repeat.core.path import compile_leg

DT = 0.02
WALL_Y = 0.20   # 티칭 경로(y=0) 왼쪽 0.2 m에 벽 → 좌측 PSD 티칭값 0.2


def straight_leg(length=1.0, v=0.25, with_psd=False):
    samples, t, x = [], 0.0, 0.0
    while x <= length + 1e-9:
        raw = {'psd_l': WALL_Y} if with_psd else {}
        samples.append(make_sample(t, 'T', 0, x, 0.0, 0.0, v, 0.0, [0] * 4, 0.0, raw, True))
        t += DT
        x += v * DT
    return samples


class Robot:
    """홀로노믹 운동학 모델. odom 드리프트를 흉내내기 위해 true pose와 odom pose를 분리."""

    def __init__(self, x=0.0, y=0.0, th=0.0, odom_offset_y=0.0):
        self.x, self.y, self.th = x, y, th
        self.odom_offset_y = odom_offset_y

    def apply(self, vx, vy, wz):
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += (c * vx - s * vy) * DT
        self.y += (s * vx + c * vy) * DT
        self.th = wrap(self.th + wz * DT)

    @property
    def odom(self):
        return self.x, self.y + self.odom_offset_y, self.th


def run(ctrl, robot, sensors_fn=lambda r: {}, max_t=30.0):
    t = 0.0
    last = None
    while t < max_t:
        ox, oy, oth = robot.odom
        last = ctrl.step(t, ox, oy, oth, oth, 0.0, sensors_fn(robot))
        if last.done or last.aborted:
            break
        robot.apply(last.vx, last.vy, last.wz)
        t += DT
    return last, t


def test_straight_follow_converges_from_lateral_offset():
    segs = compile_leg(straight_leg(1.0))
    ctrl = RepeatController(segs, RepeatGains())
    robot = Robot(y=0.05)
    res, t = run(ctrl, robot)
    assert res.done and not res.aborted
    assert math.hypot(robot.x - 1.0, robot.y) < 0.02
    assert abs(robot.y) < 0.01
    assert t < 10.0


def test_speed_profile_and_decel_respected():
    segs = compile_leg(straight_leg(1.0, v=0.25))
    ctrl = RepeatController(segs, RepeatGains())
    robot = Robot()
    vmax_seen = 0.0
    t = 0.0
    while t < 20:
        res = ctrl.step(t, robot.x, robot.y, robot.th, robot.th, 0.0, {})
        if res.done:
            break
        vmax_seen = max(vmax_seen, res.vx)
        assert res.vx <= 0.25 + 1e-9
        robot.apply(res.vx, res.vy, res.wz)
        t += DT
    assert vmax_seen > 0.2


def test_signature_trimmer_moves_lat_bias_the_right_way():
    segs = compile_leg(straight_leg(1.0, with_psd=True))
    ctrl = RepeatController(segs, RepeatGains())
    # 실제로는 경로 좌측 0.03 m에 있는데 odom은 y=0이라고 믿는다(드리프트).
    robot = Robot(y=0.03, odom_offset_y=-0.03)

    def sensors(r):
        return {'psd_l': WALL_Y - r.y}     # 왼쪽 벽까지 실제 거리

    res, _ = run(ctrl, robot, sensors)
    assert res.done and not res.aborted
    # 서명 보정이 실제 위치를 되찾아 로봇을 경로(y=0)로 데려와야 한다
    assert abs(robot.y) < 0.012


def test_lat_bias_sign_midway():
    segs = compile_leg(straight_leg(1.0, with_psd=True))
    ctrl = RepeatController(segs, RepeatGains())
    robot = Robot(y=0.03, odom_offset_y=-0.03)
    t = 0.0
    for _ in range(50):
        ox, oy, oth = robot.odom
        ctrl.step(t, ox, oy, oth, oth, 0.0, {'psd_l': WALL_Y - robot.y})
        t += DT
    assert ctrl.lat_bias > 0.01


def test_signature_gate_rejects_large_residual_and_rotation():
    segs = compile_leg(straight_leg(1.0, with_psd=True))
    ctrl = RepeatController(segs, RepeatGains())
    t = 0.0
    for _ in range(20):
        ctrl.step(t, 0.1, 0.0, 0.0, 0.0, 0.0, {'psd_l': WALL_Y - 0.5})   # 잔차 0.5 > 0.15
        t += DT
    assert ctrl.lat_bias == 0.0
    for _ in range(20):
        ctrl.step(t, 0.1, 0.0, 0.0, 0.0, 0.5, {'psd_l': WALL_Y - 0.03})  # 회전 중
        t += DT
    assert ctrl.lat_bias == 0.0


def test_deviation_abort():
    segs = compile_leg(straight_leg(1.0))
    ctrl = RepeatController(segs, RepeatGains(k_lat=0.01))   # 보정 못 하는 게인
    robot = Robot(y=0.2)
    res, t = run(ctrl, robot)
    assert res.aborted and 'deviation' in res.reason
    assert t < 1.0


def test_r_segment_turns_to_target():
    samples = []
    t = 0.0
    x = 0.0
    while x <= 0.3:
        samples.append(make_sample(t, 'T', 0, x, 0, 0, 0.25, 0, [0] * 4, 0.0, {}, True))
        t += DT
        x += 0.25 * DT
    th = 0.0
    while th < math.pi / 2:
        samples.append(make_sample(t, 'R', 1, 0.3, 0, th, 0, 1.0, [0] * 4, th, {}, False))
        t += DT
        th += DT
    segs = compile_leg(samples)
    assert [s.kind for s in segs] == ['T', 'R']
    ctrl = RepeatController(segs, RepeatGains())
    robot = Robot()
    labels = set()
    t = 0.0
    while t < 20:
        res = ctrl.step(t, robot.x, robot.y, robot.th, robot.th, 0.0, {})
        labels.add(res.seg_label)
        if res.done:
            break
        if res.seg_label.startswith('R'):
            assert res.vx == 0.0 and res.vy == 0.0
            assert abs(res.wz) <= 1.2 + 1e-9
        robot.apply(res.vx, res.vy, res.wz)
        t += DT
    assert res.done
    assert 'R1' in labels
    assert abs(wrap(robot.th - segs[1].end_yaw_rel)) < math.radians(1.5)


def test_empty_segments_done_immediately():
    ctrl = RepeatController([], RepeatGains())
    res = ctrl.step(0.0, 0, 0, 0, 0, 0, {})
    assert res.done and res.vx == 0.0
