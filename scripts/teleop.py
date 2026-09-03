#!/usr/bin/env python3
"""키보드 텔레옵 원클릭 실행.

사용 (VMX-pi 터미널, 저장소 루트 어디서든):
    python3 scripts/teleop.py            # 실기
    python3 scripts/teleop.py --sim      # 시뮬레이터(하드웨어 없이)

하는 일:
  1. install/setup.bash 자동 소싱
  2. 배경: ros2 launch studica_base base.launch.py  (하드웨어 + cmd_vel→모터 베이스)
  3. 전경: ros2 run studica_teleop keyboard_teleop   (WASD / Shift+Q,E / 1,2,3 / space)
  4. Ctrl-C 로 전부 종료
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import (Background, bool_arg, print_banner, ros_environment,  # noqa: E402
                     run_foreground, wait_for)


def main() -> int:
    ap = argparse.ArgumentParser(description="Studica 키보드 텔레옵")
    ap.add_argument("--sim", action="store_true", help="실기 대신 시뮬레이터 사용")
    ap.add_argument("--params", default=None, help="studica_control params.yaml 경로 (기본: studica_base/config/params.yaml)")
    args = ap.parse_args()

    env = ros_environment()
    print_banner("키보드 텔레옵", [
        "이동: W/A/S/D   회전: Shift+Q / Shift+E   정지: Space",
        "속도: 1(저) 2(중) 3(고)  +/-  |  yaw·odom 영점: z  |  도움말: h",
        "모드: " + ("시뮬레이터" if args.sim else "실기 — EMS 버튼을 손 닿는 곳에 두세요"),
    ])

    launch_cmd = ["ros2", "launch", "studica_base", "base.launch.py", f"sim:={bool_arg(args.sim)}"]
    if args.params:
        launch_cmd.append(f"params_file:={args.params}")
    base = Background("base", launch_cmd, env)
    try:
        wait_for(env, base, topic="/odom")
        return run_foreground(["ros2", "run", "studica_teleop", "keyboard_teleop"], env)
    finally:
        print("\n[정리] 베이스/하드웨어 노드 종료 중...")
        base.stop()
        print("[완료]")


if __name__ == "__main__":
    sys.exit(main())
