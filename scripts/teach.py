#!/usr/bin/env python3
"""경로 기록(티칭) 원클릭 실행 — 플랜B teach 모드.

사용:
    python3 scripts/teach.py                   # 미션 이름 기본값 mission_a, 실기
    python3 scripts/teach.py mission_a --sim   # 시뮬레이터
    python3 scripts/teach.py mission_a --missions-dir ~/studica_missions

하는 일:
  1. install/setup.bash 자동 소싱
  2. 배경: ros2 launch studica_repeat teach.launch.py mission:=<name>  (하드웨어 + 베이스, 헤딩락 on)
  3. 전경: ros2 run studica_repeat teach_node   (텔레옵 + 기록기 HUD)
       [  기록 시작 (from → to 체크포인트 입력)     ]  종료·저장
       ESC 폐기                                    r  같은 레그 재기록
  4. 결과: <missions_dir>/<mission>/taught_legs/<from>__<to>.jsonl + index.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import (Background, bool_arg, print_banner, ros_environment,  # noqa: E402
                     run_foreground, wait_for)


def main() -> int:
    ap = argparse.ArgumentParser(description="플랜B 경로 기록(티칭)")
    ap.add_argument("mission", nargs="?", default="mission_a", help="미션 이름 (폴더명)")
    ap.add_argument("--sim", action="store_true", help="실기 대신 시뮬레이터 사용")
    ap.add_argument("--missions-dir", default=None, help="티칭 파일 저장 루트 (기본 ~/studica_missions)")
    ap.add_argument("--params", default=None, help="studica_control params.yaml 경로")
    args = ap.parse_args()

    env = ros_environment()
    print_banner(f"경로 기록 — 미션 '{args.mission}'", [
        "이동: W/A/S/D   회전: Shift+Q / Shift+E   정지: Space   속도: 1/2/3",
        "기록: [ 시작   ] 종료·저장   ESC 폐기   r 재기록",
        "수칙: 레그는 정지 상태에서 시작·종료, 회전은 꼭 필요할 때만, 벽 15~25 cm 유지",
        "모드: " + ("시뮬레이터" if args.sim else "실기 — EMS 버튼을 손 닿는 곳에 두세요"),
    ])

    launch_cmd = ["ros2", "launch", "studica_repeat", "teach.launch.py",
                  f"mission:={args.mission}", f"sim:={bool_arg(args.sim)}"]
    node_params = ["-p", f"mission:={args.mission}"]
    if args.missions_dir:
        launch_cmd.append(f"missions_dir:={args.missions_dir}")
        node_params += ["-p", f"missions_dir:={args.missions_dir}"]
    if args.params:
        launch_cmd.append(f"params_file:={args.params}")

    base = Background("teach", launch_cmd, env)
    try:
        wait_for(env, base, topic="/odom")
        return run_foreground(
            ["ros2", "run", "studica_repeat", "teach_node", "--ros-args", *node_params], env)
    finally:
        print("\n[정리] 베이스/하드웨어 노드 종료 중...")
        base.stop()
        print("[완료]")


if __name__ == "__main__":
    sys.exit(main())
