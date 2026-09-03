#!/usr/bin/env python3
"""경로 재생(자율주행) 원클릭 실행 — 플랜B repeat 모드.

사용:
    python3 scripts/repeat.py mission_a N1 N2 N3          # N1→N2→N3 순서로 티칭 레그 재생
    python3 scripts/repeat.py mission_a N1 N2 --sim       # 시뮬레이터
    python3 scripts/repeat.py mission_a N1 N2 --speed 0.8 # 티칭 속도의 80 %
    python3 scripts/repeat.py mission_a --validate        # 전 레그 연속 재생 + 검증 리포트
    python3 scripts/repeat.py mission_a --list            # 티칭된 레그 목록만 출력

하는 일:
  1. install/setup.bash 자동 소싱
  2. 배경: ros2 launch studica_repeat repeat.launch.py mission:=<name>  (하드웨어 + 베이스(헤딩락 off) + 재생 제어기)
  3. 전경: ros2 run studica_repeat send_route N1 N2 ...   (FollowRoute 액션 호출, 진행상황 출력)
     --validate 이면 validate_node 가 전 레그를 순서대로 재생하고 tracking.csv / report.txt 생성
  4. Ctrl-C 로 즉시 정지·전부 종료

주의: 로봇을 첫 체크포인트(N1)의 티칭 시작 자세에 정확히 놓고 시작하세요(거치 지그 권장).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import (Background, bool_arg, die, print_banner, ros_environment,  # noqa: E402
                     run_foreground, wait_for)

DEFAULT_MISSIONS_DIR = Path.home() / "studica_missions"


def list_legs(mission: str, missions_dir: Path) -> int:
    """index.yaml 을 읽어 티칭된 레그를 보여준다. ROS 없이도 동작."""
    index = missions_dir / mission / "taught_legs" / "index.yaml"
    if not index.exists():
        die(f"티칭 파일이 없습니다: {index}\n  먼저 python3 scripts/teach.py {mission} 로 기록하세요.")
    try:
        import yaml  # PyYAML 은 ROS 배포판에 기본 포함
    except ImportError:
        print(index.read_text(encoding="utf-8"))
        return 0
    data = yaml.safe_load(index.read_text(encoding="utf-8")) or {}
    legs = data.get("legs", [])
    if not legs:
        print("티칭된 레그가 없습니다.")
        return 0
    print(f"미션 '{mission}' 티칭 레그 {len(legs)}개:")
    for leg in legs:
        print(f"  {leg.get('from')} -> {leg.get('to')}   {float(leg.get('length_m', 0)):.2f} m   "
              f"{float(leg.get('duration_s', 0)):.1f} s   v{leg.get('version', 1)}   {leg.get('recorded', '')}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="플랜B 경로 재생")
    ap.add_argument("mission", help="미션 이름 (teach.py 에서 쓴 이름)")
    ap.add_argument("nodes", nargs="*", help="체크포인트 열 (예: N1 N2 N3)")
    ap.add_argument("--sim", action="store_true", help="실기 대신 시뮬레이터 사용")
    ap.add_argument("--speed", type=float, default=1.0, help="티칭 속도 대비 배율 (0.1~1.0)")
    ap.add_argument("--validate", action="store_true", help="전 레그 연속 재생 + 검증 리포트")
    ap.add_argument("--legs", default="all", help="--validate 대상: all 또는 N1__N2,N2__N3")
    ap.add_argument("--list", action="store_true", help="티칭 레그 목록만 출력하고 종료")
    ap.add_argument("--missions-dir", default=str(DEFAULT_MISSIONS_DIR), help="티칭 파일 루트")
    ap.add_argument("--params", default=None, help="studica_control params.yaml 경로")
    args = ap.parse_args()

    missions_dir = Path(args.missions_dir).expanduser()
    if args.list:
        return list_legs(args.mission, missions_dir)
    if not args.validate and len(args.nodes) < 2:
        die("체크포인트를 2개 이상 주세요. 예: python3 scripts/repeat.py mission_a N1 N2\n"
            "  티칭된 레그 확인: python3 scripts/repeat.py mission_a --list")
    if not 0.1 <= args.speed <= 1.0:
        die("--speed 는 0.1~1.0 사이여야 합니다 (티칭 속도가 상한).")

    env = ros_environment()
    mode = "검증 모드(전 레그 연속 재생)" if args.validate else " → ".join(args.nodes)
    print_banner(f"경로 재생 — 미션 '{args.mission}'", [
        f"경로: {mode}",
        f"속도 배율: {args.speed:.2f}   모드: " + ("시뮬레이터" if args.sim else "실기 — EMS 버튼 준비"),
        "로봇을 첫 체크포인트의 티칭 시작 자세에 놓았는지 확인하세요.",
        "정지: Ctrl-C",
    ])

    launch_cmd = ["ros2", "launch", "studica_repeat", "repeat.launch.py",
                  f"mission:={args.mission}", f"sim:={bool_arg(args.sim)}",
                  f"missions_dir:={missions_dir}"]
    if args.params:
        launch_cmd.append(f"params_file:={args.params}")

    base = Background("repeat", launch_cmd, env)
    try:
        wait_for(env, base, topic="/odom")
        wait_for(env, base, action="/nav/follow_route")
        if args.validate:
            cmd = ["ros2", "run", "studica_repeat", "validate_node", "--ros-args",
                   "-p", f"mission:={args.mission}", "-p", f"missions_dir:={missions_dir}",
                   "-p", f"legs:={args.legs}", "-p", f"speed_scale:={args.speed}"]
        else:
            cmd = ["ros2", "run", "studica_repeat", "send_route", *args.nodes, "--speed", str(args.speed)]
        return run_foreground(cmd, env)
    finally:
        print("\n[정리] 재생기/베이스/하드웨어 노드 종료 중...")
        base.stop()
        print("[완료]")


if __name__ == "__main__":
    sys.exit(main())
