#!/usr/bin/env python3
"""학생용 실행 스크립트 공통 모듈 (teleop.py / teach.py / repeat.py 에서 import).

역할:
  - 워크스페이스(install/setup.bash)를 찾아 ROS 환경을 자동으로 소싱한다.
    학생이 `source` 를 잊어도 `python3 scripts/teleop.py` 한 줄로 실행되게 하기 위함.
  - 하드웨어/베이스 launch 를 백그라운드로 띄우고(로그는 파일로), 키보드 노드는
    같은 터미널 전경에서 실행한다. 키보드 노드는 tty 가 필요하고, launch 로그가
    HUD 화면을 덮어쓰면 안 되기 때문에 둘을 분리한다.
  - Ctrl-C 한 번으로 전경·배경 프로세스를 모두 정리한다.
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

ROS_DISTRO_DEFAULT = "humble"
TOPIC_WAIT_TIMEOUT_S = 40.0       # 하드웨어 노드는 Titan 초기화에 수 초가 걸린다
TOPIC_POLL_INTERVAL_S = 1.0
SHUTDOWN_GRACE_S = 8.0            # SIGINT 후 SIGTERM 까지 기다리는 시간
LOG_DIR_NAME = "studica_logs"


def die(msg: str, code: int = 1) -> "NoReturn":  # type: ignore[name-defined]
    print(f"[오류] {msg}", file=sys.stderr)
    sys.exit(code)


def workspace_root() -> Path:
    """scripts/ 의 부모 = 저장소 루트 = colcon 워크스페이스 루트."""
    return Path(__file__).resolve().parent.parent


def ros_environment() -> Dict[str, str]:
    """ROS 배포판 + 워크스페이스 setup.bash 를 bash 에서 소싱한 환경변수 사전을 돌려준다.

    bash 를 한 번만 띄워 `env -0` 결과를 파싱한다. 이 사전을 subprocess 의 env 로 넘기면
    현재 파이썬 프로세스가 소싱되지 않았어도 ros2 명령이 동작한다.
    """
    ws = workspace_root()
    ws_setup = ws / "install" / "setup.bash"
    if not ws_setup.exists():
        die(
            "워크스페이스가 빌드되지 않았습니다.\n"
            f"  다음을 먼저 실행하세요 (위치: {ws}):\n"
            f"    source /opt/ros/{ROS_DISTRO_DEFAULT}/setup.bash && colcon build --symlink-install"
        )

    distro = os.environ.get("ROS_DISTRO", ROS_DISTRO_DEFAULT)
    distro_setup = Path(f"/opt/ros/{distro}/setup.bash")
    parts: List[str] = []
    if "AMENT_PREFIX_PATH" not in os.environ:
        if not distro_setup.exists():
            die(f"ROS 배포판을 찾지 못했습니다: {distro_setup}")
        parts.append(f"source '{distro_setup}'")
    parts.append(f"source '{ws_setup}'")
    parts.append("env -0")
    try:
        out = subprocess.check_output(["bash", "-c", " && ".join(parts)])
    except (OSError, subprocess.CalledProcessError) as exc:
        die(f"ROS 환경 소싱 실패: {exc}")

    env: Dict[str, str] = {}
    for item in out.split(b"\0"):
        if not item:
            continue
        key, _, value = item.decode("utf-8", errors="replace").partition("=")
        env[key] = value
    return env


def log_path(name: str) -> Path:
    """배경 launch 로그 파일 경로. HUD 를 덮지 않도록 터미널 대신 파일로 보낸다."""
    log_dir = Path.home() / LOG_DIR_NAME
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return log_dir / f"{name}_{stamp}.log"


class Background:
    """백그라운드 ros2 프로세스(launch 등). 자체 프로세스 그룹으로 띄워 한꺼번에 정리한다."""

    def __init__(self, name: str, cmd: List[str], env: Dict[str, str]):
        self.name = name
        self.cmd = cmd
        self.log = log_path(name)
        self._log_fh = open(self.log, "w", encoding="utf-8")
        print(f"[시작] {' '.join(cmd)}")
        print(f"       로그: {self.log}")
        self.proc = subprocess.Popen(
            cmd,
            env=env,
            stdout=self._log_fh,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,  # 전경 Ctrl-C 가 곧바로 전파되지 않게 분리 — 정리 순서를 우리가 제어
        )

    def alive(self) -> bool:
        return self.proc.poll() is None

    def tail(self, n: int = 20) -> str:
        try:
            self._log_fh.flush()
            lines = self.log.read_text(encoding="utf-8", errors="replace").splitlines()
            return "\n".join(lines[-n:])
        except OSError:
            return ""

    def stop(self) -> None:
        if not self.alive():
            self._log_fh.close()
            return
        pgid = os.getpgid(self.proc.pid)
        # ros2 launch 는 SIGINT 를 받아야 자식 노드들을 정상 종료(모터 0 발행)한다
        os.killpg(pgid, signal.SIGINT)
        deadline = time.monotonic() + SHUTDOWN_GRACE_S
        while self.alive() and time.monotonic() < deadline:
            time.sleep(0.2)
        if self.alive():
            print(f"[경고] {self.name} 이(가) {SHUTDOWN_GRACE_S:.0f}초 안에 안 끝나 강제 종료합니다")
            os.killpg(pgid, signal.SIGTERM)
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
        self._log_fh.close()


def _ros2_list(env: Dict[str, str], kind: str) -> List[str]:
    try:
        out = subprocess.check_output(
            ["ros2", kind, "list"], env=env, stderr=subprocess.DEVNULL, timeout=10.0
        )
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return []
    return out.decode("utf-8", errors="replace").split()


def wait_for(env: Dict[str, str], bg: Background, *, topic: Optional[str] = None,
             action: Optional[str] = None, timeout_s: float = TOPIC_WAIT_TIMEOUT_S) -> None:
    """배경 프로세스가 준비될 때까지(토픽/액션 등장) 기다린다. 죽으면 로그 꼬리를 보여주고 종료."""
    target = topic or action
    kind = "topic" if topic else "action"
    print(f"[대기] {target} 준비 중...", end="", flush=True)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not bg.alive():
            print()
            print(bg.tail())
            die(f"{bg.name} 이(가) 종료되었습니다. 위 로그를 확인하세요 (전체: {bg.log})")
        if target in _ros2_list(env, kind):
            print(" 준비됨")
            return
        print(".", end="", flush=True)
        time.sleep(TOPIC_POLL_INTERVAL_S)
    print()
    print(bg.tail())
    bg.stop()
    die(f"{timeout_s:.0f}초 안에 {target} 이(가) 나타나지 않았습니다. 하드웨어 연결과 로그를 확인하세요.")


def run_foreground(cmd: List[str], env: Dict[str, str]) -> int:
    """전경 노드 실행. Ctrl-C 는 자식에게 먼저 가고, 여기서는 정상 흐름으로 되돌린다."""
    print(f"[실행] {' '.join(cmd)}")
    print("       종료: Ctrl-C")
    try:
        return subprocess.call(cmd, env=env)
    except KeyboardInterrupt:
        return 130


def bool_arg(value: bool) -> str:
    return "true" if value else "false"


def print_banner(title: str, lines: List[str]) -> None:
    width = max(len(title), *(len(l) for l in lines)) + 4
    print("=" * width)
    print(f"  {title}")
    for line in lines:
        print(f"  {line}")
    print("=" * width)
