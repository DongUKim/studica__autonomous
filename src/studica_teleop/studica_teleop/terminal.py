"""raw 터미널 키 입력 도우미 (ROS 무의존).

termios/tty는 Linux 전용이라 모듈 임포트 시점이 아니라 RawTerminal 진입 시점에만
임포트한다 — Windows 개발 PC에서도 KeyReader 로직을 단위 테스트할 수 있어야 하기 때문.
"""
from __future__ import annotations

import sys
from typing import Callable, Optional

ESC = '\x1b'
# 방향키 등 ESC 시퀀스는 ESC 직후 수 ms 안에 나머지 바이트가 따라온다.
# 사람이 ESC 키를 단독으로 누른 뒤 20 ms 안에 '['를 치는 일은 없다고 본다.
ESC_SEQ_WAIT_S = 0.02

ReadByteFn = Callable[[float], Optional[str]]


def _stdin_read_byte(timeout_s: float) -> Optional[str]:
    """stdin에서 1바이트를 timeout 안에 읽는다. 없으면 None."""
    import select
    ready, _, _ = select.select([sys.stdin], [], [], max(0.0, timeout_s))
    if not ready:
        return None
    ch = sys.stdin.read(1)
    return ch if ch else None


class KeyReader:
    """바이트 소스에서 키 하나를 읽는다. ESC 시퀀스(방향키 등)는 통째로 버리고 None을 돌려준다.

    read_byte를 주입할 수 있게 해서 실제 tty 없이 테스트한다.
    """

    def __init__(self, read_byte: Optional[ReadByteFn] = None):
        self._read = read_byte if read_byte is not None else _stdin_read_byte

    def read_key(self, timeout_s: float) -> Optional[str]:
        ch = self._read(timeout_s)
        if ch is None:
            return None
        if ch != ESC:
            return ch
        nxt = self._read(ESC_SEQ_WAIT_S)
        if nxt is None:
            return ESC  # 단독 ESC 키
        if nxt in '[O':
            # CSI/SS3 시퀀스: 최종 바이트(0x40~0x7E)까지 소비
            while True:
                c = self._read(ESC_SEQ_WAIT_S)
                if c is None or '@' <= c <= '~':
                    break
            return None
        # ESC + 일반 문자(Alt 조합 등)는 의도한 키가 아니므로 버린다
        return None


class RawTerminal:
    """stdin을 cbreak 모드로 전환하는 컨텍스트 매니저.

    setraw가 아니라 setcbreak를 쓴다: ISIG가 살아 있어 Ctrl-C가 KeyboardInterrupt로 오고,
    OPOST가 살아 있어 '\\n' 출력이 그대로 동작한다.
    """

    def __init__(self):
        self._fd = None
        self._saved = None

    def __enter__(self) -> 'RawTerminal':
        import termios
        import tty
        if not sys.stdin.isatty():
            raise RuntimeError('stdin이 tty가 아닙니다 (ssh -t 로 접속하세요)')
        self._fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fd is not None and self._saved is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        self._fd = None
        self._saved = None


def read_key(timeout_s: float, reader: Optional[KeyReader] = None) -> Optional[str]:
    """모듈 수준 편의 함수. reader를 주지 않으면 stdin을 쓴다."""
    return (reader or KeyReader()).read_key(timeout_s)
