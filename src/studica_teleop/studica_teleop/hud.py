"""ANSI 고정 높이 HUD 렌더러 (ROS 무의존).

매 갱신마다 커서를 이전 출력 높이만큼 올려 같은 자리에 덮어쓴다. 화면 전체 clear를 하지
않아 깜빡임이 없고, 이전 로그가 위에 남는다.
"""
from __future__ import annotations

import shutil
import sys
import unicodedata
from typing import List, Optional, TextIO

CLEAR_LINE = '\x1b[2K'


def display_width(text: str) -> int:
    """터미널 셀 폭. 한글 등 동아시아 전각 문자는 2칸을 차지한다."""
    width = 0
    for ch in text:
        if unicodedata.combining(ch):
            continue
        width += 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1
    return width


def fit_width(text: str, columns: int) -> str:
    """columns 셀에 맞게 자르거나 공백으로 채운다."""
    out = []
    used = 0
    for ch in text:
        w = 0 if unicodedata.combining(ch) else (2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1)
        if used + w > columns:
            break
        out.append(ch)
        used += w
    return ''.join(out) + ' ' * (columns - used)


class Hud:
    """render(lines)를 반복 호출하면 같은 영역을 제자리에서 다시 그린다.

    lines_fixed: 최소 높이. 줄 수가 이보다 적으면 빈 줄로 채워 높이가 흔들리지 않게 한다.
    줄 수가 더 많으면 그만큼 늘어나고, 다시 줄어들면 남는 줄을 지운다.
    """

    def __init__(self, lines_fixed: int = 0, out: Optional[TextIO] = None):
        self.lines_fixed = max(0, lines_fixed)
        self._out = out if out is not None else sys.stdout
        self._last_n = 0

    def columns(self) -> int:
        return max(20, shutil.get_terminal_size(fallback=(80, 24)).columns)

    def render(self, lines: List[str]) -> None:
        cols = self.columns()
        rows = list(lines)
        if len(rows) < self.lines_fixed:
            rows += [''] * (self.lines_fixed - len(rows))
        buf = []
        if self._last_n:
            buf.append(f'\x1b[{self._last_n}A')
        for line in rows:
            buf.append('\r' + CLEAR_LINE + fit_width(line, cols - 1) + '\n')
        # 이전 출력이 더 길었으면 남는 줄을 지우고 커서를 새 영역 끝으로 되돌린다
        extra = self._last_n - len(rows)
        if extra > 0:
            buf.append(('\r' + CLEAR_LINE + '\n') * extra)
            buf.append(f'\x1b[{extra}A')
        self._out.write(''.join(buf))
        self._out.flush()
        self._last_n = len(rows)

    def reset(self) -> None:
        """다음 render가 새 영역에 그리도록 한다 (read_line 등으로 커서가 이동한 뒤 사용)."""
        self._last_n = 0

    def write_below(self, text: str) -> None:
        """HUD 바로 아래 줄에 쓴다. 호출 후 커서는 그 줄에 남는다."""
        self._out.write('\r' + CLEAR_LINE + text)
        self._out.flush()

    def clear_below(self) -> None:
        self._out.write('\r' + CLEAR_LINE)
        self._out.flush()
