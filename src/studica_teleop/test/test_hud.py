import io

from studica_teleop.hud import Hud, display_width, fit_width


def test_display_width_counts_korean_as_two():
    assert display_width('abc') == 3
    assert display_width('한글') == 4
    assert display_width('a한') == 3


def test_fit_width_pads_and_truncates():
    assert fit_width('ab', 5) == 'ab   '
    assert fit_width('abcdef', 3) == 'abc'
    # 전각 문자가 경계를 넘으면 자르고 공백으로 채운다
    assert fit_width('한글', 3) == '한 '


class FixedHud(Hud):
    def columns(self):
        return 21   # fit_width에 cols-1 = 20 이 들어간다


def test_render_moves_cursor_up_on_second_call():
    out = io.StringIO()
    hud = FixedHud(lines_fixed=3, out=out)
    hud.render(['a'])
    first = out.getvalue()
    assert '\x1b[3A' not in first          # 첫 렌더는 커서 이동 없음
    assert first.count('\n') == 3          # 최소 높이 3줄로 패딩
    out.truncate(0); out.seek(0)
    hud.render(['b', 'c'])
    second = out.getvalue()
    assert second.startswith('\x1b[3A')    # 이전 높이만큼 위로
    assert second.count('\n') == 3


def test_render_clears_extra_lines_when_shrinking():
    out = io.StringIO()
    hud = FixedHud(lines_fixed=0, out=out)
    hud.render(['1', '2', '3', '4'])
    out.truncate(0); out.seek(0)
    hud.render(['x'])
    s = out.getvalue()
    assert s.startswith('\x1b[4A')
    assert s.endswith('\x1b[3A')           # 지운 3줄만큼 다시 올라와 영역 끝에 정렬
