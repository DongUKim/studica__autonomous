from studica_teleop.terminal import ESC, KeyReader


class FakeSource:
    """주입 가능한 바이트 소스. None은 '타임아웃(입력 없음)'을 뜻한다."""

    def __init__(self, seq):
        self.seq = list(seq)
        self.calls = 0

    def __call__(self, timeout_s):
        self.calls += 1
        if not self.seq:
            return None
        return self.seq.pop(0)


def keys_from(seq, n):
    r = KeyReader(FakeSource(seq))
    return [r.read_key(0.0) for _ in range(n)]


def test_plain_key_passthrough():
    assert keys_from(['w', 'Q', ' '], 3) == ['w', 'Q', ' ']


def test_timeout_returns_none():
    assert keys_from([], 1) == [None]


def test_lone_esc_returned():
    # ESC 뒤에 아무것도 안 오면 단독 ESC 키
    assert keys_from([ESC, None, 'w'], 2) == [ESC, 'w']


def test_arrow_sequence_collapsed():
    # 위 방향키 ESC [ A → 버리고 None, 다음 키는 정상
    assert keys_from([ESC, '[', 'A', 'w'], 2) == [None, 'w']


def test_ss3_sequence_collapsed():
    assert keys_from([ESC, 'O', 'P', 's'], 2) == [None, 's']


def test_csi_with_params_collapsed():
    # Home 키 ESC [ 1 ; 5 H — 최종 바이트 H까지 소비
    assert keys_from([ESC, '[', '1', ';', '5', 'H', 'a'], 2) == [None, 'a']


def test_alt_combo_dropped():
    assert keys_from([ESC, 'x', 'd'], 2) == [None, 'd']
