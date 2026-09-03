from studica_repeat.core.graph import LegGraph
from studica_repeat.core.leg_io import IndexEntry


def entry(a, b, length):
    return IndexEntry(a, b, f'{a}__{b}.jsonl', length, 1.0, '', 1, 10)


def test_find_and_neighbors():
    g = LegGraph([entry('A', 'B', 1.0), entry('B', 'C', 2.0), entry('A', 'C', 5.0)])
    assert g.find('A', 'B').length_m == 1.0
    assert g.find('B', 'A') is None            # 방향성
    assert sorted(g.neighbors('A')) == ['B', 'C']
    assert g.neighbors('C') == []
    assert g.nodes == ['A', 'B', 'C']


def test_shortest_path_prefers_short_chain():
    g = LegGraph([entry('A', 'B', 1.0), entry('B', 'C', 2.0), entry('A', 'C', 5.0)])
    assert g.shortest_path('A', 'C') == ['A', 'B', 'C']
    assert g.shortest_path('A', 'A') == ['A']
    assert g.shortest_path('C', 'A') is None
    assert g.shortest_path('A', 'Z') is None


def test_missing_edges():
    g = LegGraph([entry('A', 'B', 1.0)])
    assert g.missing_edges(['A', 'B', 'C', 'A']) == [('B', 'C'), ('C', 'A')]
