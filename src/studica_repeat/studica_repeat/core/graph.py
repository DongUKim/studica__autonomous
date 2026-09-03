"""티칭 레그 그래프 — index.yaml의 엣지 목록을 그래프로 보고 경로를 찾는다.

플랜B §5.2: 노드 = 체크포인트, 가중치 = 티칭 호장 길이. 레그는 방향성이 있다(N1→N2 티칭이
N2→N1을 보장하지 않는다).
"""
from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from .leg_io import IndexEntry


class LegGraph:
    def __init__(self, entries: List[IndexEntry]) -> None:
        self._edges: Dict[Tuple[str, str], IndexEntry] = {}
        self._adj: Dict[str, List[IndexEntry]] = {}
        for e in entries:
            self._edges[(e.from_node, e.to_node)] = e
            self._adj.setdefault(e.from_node, []).append(e)
            self._adj.setdefault(e.to_node, [])

    @property
    def nodes(self) -> List[str]:
        return sorted(self._adj.keys())

    def find(self, from_node: str, to_node: str) -> Optional[IndexEntry]:
        return self._edges.get((from_node, to_node))

    def neighbors(self, node: str) -> List[str]:
        return [e.to_node for e in self._adj.get(node, [])]

    def shortest_path(self, start: str, goal: str) -> Optional[List[str]]:
        """다익스트라(가중치 = length_m). 경로 없으면 None. start == goal이면 [start]."""
        if start not in self._adj or goal not in self._adj:
            return None
        dist: Dict[str, float] = {start: 0.0}
        prev: Dict[str, str] = {}
        heap: List[Tuple[float, str]] = [(0.0, start)]
        while heap:
            d, n = heapq.heappop(heap)
            if n == goal:
                break
            if d > dist.get(n, float('inf')):
                continue
            for e in self._adj.get(n, []):
                nd = d + max(e.length_m, 0.0)
                if nd < dist.get(e.to_node, float('inf')):
                    dist[e.to_node] = nd
                    prev[e.to_node] = n
                    heapq.heappush(heap, (nd, e.to_node))
        if goal not in dist:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return list(reversed(path))

    def missing_edges(self, nodes: List[str]) -> List[Tuple[str, str]]:
        """노드 열에서 티칭되지 않은 연속 쌍을 돌려준다(티칭 체크리스트용)."""
        return [(a, b) for a, b in zip(nodes, nodes[1:]) if self.find(a, b) is None]
