from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple
import random
import math

Vector = List[float]  # 6D in degrees
StateValidFn = Callable[[Vector], bool]

def dist(a: Vector, b: Vector) -> float:
    return math.sqrt(sum((x-y)*(x-y) for x, y in zip(a, b)))

def steer(a: Vector, b: Vector, step: float) -> Vector:
    d = dist(a, b)
    if d <= step:
        return list(b)
    t = step / d
    return [x + (y-x)*t for x, y in zip(a, b)]

@dataclass
class Node:
    q: Vector
    parent: Optional[int]

class Tree:
    def __init__(self, root: Vector):
        self.nodes: List[Node] = [Node(q=list(root), parent=None)]

    def add(self, q: Vector, parent: int) -> int:
        self.nodes.append(Node(q=list(q), parent=parent))
        return len(self.nodes) - 1

    def nearest(self, q: Vector) -> int:
        best_i = 0
        best_d = dist(self.nodes[0].q, q)
        for i, n in enumerate(self.nodes):
            d = dist(n.q, q)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def path_to_root(self, idx: int) -> List[Vector]:
        out: List[Vector] = []
        while idx is not None:
            out.append(self.nodes[idx].q)
            idx = self.nodes[idx].parent  # type: ignore
        out.reverse()
        return out

def segment_is_valid(a: Vector, b: Vector, is_valid: StateValidFn, resolution_deg: float = 5.0) -> bool:
    d = dist(a, b)
    steps = max(1, int(d / resolution_deg))
    for i in range(1, steps + 1):
        t = i / steps
        q = [x + (y-x)*t for x, y in zip(a, b)]
        if not is_valid(q):
            return False
    return True

def _rrt_connect(
    start: List[float],
    goal: List[float],
    limits: List[Tuple[float, float]],
    is_valid: Callable[[List[float]], bool],
    step_deg: float = 10.0,
    max_iters: int = 8000,
    goal_bias: float = 0.25,
) -> Optional[List[List[float]]]:
    """
    Bi-directional RRT-Connect in joint space.
    Returns waypoint list if found, else None.
    """

    def clamp(q: List[float]) -> List[float]:
        return [max(lo, min(hi, v)) for v, (lo, hi) in zip(q, limits)]

    def dist(a: List[float], b: List[float]) -> float:
        return sum((aa - bb) ** 2 for aa, bb in zip(a, b)) ** 0.5

    def steer(a: List[float], b: List[float], step: float) -> List[float]:
        d = dist(a, b)
        if d <= step:
            return list(b)
        t = step / d
        return [aa + (bb - aa) * t for aa, bb in zip(a, b)]

    def sample() -> List[float]:
        if random.random() < goal_bias:
            return list(goal)
        return [random.uniform(lo, hi) for lo, hi in limits]

    # Trees: nodes + parent pointers
    Ta: List[List[float]] = [list(start)]
    Pa: List[int] = [-1]

    Tb: List[List[float]] = [list(goal)]
    Pb: List[int] = [-1]

    if not is_valid(start) or not is_valid(goal):
        return None

    def nearest(tree: List[List[float]], q: List[float]) -> int:
        best_i = 0
        best_d = dist(tree[0], q)
        for i in range(1, len(tree)):
            d = dist(tree[i], q)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def extend(tree: List[List[float]], parents: List[int], q: List[float]) -> Optional[int]:
        ni = nearest(tree, q)
        qnear = tree[ni]
        qnew = clamp(steer(qnear, q, step_deg))

        if _segment_is_valid(qnear, qnew, is_valid, resolution_deg=step_deg / 2):
            tree.append(qnew)
            parents.append(ni)
            return len(tree) - 1
        return None

    def connect(tree: List[List[float]], parents: List[int], q: List[float]) -> Optional[int]:
        last = None
        while True:
            nxt = extend(tree, parents, q)
            if nxt is None:
                return last
            last = nxt
            # If exactly reached target
            if tree[last] == q:
                return last

    def build_path(
        treeA: List[List[float]], parentA: List[int], idxA: int,
        treeB: List[List[float]], parentB: List[int], idxB: int
    ) -> List[List[float]]:
        # path from root(A) to idxA
        pathA: List[List[float]] = []
        i = idxA
        while i != -1:
            pathA.append(treeA[i])
            i = parentA[i]
        pathA.reverse()

        # path from idxB to root(B)
        pathB: List[List[float]] = []
        i = idxB
        while i != -1:
            pathB.append(treeB[i])
            i = parentB[i]

        # treeB path is from connection -> root, so append as-is
        # but avoid duplicating the connection point
        return pathA + pathB[1:]

    # Main loop: alternate which tree is “active”
    for _ in range(max_iters):
        qrand = sample()

        a_new = extend(Ta, Pa, qrand)
        if a_new is None:
            # swap roles
            Ta, Tb = Tb, Ta
            Pa, Pb = Pb, Pa
            continue

        qnew = Ta[a_new]

        b_new = connect(Tb, Pb, qnew)
        if b_new is not None and Tb[b_new] == qnew:
            # We connected: Ta[a_new] == Tb[b_new] == qnew
            # Need to build from start-root to goal-root regardless of swapping.
            if Ta[0] == start:
                return build_path(Ta, Pa, a_new, Tb, Pb, b_new)
            else:
                # swapped: Tb root is start
                return build_path(Tb, Pb, b_new, Ta, Pa, a_new)

        # swap roles each iteration (typical RRT-Connect)
        Ta, Tb = Tb, Ta
        Pa, Pb = Pb, Pa

    return None
