from __future__ import annotations
import math
import random
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

# state = 6 joint angles in degrees
State = List[float]
IsValidFn = Callable[[State], bool]

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def dist(a: State, b: State) -> float:
    # L2 distance in degrees
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def interpolate(a: State, b: State, n: int) -> List[State]:
    out = []
    for i in range(n):
        t = i / (n - 1)
        out.append([lerp(x, y, t) for x, y in zip(a, b)])
    return out

def edge_is_valid(a: State, b: State, is_valid: IsValidFn, step_deg: float = 3.0) -> bool:
    d = dist(a, b)
    steps = max(2, int(d / step_deg) + 1)
    for q in interpolate(a, b, steps):
        if not is_valid(q):
            return False
    return True

@dataclass
class Node:
    q: State
    parent: Optional[int]

class Tree:
    def __init__(self, root: State):
        self.nodes: List[Node] = [Node(root, None)]

    def add(self, q: State, parent: int) -> int:
        self.nodes.append(Node(q, parent))
        return len(self.nodes) - 1

    def nearest(self, q: State) -> int:
        best_i = 0
        best_d = float("inf")
        for i, n in enumerate(self.nodes):
            d = dist(n.q, q)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def path_to_root(self, idx: int) -> List[State]:
        path: List[State] = []
        while idx is not None:
            node = self.nodes[idx]
            path.append(node.q)
            idx = node.parent
        path.reverse()
        return path

def steer(from_q: State, to_q: State, max_step: float) -> State:
    d = dist(from_q, to_q)
    if d <= max_step:
        return list(to_q)
    t = max_step / d
    return [lerp(a, b, t) for a, b in zip(from_q, to_q)]

def rrt_connect(
    start: State,
    goal: State,
    limits: List[Tuple[float, float]],
    is_valid: IsValidFn,
    max_iters: int = 3000,
    goal_bias: float = 0.12,
    max_step_deg: float = 15.0,
    edge_step_deg: float = 3.0,
    rng_seed: Optional[int] = None,
) -> Optional[List[State]]:
    """
    RRT-Connect in joint space (degrees).
    Returns list of waypoint states (degrees) or None if fail.
    """
    if rng_seed is not None:
        random.seed(rng_seed)

    # quick rejects
    if not is_valid(start) or not is_valid(goal):
        return None
    if edge_is_valid(start, goal, is_valid, step_deg=edge_step_deg):
        return [start, goal]

    Ta = Tree(start)
    Tb = Tree(goal)

    def sample() -> State:
        if random.random() < goal_bias:
            return list(goal)
        return [random.uniform(lo, hi) for (lo, hi) in limits]

    def extend(T: Tree, q_rand: State) -> Tuple[str, int]:
        i_near = T.nearest(q_rand)
        q_near = T.nodes[i_near].q
        q_new = steer(q_near, q_rand, max_step_deg)
        # clamp to limits
        q_new = [clamp(v, lo, hi) for v, (lo, hi) in zip(q_new, limits)]
        if not edge_is_valid(q_near, q_new, is_valid, step_deg=edge_step_deg):
            return ("trapped", i_near)
        new_i = T.add(q_new, i_near)
        if dist(q_new, q_rand) < 1e-6:
            return ("reached", new_i)
        return ("advanced", new_i)

    def connect(T: Tree, q_target: State) -> Tuple[str, int]:
        status = "advanced"
        last_i = 0
        while status == "advanced":
            status, last_i = extend(T, q_target)
        return status, last_i

    for k in range(max_iters):
        q_rand = sample()

        status_a, a_new = extend(Ta, q_rand)
        if status_a != "trapped":
            status_b, b_new = connect(Tb, Ta.nodes[a_new].q)
            if status_b == "reached":
                # build full path
                path_a = Ta.path_to_root(a_new)
                path_b = Tb.path_to_root(b_new)
                path_b.reverse()
                return path_a + path_b[1:]

        # swap trees
        Ta, Tb = Tb, Ta

    return None

def shortcut_smooth(
    path: List[State],
    is_valid: IsValidFn,
    iters: int = 150,
    edge_step_deg: float = 3.0,
) -> List[State]:
    if len(path) <= 2:
        return path

    p = list(path)
    for _ in range(iters):
        if len(p) <= 2:
            break
        i = random.randint(0, len(p) - 2)
        j = random.randint(i + 1, len(p) - 1)
        if j == i + 1:
            continue
        if edge_is_valid(p[i], p[j], is_valid, step_deg=edge_step_deg):
            # remove intermediate
            p = p[: i + 1] + p[j:]
    return p
