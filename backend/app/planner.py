from __future__ import annotations

from typing import Callable, List, Optional, Tuple
import random

# -------- interpolation (your cubic smoothstep) --------

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def smoothstep_cubic(t: float) -> float:
    return 3.0 * t * t - 2.0 * t * t * t

def interpolate_cubic(current: List[float], target: List[float], steps: int) -> List[List[float]]:
    if len(current) != 6 or len(target) != 6:
        raise ValueError("Expected 6 joint values")
    if steps < 2:
        raise ValueError("steps must be >= 2")

    path: List[List[float]] = []
    for i in range(steps):
        t = i / (steps - 1)
        s = smoothstep_cubic(t)
        path.append([lerp(c, tg, s) for c, tg in zip(current, target)])
    return path


def interpolate_linear(current: List[float], target: List[float], steps: int) -> List[List[float]]:
    if len(current) != 6 or len(target) != 6:
        raise ValueError("Expected 6 joint values")
    if steps < 2:
        raise ValueError("steps must be >= 2")

    path: List[List[float]] = []
    for i in range(steps):
        t = i / (steps - 1)
        path.append([lerp(c, tg, t) for c, tg in zip(current, target)])
    return path

# -------- helpers for collision sampling --------

def _segment_is_valid(
    a: List[float],
    b: List[float],
    is_valid: Callable[[List[float]], bool],
    resolution_deg: float = 5.0,
) -> bool:
    # sample along straight line in joint space
    diffs = [abs(bb - aa) for aa, bb in zip(a, b)]
    maxdiff = max(diffs) if diffs else 0.0
    n = max(2, int(maxdiff / resolution_deg) + 1)

    for i in range(n):
        t = i / (n - 1)
        q = [lerp(aa, bb, t) for aa, bb in zip(a, b)]
        if not is_valid(q):
            return False
    return True

def _shortcut_path(
    waypoints: List[List[float]],
    is_valid: Callable[[List[float]], bool],
    attempts: int = 250,
) -> List[List[float]]:
    if len(waypoints) <= 2:
        return waypoints

    pts = [list(w) for w in waypoints]
    for _ in range(attempts):
        if len(pts) <= 2:
            break
        i = random.randint(0, len(pts) - 2)
        j = random.randint(i + 1, len(pts) - 1)
        if j == i + 1:
            continue
        if _segment_is_valid(pts[i], pts[j], is_valid):
            pts = pts[: i + 1] + pts[j:]
    return pts


def _path_is_valid(
    path: List[List[float]],
    is_valid: Callable[[List[float]], bool],
    resolution_deg: float = 2.5,
) -> bool:
    """Validate an entire joint-space path.

    Checks:
    - each sample point is valid
    - each consecutive segment is valid with additional interpolation samples

    This is important because time-parameterization (cubic interpolation) can
    introduce collisions between sparse waypoints if not re-validated.
    """
    if not path:
        return False
    if not is_valid(path[0]) or not is_valid(path[-1]):
        return False

    for i in range(len(path)):
        if not is_valid(path[i]):
            return False
        if i > 0:
            if not _segment_is_valid(path[i - 1], path[i], is_valid, resolution_deg=resolution_deg):
                return False
    return True

# -------- very practical RRT-Connect (joint space) --------

def _rrt_connect(
    start: List[float],
    goal: List[float],
    limits: List[Tuple[float, float]],
    is_valid: Callable[[List[float]], bool],
    step_deg: float = 10.0,
    edge_resolution_deg: float = 2.5,
    max_iters: int = 8000,
    goal_bias: float = 0.25,
) -> Optional[List[List[float]]]:
    """
    Simple bi-directional RRT-Connect in joint-space.
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

    # Each tree: nodes list + parent index
    Ta = [list(start)]
    Pa = [-1]
    Tb = [list(goal)]
    Pb = [-1]

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
        if _segment_is_valid(qnear, qnew, is_valid, resolution_deg=edge_resolution_deg):
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
            if tree[last] == q:
                return last

    def build_path(
        tree_a: List[List[float]],
        parents_a: List[int],
        idx_a: int,
        tree_b: List[List[float]],
        parents_b: List[int],
        idx_b: int,
    ) -> List[List[float]]:
        """Reconstructs a full path from root(tree_a) -> connection -> root(tree_b).

        Important: tree_a/parents_a and idx_a must be consistent (same for tree_b).
        """
        path_a: List[List[float]] = []
        i = idx_a
        while i != -1:
            if i < 0 or i >= len(tree_a):
                raise ValueError("Invalid parent pointer while building path (tree_a)")
            path_a.append(tree_a[i])
            if i >= len(parents_a):
                raise ValueError("Invalid parent pointer list while building path (tree_a)")
            i = parents_a[i]
        path_a.reverse()

        path_b: List[List[float]] = []
        i = idx_b
        while i != -1:
            if i < 0 or i >= len(tree_b):
                raise ValueError("Invalid parent pointer while building path (tree_b)")
            path_b.append(tree_b[i])
            if i >= len(parents_b):
                raise ValueError("Invalid parent pointer list while building path (tree_b)")
            i = parents_b[i]

        # Avoid duplicating the connection point (tree_a[idx_a] == tree_b[idx_b])
        if path_b:
            path_b = path_b[1:]

        return path_a + path_b

    for _ in range(max_iters):
        qrand = sample()

        a_new = extend(Ta, Pa, qrand)
        if a_new is None:
            Ta, Tb = Tb, Ta
            Pa, Pb = Pb, Pa
            continue

        qnew = Ta[a_new]
        b_new = connect(Tb, Pb, qnew)
        if b_new is not None and Tb[b_new] == qnew:
            # We connected: Ta[a_new] == Tb[b_new] == qnew.
            # Build start->goal regardless of how we swapped trees.
            if Ta[0] == start:
                return build_path(Ta, Pa, a_new, Tb, Pb, b_new)
            return build_path(Tb, Pb, b_new, Ta, Pa, a_new)

        Ta, Tb = Tb, Ta
        Pa, Pb = Pb, Pa

    return None

# -------- unified entry point expected by main.py --------

def plan_path(
    *,
    current: List[float],
    target: List[float],
    steps: int,
    limits: List[Tuple[float, float]],
    is_valid: Optional[Callable[[List[float]], bool]],
    avoid_obstacles: bool = True,
) -> List[List[float]]:
    """
    Entry point used by main.py.

    If avoid_obstacles=True and is_valid is provided:
      1) Try direct cubic interpolation and validate each sample.
      2) If collision: run RRT-Connect (waypoints)
      3) Shortcut
      4) Time-parameterize with cubic interpolation between waypoints

    If avoid_obstacles=False or is_valid is None:
      - just return cubic interpolation.
    """
    # Always clamp target within limits
    target = [max(lo, min(hi, v)) for v, (lo, hi) in zip(target, limits)]

    if not avoid_obstacles or is_valid is None:
        return interpolate_cubic(current, target, steps)

    # 1) direct attempt
    direct = interpolate_cubic(current, target, steps)
    # Validate along each segment too (not just the sampled waypoints).
    if _path_is_valid(direct, is_valid, resolution_deg=2.5):
        return direct

    # 2) RRT-Connect
    wp = _rrt_connect(
        start=current,
        goal=target,
        limits=limits,
        is_valid=is_valid,
        step_deg=10.0,
        edge_resolution_deg=2.5,
        max_iters=8000,
        goal_bias=0.25,
    )
    if not wp:
        raise ValueError("No collision-free path found (RRT-Connect failed).")

    # 3) shortcut
    wp = _shortcut_path(wp, is_valid, attempts=300)

    # 4) time parameterize: distribute samples across segments
    # make at least 2 samples per segment and total ~ steps
    segments = max(1, len(wp) - 1)
    per_seg = max(2, steps // segments)

    out: List[List[float]] = []
    for i in range(segments):
        seg = interpolate_cubic(wp[i], wp[i + 1], per_seg)
        if i > 0:
            seg = seg[1:]  # avoid duplicating waypoint
        out.extend(seg)

    # Final safety check: ensure the generated trajectory really is collision-free.
    # (RRT/shortcut validates edges, but cubic re-sampling can reintroduce collisions.)
    if not _path_is_valid(out, is_valid, resolution_deg=2.5):
        raise ValueError("Planned trajectory is not collision-free after time-parameterization.")

    return out
