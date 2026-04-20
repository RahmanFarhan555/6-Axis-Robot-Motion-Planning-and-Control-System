from __future__ import annotations
from typing import List
from .planner import interpolate_cubic

def shortcut_path(path: List[List[float]], is_segment_valid, attempts: int = 200) -> List[List[float]]:
    """
    Random shortcutting: remove unnecessary waypoints if straight segment is valid.
    is_segment_valid(a,b) should collision-check the whole segment.
    """
    if len(path) <= 2:
        return path
    import random
    out = list(path)
    for _ in range(attempts):
        if len(out) <= 2:
            break
        i = random.randint(0, len(out) - 2)
        j = random.randint(i + 1, len(out) - 1)
        a, b = out[i], out[j]
        if is_segment_valid(a, b):
            out = out[: i + 1] + out[j:]
    return out

def time_parameterize_cubic(waypoints: List[List[float]], steps_per_segment: int) -> List[List[float]]:
    """
    For each segment, generate cubic interpolation samples and concatenate.
    """
    if len(waypoints) < 2:
        return waypoints
    full: List[List[float]] = []
    for k in range(len(waypoints) - 1):
        seg = interpolate_cubic(waypoints[k], waypoints[k+1], steps_per_segment)
        if k > 0:
            seg = seg[1:]  # avoid duplicates
        full.extend(seg)
    return full
