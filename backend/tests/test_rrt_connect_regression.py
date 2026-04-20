from __future__ import annotations

import pytest

from app.planner import _rrt_connect


def test_rrt_connect_does_not_index_error_when_trees_swap() -> None:
    # Construct a trivial always-valid space so RRT-Connect can connect.
    limits = [(-10.0, 10.0)] * 6
    start = [0.0] * 6
    goal = [1.0] * 6

    def is_valid(q: list[float]) -> bool:
        return True

    # The regression we care about: never crash with IndexError during build_path,
    # even with repeated tree swapping.
    for _ in range(25):
        path = _rrt_connect(
            start=start,
            goal=goal,
            limits=limits,
            is_valid=is_valid,
            step_deg=10.0,
            max_iters=200,
            goal_bias=0.5,
        )
        assert path is not None
        assert path[0] == start
        assert path[-1] == goal
