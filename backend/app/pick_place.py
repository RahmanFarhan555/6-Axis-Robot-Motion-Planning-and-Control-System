from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, List, Literal, Optional

from .collision_world import CollisionWorld
from .models import ObjectBox
from .robot import RobotArm
from .scene import (
    GRASP_CLEARANCE_Z,
    PICK_APPROACH_CLEARANCE_Z,
    TABLE2_CENTER_X,
    TABLE_SX,
    fixed_pick_xyz,
    fixed_place_xyz,
)


MotionPostAction = Optional[Literal["grasp", "release"]]
MotionPostData = Optional[dict[str, Any]]


@dataclass(frozen=True)
class MotionStep:
    path: List[List[float]]
    dt: float
    post_action: MotionPostAction = None
    post_data: MotionPostData = None


class PickPlaceError(Exception):
    pass


class PickPlaceIKError(PickPlaceError):
    pass


class PickPlacePlanningError(PickPlaceError):
    pass


class PickPlacePathError(PickPlaceError):
    pass


def _approach_xyz(xyz: list[float], clearance_z: float) -> list[float]:
    return [float(xyz[0]), float(xyz[1]), float(xyz[2] + float(clearance_z))]


def _prefer_current_ee_orientation_xyzw(world: CollisionWorld, robot: RobotArm) -> Optional[List[float]]:
    ee = world.get_end_effector_pose(robot.get_joint_values())
    if ee and isinstance(ee.get("orientation"), list) and len(ee["orientation"]) == 4:
        return [float(x) for x in ee["orientation"]]
    return None


def plan_fixed_pick_place(
    *,
    world: CollisionWorld,
    robot: RobotArm,
    object_box: ObjectBox,
    move_seconds: float,
    home_joints_deg: List[float],
    solve_ik_collision_free: Callable[[List[float], Optional[List[float]]], List[float]],
    build_motion_path: Callable[[List[float], List[float], float], List[List[float]]],
) -> tuple[ObjectBox, List[MotionStep]]:
    """Plan the fixed pick&place sequence and return queued motion steps.

    This function is deliberately UI/API-agnostic:
    - It raises typed exceptions instead of HTTPException.
    - It does not mutate global state.

    Caller is responsible for:
    - Checking robot.is_moving() before calling.
    - Enqueuing returned MotionSteps into the async motion executor.
    """

    move_seconds = float(move_seconds)
    if move_seconds <= 0:
        raise PickPlacePlanningError("move_seconds must be > 0")

    # Object centers on the table surfaces.
    pick_obj_xyz = fixed_pick_xyz(object_box)
    place_obj_xyz = fixed_place_xyz(object_box)

    # IK targets should be above the cube (near its top) to avoid intersecting the table.
    pick_xyz = [
        float(pick_obj_xyz[0]),
        float(pick_obj_xyz[1]),
        float(pick_obj_xyz[2] + (object_box.sz / 2.0) + GRASP_CLEARANCE_Z),
    ]
    place_xyz = [
        float(place_obj_xyz[0]),
        float(place_obj_xyz[1]),
        float(place_obj_xyz[2] + (object_box.sz / 2.0) + GRASP_CLEARANCE_Z),
    ]

    pick_pre_xyz = _approach_xyz(pick_xyz, PICK_APPROACH_CLEARANCE_Z)

    # Prefer to keep current end-effector orientation for more predictable paths.
    prefer_orn = _prefer_current_ee_orientation_xyzw(world, robot)

    try:
        q_pick_pre = robot.set_target(solve_ik_collision_free(pick_pre_xyz, prefer_orn))
    except ValueError as e:
        raise PickPlaceIKError(f"pick_pre IK failed: {e}")

    try:
        q_pick = robot.set_target(solve_ik_collision_free(pick_xyz, prefer_orn))
    except ValueError as e:
        raise PickPlaceIKError(f"pick IK failed: {e}")

    # Choose the smallest place approach clearance that is feasible with current
    # obstacles and self-collision rules. Prefer a direct approach above the place
    # point; fall back to an entry waypoint only when needed.
    clearance_candidates = [0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]

    best_place: Optional[dict[str, Any]] = None
    last_place_err: Optional[Exception] = None

    table2_x_min = float(TABLE2_CENTER_X - (TABLE_SX / 2.0))
    place_entry_xyz = [
        float(min(table2_x_min - 0.05, place_xyz[0])),
        0.0,
        float(place_xyz[2]),
    ]

    for clearance in clearance_candidates:
        place_over_pre_xyz = _approach_xyz(place_xyz, clearance)

        try:
            q_place_over_pre = robot.set_target(solve_ik_collision_free(place_over_pre_xyz, prefer_orn))
            q_place = robot.set_target(solve_ik_collision_free(place_xyz, prefer_orn))
        except ValueError as e:
            last_place_err = e
            continue

        # Validate that the key segments are actually plannable while carrying.
        try:
            world.set_grasped_object(object_box)
            _ = build_motion_path(q_pick_pre, q_place_over_pre, move_seconds)
            _ = build_motion_path(q_place_over_pre, q_place, move_seconds)
            best_place = {
                "clearance": float(clearance),
                "use_entry": False,
                "q_place_over_pre": q_place_over_pre,
                "q_place": q_place,
            }
            break
        except ValueError as e:
            last_place_err = e
        finally:
            world.clear_grasped_object()

        # Fallback: include an entry waypoint outside the table footprint.
        place_entry_pre_xyz = _approach_xyz(place_entry_xyz, clearance)
        try:
            q_place_entry_pre = robot.set_target(solve_ik_collision_free(place_entry_pre_xyz, prefer_orn))
            world.set_grasped_object(object_box)
            _ = build_motion_path(q_pick_pre, q_place_entry_pre, move_seconds)
            _ = build_motion_path(q_place_entry_pre, q_place_over_pre, move_seconds)
            _ = build_motion_path(q_place_over_pre, q_place, move_seconds)
            best_place = {
                "clearance": float(clearance),
                "use_entry": True,
                "q_place_entry_pre": q_place_entry_pre,
                "q_place_over_pre": q_place_over_pre,
                "q_place": q_place,
            }
            break
        except ValueError as e:
            last_place_err = e
        finally:
            world.clear_grasped_object()

    if best_place is None:
        raise PickPlacePlanningError(f"place planning failed: {last_place_err}")

    q_place_over_pre = best_place["q_place_over_pre"]
    q_place = best_place["q_place"]
    q_place_entry_pre = best_place.get("q_place_entry_pre")

    # Build joint-space paths for each segment.
    try:
        path1 = build_motion_path(robot.get_joint_values(), q_pick_pre, move_seconds)
        path2 = build_motion_path(q_pick_pre, q_pick, move_seconds)

        world.set_grasped_object(object_box)
        path3 = build_motion_path(q_pick, q_pick_pre, move_seconds)

        if q_place_entry_pre is not None:
            path4 = build_motion_path(q_pick_pre, q_place_entry_pre, move_seconds)
            path5 = build_motion_path(q_place_entry_pre, q_place_over_pre, move_seconds)
        else:
            path4 = build_motion_path(q_pick_pre, q_place_over_pre, move_seconds)
            path5 = []

        path6 = build_motion_path(q_place_over_pre, q_place, move_seconds)

        world.clear_grasped_object()
        path7 = build_motion_path(q_place, q_place_over_pre, move_seconds)

        q_home = robot.set_target(list(home_joints_deg))
        if q_place_entry_pre is not None:
            path8 = build_motion_path(q_place_over_pre, q_place_entry_pre, move_seconds)
            path9 = build_motion_path(q_place_entry_pre, q_home, move_seconds)
        else:
            path8 = []
            path9 = build_motion_path(q_place_over_pre, q_home, move_seconds)

    except ValueError as e:
        raise PickPlacePathError(str(e))
    finally:
        world.clear_grasped_object()

    def _dt(path: List[List[float]]) -> float:
        return move_seconds / max(1, (len(path) - 1))

    steps: List[MotionStep] = [
        MotionStep(path=path1, dt=_dt(path1)),
        MotionStep(path=path2, dt=_dt(path2), post_action="grasp"),
        MotionStep(path=path3, dt=_dt(path3)),
        MotionStep(path=path4, dt=_dt(path4)),
    ]
    if path5:
        steps.append(MotionStep(path=path5, dt=_dt(path5)))

    steps.append(
        MotionStep(
            path=path6,
            dt=_dt(path6),
            post_action="release",
            post_data={"mode": "xyz", "pos": place_obj_xyz},
        )
    )
    steps.append(MotionStep(path=path7, dt=_dt(path7)))

    if path8:
        steps.append(MotionStep(path=path8, dt=_dt(path8)))

    steps.append(MotionStep(path=path9, dt=_dt(path9)))

    # Update the object to be *at the pick location* immediately.
    # The executor will keep it attached while grasped.
    object_box_at_pick = object_box.model_copy(update={"x": pick_obj_xyz[0], "y": pick_obj_xyz[1], "z": pick_obj_xyz[2]})

    return object_box_at_pick, steps
