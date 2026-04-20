from __future__ import annotations

import asyncio
import random
from typing import List, Optional, Set, Literal, Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware

import pybullet as p

from .robot import RobotArm, Joint
from .planner import plan_path, interpolate_cubic
from .scene import (
    TABLE1_CENTER_X,
    TABLE1_CENTER_Y,
    TABLE1_TOP_Z,
    BAR_HEIGHT_DEFAULT,
    reset_scene_obstacles,
    fixed_pick_xyz,
    fixed_place_xyz,
)
from .models import (
    RobotState,
    SetTargetRequest,
    PlaceRequest,
    CartesianPoseRequest,
    JogRequest,
    ObjectBox,
    SetObjectRequest,
    BarHeightRequest,
)
from .settings import settings
from .collision_world import CollisionWorld
from .pick_place import (
    plan_fixed_pick_place,
    PickPlaceIKError,
    PickPlacePlanningError,
    PickPlacePathError,
)

app = FastAPI(title="6-Axis Robot Control")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
def api_root():
    return {
        "status": "ok",
        "docs": "/docs",
        "openapi": "/openapi.json",
        "state": "/api/state",
    }

robot = RobotArm([
    Joint("shoulder_pan_joint",   -360.0, 360.0, 0.0),
    Joint("shoulder_lift_joint",  -360.0, 360.0, 0.0),
    Joint("elbow_joint",          -360.0, 360.0, 0.0),
    Joint("wrist_1_joint",        -360.0, 360.0, 0.0),
    Joint("wrist_2_joint",        -360.0, 360.0, 0.0),
    Joint("wrist_3_joint",        -360.0, 360.0, 0.0),
])

UR5_URDF_PATH = "/app/app/assets/ur5.urdf"

# Do not rotate the URDF in PyBullet unless you also rotate obstacles.
ROBOT_BASE_Z = 0.5
world = CollisionWorld(
    UR5_URDF_PATH,
    base_position=(0, 0, ROBOT_BASE_Z),
    base_orientation_quat=(0, 0, 0, 1),
    safety_margin=0.005,
    check_self_collision=True,
    check_obstacles=True,
)

bar_height_m: float = float(BAR_HEIGHT_DEFAULT)


def _reset_scene_obstacles() -> None:
    """Rebuild the fixed collision scene: two tables + a vertical bar."""
    reset_scene_obstacles(world, bar_height_m=float(bar_height_m))


_reset_scene_obstacles()

clients: Set[WebSocket] = set()

MotionPostAction = Optional[Literal["grasp", "release"]]
MotionPostData = Optional[dict[str, Any]]
motion_queue: asyncio.Queue[tuple[list[list[float]], float, MotionPostAction, MotionPostData]] = asyncio.Queue()

# Simple pick-and-place object state (fixed initial location on table 1)
object_box: Optional[ObjectBox] = ObjectBox(
    id="object",
    x=float(TABLE1_CENTER_X),
    y=float(TABLE1_CENTER_Y),
    z=float(TABLE1_TOP_Z + 0.06 / 2.0),
    sx=0.06,
    sy=0.06,
    sz=0.06,
)
grasped: bool = False

# Fixed joint target used for placing (degrees) (kept as a fallback)
PLACE_JOINTS_DEG = [45.0, -60.0, 80.0, -100.0, 0.0, 0.0]

# Fixed Home pose (degrees) — matches frontend "Home pose".
HOME_JOINTS_DEG = [90.0, -90.0, 90.0, -90.0, 0.0, 0.0]

# Default "startup" pose of the simulation (degrees).
# Keeping this as a seed helps produce consistent IK solutions when starting
# pick&place from different poses (e.g. Home vs initial zeros).
START_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# (FK-recorded poses are no longer the primary UX; retained internally as a fallback)
pick_pose_joints_deg: Optional[List[float]] = None
place_pose_joints_deg: List[float] = list(PLACE_JOINTS_DEG)
place_pose_user_set: bool = False


def _fixed_pick_xyz(obj: ObjectBox) -> list[float]:
    return fixed_pick_xyz(obj)


def _fixed_place_xyz(obj: ObjectBox) -> list[float]:
    return fixed_place_xyz(obj)


def _solve_ik_prefer_orientation(target_pos_xyz: List[float], prefer_orn_xyzw: Optional[List[float]]) -> List[float]:
    """Solve IK, preferring to keep a given end-effector orientation.

    If orientation-constrained IK fails (can happen near joint limits), fall back
    to position-only IK for robustness.
    """
    if prefer_orn_xyzw is not None:
        try:
            return solve_ik_joints_deg(target_pos_xyz, prefer_orn_xyzw)
        except Exception:
            pass
    return solve_ik_joints_deg(target_pos_xyz)


def _rad2deg(rad: float) -> float:
    return float(rad) * 180.0 / 3.141592653589793


def solve_ik_joints_deg(
    target_pos_xyz: List[float],
    target_orn_xyzw: Optional[List[float]] = None,
    *,
    rest_deg: Optional[List[float]] = None,
) -> List[float]:
    """Solve IK in PyBullet and return 6 joint angles in degrees.

    If target_orn_xyzw is omitted, solves position-only IK (more robust for the demo).
    """
    # Start near current pose for stability.
    current_deg = robot.get_joint_values()
    world.set_joints([q * 3.141592653589793 / 180.0 for q in current_deg])

    if not world.joint_indices:
        raise ValueError("IK unavailable: no revolute joints found")
    ee_index = getattr(world, "ee_link_index", None)
    if ee_index is None:
        ee_index = world.joint_indices[-1]

    limits_deg = robot.get_limits()
    lower = [lo * 3.141592653589793 / 180.0 for (lo, _hi) in limits_deg]
    upper = [hi * 3.141592653589793 / 180.0 for (_lo, hi) in limits_deg]
    ranges = [max(1e-6, u - l) for l, u in zip(lower, upper)]
    if rest_deg is None:
        rest_deg = current_deg
    rest = [q * 3.141592653589793 / 180.0 for q in rest_deg]

    ik_kwargs = dict(
        bodyUniqueId=world.robot_id,
        endEffectorLinkIndex=ee_index,
        targetPosition=list(map(float, target_pos_xyz)),
        lowerLimits=lower,
        upperLimits=upper,
        jointRanges=ranges,
        restPoses=rest,
        maxNumIterations=100,
        residualThreshold=1e-4,
        physicsClientId=world.cid,
    )
    if target_orn_xyzw is not None:
        ik_kwargs["targetOrientation"] = list(map(float, target_orn_xyzw))

    sol = p.calculateInverseKinematics(**ik_kwargs)

    # PyBullet can return either:
    # - a tuple sized for *all* joints (indexable by joint index), OR
    # - a tuple sized only for movable joints in order (often length 6 here).
    sol_list = list(sol)
    if not world.joint_indices:
        raise ValueError("IK unavailable: joint index list is empty")

    max_joint_index = max(world.joint_indices)
    if len(sol_list) >= (max_joint_index + 1):
        # Indexable by joint index.
        q_rad = [sol_list[j] for j in world.joint_indices]
    else:
        # Fall back to sequential mapping.
        if len(sol_list) < len(world.joint_indices):
            raise ValueError(
                f"IK returned {len(sol_list)} joint values, expected at least {len(world.joint_indices)}"
            )
        q_rad = sol_list[: len(world.joint_indices)]

    return [_rad2deg(q) for q in q_rad]


def _clamp_joints_deg(q_deg: List[float]) -> List[float]:
    limits = robot.get_limits()
    return [max(lo, min(hi, v)) for v, (lo, hi) in zip(q_deg, limits)]


def _solve_ik_collision_free(target_pos_xyz: List[float], prefer_orn_xyzw: Optional[List[float]]) -> List[float]:
    """Try multiple IK seeds and choose the best collision-free solution.

    Key idea: IK can produce multiple valid configurations for the same Cartesian
    target. Picking the *first* collision-free one can yield odd-looking paths
    from certain starting poses (e.g. Home). Instead, pick the collision-free
    solution that is closest to the current joint state.
    """

    current = robot.get_joint_values()
    # IK is local; broaden the search with a small set of diverse rest-pose seeds.
    rest_candidates: List[List[float]] = [
        list(current),
        list(HOME_JOINTS_DEG),
        list(START_JOINTS_DEG),

        # elbow-up-ish variants
        [90.0, -70.0, 100.0, -120.0, 0.0, 0.0],
        [90.0, -110.0, 120.0, -100.0, 0.0, 0.0],
        [60.0, -90.0, 110.0, -110.0, 0.0, 0.0],

        # more tucked variants
        [120.0, -120.0, 140.0, -110.0, 0.0, 0.0],
    ]

    def joint_distance(a: List[float], b: List[float]) -> float:
        return sum((aa - bb) ** 2 for aa, bb in zip(a, b)) ** 0.5

    best: Optional[List[float]] = None
    best_score: float = 1e18
    last_reason: Optional[str] = None

    def consider(ik_deg: List[float]) -> None:
        nonlocal best, best_score, last_reason
        ik_deg = _clamp_joints_deg(ik_deg)
        reason = world.state_invalid_reason(ik_deg)
        if reason is not None:
            last_reason = reason
            return
        score = joint_distance(current, ik_deg)
        if score < best_score:
            best = ik_deg
            best_score = score

    # Pass 1: orientation preferred.
    if prefer_orn_xyzw is not None:
        for rest in rest_candidates:
            try:
                consider(solve_ik_joints_deg(target_pos_xyz, prefer_orn_xyzw, rest_deg=rest))
            except Exception:
                continue

    # Pass 2: position-only fallback.
    for rest in rest_candidates:
        try:
            consider(solve_ik_joints_deg(target_pos_xyz, None, rest_deg=rest))
        except Exception:
            continue

    # Pass 3: randomized rest-pose sampling (bounded) to escape local minima.
    limits = robot.get_limits()
    for _ in range(40):
        rand_rest = [random.uniform(lo, hi) for (lo, hi) in limits]
        if prefer_orn_xyzw is not None:
            try:
                consider(solve_ik_joints_deg(target_pos_xyz, prefer_orn_xyzw, rest_deg=rand_rest))
            except Exception:
                pass
        try:
            consider(solve_ik_joints_deg(target_pos_xyz, None, rest_deg=rand_rest))
        except Exception:
            continue

    if best is not None:
        return best

    if last_reason is None:
        last_reason = "IK failed to produce any candidate solution"
    raise ValueError(f"No collision-free IK solution: {last_reason}")

def get_state() -> RobotState:
    global object_box, grasped

    obj = object_box
    if obj is not None and grasped:
        ee = world.get_end_effector_pose(robot.get_joint_values())
        if ee and isinstance(ee.get("position"), list) and len(ee["position"]) >= 3:
            x, y, z = float(ee["position"][0]), float(ee["position"][1]), float(ee["position"][2])
            obj = obj.model_copy(update={"x": x, "y": y, "z": z})

    return RobotState(
        joints_deg=robot.get_joint_values(),
        moving=robot.is_moving(),
        target_deg=robot.get_target(),
        object=obj,
        grasped=grasped,
        bar_height=float(bar_height_m),
    )


@app.get("/api/bar_height")
def api_get_bar_height():
    return {"height": float(bar_height_m)}


@app.post("/api/bar_height")
def api_set_bar_height(req: BarHeightRequest):
    global bar_height_m
    bar_height_m = float(req.height)
    _reset_scene_obstacles()
    return {"height": float(bar_height_m)}

@app.get("/api/state", response_model=RobotState)
def api_get_state():
    return get_state()

def build_motion_path(current: List[float], target: List[float], move_seconds: float) -> List[List[float]]:
    steps = max(2, int(move_seconds * settings.tick_hz))

    # Reject unsafe targets instead of executing collision-ignoring motion.
    reason = world.state_invalid_reason(target)
    if reason is not None:
        raise ValueError(f"Target is in collision: {reason}")

    return plan_path(
        current=current,
        target=target,
        steps=steps,
        limits=robot.get_limits(),
        is_valid=world.is_state_valid,
        avoid_obstacles=True,
    )


def build_jog_path(current: List[float], target: List[float], move_seconds: float) -> List[List[float]]:
    """Jog is intentionally constrained: only the requested joint should move.

    If that direct motion would collide, we reject instead of running RRT-Connect
    (which can move other joints to go around obstacles).
    """
    steps = max(2, int(move_seconds * settings.tick_hz))
    path = interpolate_cubic(current, target, steps)
    for idx, q in enumerate(path):
        reason = world.state_invalid_reason(q)
        if reason is not None:
            raise ValueError(
                f"Jog path is in collision at step {idx + 1}/{len(path)}: {reason}. "
                "Adjust pose/obstacles or use Move to targets."
            )
    return path

@app.post("/api/target", response_model=RobotState)
async def api_set_target(req: SetTargetRequest):
    clamped = robot.set_target(req.joints_deg)
    current = robot.get_joint_values()
    target = clamped

    move_seconds = float(req.move_seconds)
    try:
        path = build_motion_path(current, target, move_seconds)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

    dt = move_seconds / max(1, (len(path) - 1))
    await motion_queue.put((path, dt, None, None))
    return get_state()

@app.post("/api/jog", response_model=RobotState)
async def api_jog(req: JogRequest):
    current = robot.get_joint_values()
    raw_target = list(current)
    raw_target[req.joint_index] += req.delta_deg

    clamped = robot.set_target(raw_target)
    target = clamped

    move_seconds = float(req.move_seconds)
    try:
        path = build_jog_path(current, target, move_seconds)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

    dt = move_seconds / max(1, (len(path) - 1))
    await motion_queue.put((path, dt, None, None))
    return get_state()


@app.post("/api/object", response_model=RobotState)
def api_set_object(req: SetObjectRequest):
    # Object is fixed for the challenge scenario.
    raise HTTPException(status_code=400, detail="Object is fixed for the scenario; cannot be edited")


@app.post("/api/pick_pose", response_model=RobotState)
def api_set_pick_pose():
    """Record the current robot pose as the pick pose (Option 2 / FK)."""
    global pick_pose_joints_deg
    pick_pose_joints_deg = robot.get_joint_values()
    return get_state()


@app.post("/api/place_pose", response_model=RobotState)
def api_set_place_pose():
    """Record the current robot pose as the place pose (Option 2 / FK)."""
    global place_pose_joints_deg, place_pose_user_set
    place_pose_joints_deg = robot.get_joint_values()
    place_pose_user_set = True
    return get_state()


@app.post("/api/pick", response_model=RobotState)
async def api_pick(req: PlaceRequest):
    global object_box
    if object_box is None:
        raise HTTPException(status_code=400, detail="No object set")

    raise HTTPException(status_code=400, detail="Use the fixed task endpoint /api/run_pick_place")


@app.post("/api/place", response_model=RobotState)
async def api_place(req: PlaceRequest):
    global object_box, grasped
    raise HTTPException(status_code=400, detail="Use the fixed task endpoint /api/run_pick_place")


@app.post("/api/run_pick_place", response_model=RobotState)
async def api_run_pick_place(req: PlaceRequest):
    """Run the fixed scenario: pick from table1 and place on table2."""
    global object_box, grasped

    if robot.is_moving():
        raise HTTPException(status_code=409, detail="Robot is currently moving")

    world.clear_grasped_object()

    # Ensure object exists at the table1 pick location.
    if object_box is None:
        object_box = ObjectBox(id="object", x=TABLE1_CENTER_X, y=TABLE1_CENTER_Y, z=TABLE1_TOP_Z + 0.03, sx=0.06, sy=0.06, sz=0.06)
    move_seconds = float(req.move_seconds)

    try:
        object_box, steps = plan_fixed_pick_place(
            world=world,
            robot=robot,
            object_box=object_box,
            move_seconds=move_seconds,
            home_joints_deg=list(HOME_JOINTS_DEG),
            solve_ik_collision_free=_solve_ik_collision_free,
            build_motion_path=build_motion_path,
        )
    except PickPlaceIKError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except PickPlacePlanningError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except PickPlacePathError as e:
        raise HTTPException(status_code=409, detail=str(e))

    grasped = False

    for step in steps:
        await motion_queue.put((step.path, step.dt, step.post_action, step.post_data))
    return get_state()


@app.post("/api/pick_xyz", response_model=RobotState)
async def api_pick_xyz(req: CartesianPoseRequest):
    """Pick using a Cartesian target (x,y,z) in world frame. Uses IK + collision-checked planning."""
    global object_box
    # Ensure an object exists; use provided xyz as the object center.
    if object_box is None:
        object_box = ObjectBox(id="object", x=req.x, y=req.y, z=req.z, sx=0.06, sy=0.06, sz=0.06)
    else:
        object_box = object_box.model_copy(update={"x": float(req.x), "y": float(req.y), "z": float(req.z)})

    # Keep current end-effector orientation, move only position.
    ee = world.get_end_effector_pose(robot.get_joint_values())
    orn = [0.0, 0.0, 0.0, 1.0]
    if ee and isinstance(ee.get("orientation"), list) and len(ee["orientation"]) == 4:
        orn = [float(x) for x in ee["orientation"]]

    try:
        ik_deg = solve_ik_joints_deg([req.x, req.y, req.z], orn)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    clamped = robot.set_target(ik_deg)
    current = robot.get_joint_values()
    target = clamped

    move_seconds = float(req.move_seconds)
    try:
        path = build_motion_path(current, target, move_seconds)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

    dt = move_seconds / max(1, (len(path) - 1))
    await motion_queue.put((path, dt, "grasp", None))
    return get_state()


@app.post("/api/place_xyz", response_model=RobotState)
async def api_place_xyz(req: CartesianPoseRequest):
    """Place using a Cartesian target (x,y,z) in world frame. Uses IK + collision-checked planning."""
    global object_box, grasped
    if object_box is None:
        raise HTTPException(status_code=400, detail="No object set")
    if not grasped:
        raise HTTPException(status_code=409, detail="Nothing is currently grasped")

    ee = world.get_end_effector_pose(robot.get_joint_values())
    orn = [0.0, 0.0, 0.0, 1.0]
    if ee and isinstance(ee.get("orientation"), list) and len(ee["orientation"]) == 4:
        orn = [float(x) for x in ee["orientation"]]

    try:
        ik_deg = solve_ik_joints_deg([req.x, req.y, req.z], orn)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    clamped = robot.set_target(ik_deg)
    current = robot.get_joint_values()
    target = clamped

    move_seconds = float(req.move_seconds)
    try:
        path = build_motion_path(current, target, move_seconds)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e))

    dt = move_seconds / max(1, (len(path) - 1))
    await motion_queue.put((path, dt, "release", {"mode": "xyz", "pos": [float(req.x), float(req.y), float(req.z)]}))
    return get_state()

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        await ws.send_json(get_state().model_dump())
        while True:
            msg = await ws.receive_text()
            if msg.strip().lower() == "ping":
                await ws.send_text("pong")
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)

async def broadcast_state_loop():
    period = 1.0 / settings.ws_broadcast_hz
    while True:
        if clients:
            payload = get_state().model_dump()
            dead = []
            for c in list(clients):
                try:
                    await c.send_json(payload)
                except Exception:
                    dead.append(c)
            for d in dead:
                clients.discard(d)
        await asyncio.sleep(period)

async def motion_executor_loop():
    global object_box, grasped, place_pose_user_set
    while True:
        path, dt, post_action, post_data = await motion_queue.get()
        robot.set_moving(True)
        try:
            for step in path:
                robot.set_joint_values(step)
                await asyncio.sleep(dt)
            robot.set_joint_values(path[-1])

            if post_action in ("grasp", "release") and object_box is not None:
                if post_action == "grasp":
                    # Snap object to end-effector and mark grasped.
                    ee = world.get_end_effector_pose(robot.get_joint_values())
                    if ee and isinstance(ee.get("position"), list) and len(ee["position"]) >= 3:
                        x, y, z = float(ee["position"][0]), float(ee["position"][1]), float(ee["position"][2])
                        object_box = object_box.model_copy(update={"x": x, "y": y, "z": z})
                    grasped = True
                    # Enable carried-object collision checks for any future planning.
                    world.set_grasped_object(object_box)

                elif post_action == "release":
                    mode = (post_data or {}).get("mode") if isinstance(post_data, dict) else None
                    if mode == "xyz":
                        pos = (post_data or {}).get("pos")
                        if isinstance(pos, list) and len(pos) >= 3:
                            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
                            object_box = object_box.model_copy(update={"x": x, "y": y, "z": z})
                    elif mode == "pose":
                        # Recorded place pose: drop at end-effector.
                        ee = world.get_end_effector_pose(robot.get_joint_values())
                        if ee and isinstance(ee.get("position"), list) and len(ee["position"]) >= 3:
                            x, y, z = float(ee["position"][0]), float(ee["position"][1]), float(ee["position"][2])
                            object_box = object_box.model_copy(update={"x": x, "y": y, "z": z})
                    else:
                        # Fixed table2 fallback.
                        pos = _fixed_place_xyz(object_box)
                        object_box = object_box.model_copy(update={"x": pos[0], "y": pos[1], "z": pos[2]})
                    grasped = False
                    world.clear_grasped_object()
        finally:
            robot.set_moving(False)
            motion_queue.task_done()

@app.on_event("startup")
async def startup():
    # Ensure the simulation starts from the same "Home" pose the UI expects.
    # Without this, RobotArm defaults to zeros, which looks like a random pose.
    robot.set_joint_values(list(HOME_JOINTS_DEG))
    robot.set_target(list(HOME_JOINTS_DEG))

    # Ensure fixed obstacles (tables + bar) are present after startup.
    _reset_scene_obstacles()
    asyncio.create_task(motion_executor_loop())
    asyncio.create_task(broadcast_state_loop())

@app.get("/api/debug/joint_axes")
def api_debug_joint_axes():
    axes = []
    try:
        num_joints = p.getNumJoints(world.robot_id, physicsClientId=world.cid)
        for j in range(num_joints):
            info = p.getJointInfo(world.robot_id, j, physicsClientId=world.cid)
            if info[2] == p.JOINT_REVOLUTE:
                name = info[1].decode("utf-8") if isinstance(info[1], bytes) else info[1]
                axis = list(info[13]) if len(info) > 13 and info[13] is not None else [0.0, 0.0, 0.0]
                axes.append({"index": j, "name": name, "axis": axis})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    return {"axes": axes}
