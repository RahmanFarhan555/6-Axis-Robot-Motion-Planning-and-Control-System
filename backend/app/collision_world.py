from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np
import pybullet as p
import pybullet_data

from .models import ObjectBox

def deg2rad(deg: float) -> float:
    return deg * np.pi / 180.0

def vec_deg2rad(q_deg: List[float]) -> List[float]:
    return [deg2rad(x) for x in q_deg]

@dataclass
class BulletIds:
    robot_id: int
    obstacle_ids: Dict[str, int]

class CollisionWorld:
    """
    PyBullet DIRECT world for collision queries.
    - Robot is fixed base URDF.
    - Obstacles are simple collision shapes.
    - is_state_valid(q_deg) checks robot vs obstacles (and optionally self-collision).
    """
    def __init__(
        self,
        urdf_path: str,
        base_position=(0, 0, 0),
        base_orientation_quat=(0, 0, 0, 1),
        safety_margin: float = 0.01,      # meters
        check_self_collision: bool = False,
        check_obstacles: bool = True,
    ):
        self.safety_margin = float(safety_margin)
        self.check_self_collision = bool(check_self_collision)
        self.check_obstacles = bool(check_obstacles)

        self.cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.cid)
        p.setGravity(0, 0, -9.81, physicsClientId=self.cid)

        # IMPORTANT: avoid "always-colliding" parent link contacts.
        flags = 0
        flags |= p.URDF_USE_SELF_COLLISION
        # exclude adjacent parent collisions (very important)
        if hasattr(p, "URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS"):
            flags |= p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        elif hasattr(p, "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT"):
            flags |= p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT

        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            baseOrientation=base_orientation_quat,
            useFixedBase=True,
            flags=flags,
            physicsClientId=self.cid,
        )

        # discover revolute joints (UR5 has 6)
        self.joint_indices: List[int] = []
        for j in range(p.getNumJoints(self.robot_id, physicsClientId=self.cid)):
            info = p.getJointInfo(self.robot_id, j, physicsClientId=self.cid)
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.joint_indices.append(j)
        self.joint_indices = self.joint_indices[:6]

        # Prefer a true tool/end-effector link if present (UR5 has fixed joints after wrist_3).
        # In PyBullet, getLinkState uses the joint index as the link index.
        self.ee_link_index: Optional[int] = None
        try:
            num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.cid)
            for j in range(num_joints):
                info = p.getJointInfo(self.robot_id, j, physicsClientId=self.cid)
                link_name = info[12].decode("utf-8") if isinstance(info[12], (bytes, bytearray)) else str(info[12])
                if link_name == "tool0":
                    self.ee_link_index = int(j)
                    break
        except Exception:
            self.ee_link_index = None

        if self.ee_link_index is None:
            # Fallback: last revolute link (wrist_3) if tool0 doesn't exist.
            self.ee_link_index = self.joint_indices[-1] if self.joint_indices else None

        self.obstacle_ids: Dict[str, int] = {}

        # Extra self-clearance for the end-effector link (tool0) vs the rest of the robot.
        # This helps avoid visible "gripper touching links" even when meshes are close.
        self.ee_self_clearance = max(0.0, min(0.01, self.safety_margin))

        # Optional grasped object (kinematic body) that follows the end-effector.
        self._grasped_object_id: Optional[int] = None
        self._grasped_object_box: Optional[ObjectBox] = None

        # For self-collision filtering: ignore parent-child adjacent links.
        self._parent_link: Dict[int, int] = {}
        self._num_joints: int = 0
        try:
            num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.cid)
            self._num_joints = int(num_joints)
            for j in range(num_joints):
                info = p.getJointInfo(self.robot_id, j, physicsClientId=self.cid)
                parent = int(info[16])  # parent link index
                self._parent_link[j] = parent
        except Exception:
            self._parent_link = {}
            self._num_joints = 0

    def _are_adjacent_links(self, a: int, b: int) -> bool:
        if a == b:
            return True
        pa = self._parent_link.get(a, None)
        pb = self._parent_link.get(b, None)
        return pa == b or pb == a

    def collision_reason(self) -> Optional[str]:
        """Return a human-readable reason if currently in collision, else None."""
        margin = self.safety_margin

        # --- Robot vs obstacles with safety margin ---
        if self.check_obstacles:
            for obs_id, bid in self.obstacle_ids.items():
                pts = p.getClosestPoints(
                    bodyA=self.robot_id,
                    bodyB=bid,
                    distance=margin,
                    physicsClientId=self.cid,
                )
                if pts:
                    # distance is at index 8
                    dmin = min(float(pt[8]) for pt in pts)
                    return f"obstacle '{obs_id}' within safety margin (min distance {dmin:.4f} m)"

        # --- Carried object vs obstacles with safety margin ---
        if self.check_obstacles and self._grasped_object_id is not None:
            for obs_id, bid in self.obstacle_ids.items():
                pts = p.getClosestPoints(
                    bodyA=self._grasped_object_id,
                    bodyB=bid,
                    distance=margin,
                    physicsClientId=self.cid,
                )
                if pts:
                    dmin = min(float(pt[8]) for pt in pts)
                    return f"carried object near obstacle '{obs_id}' (min distance {dmin:.4f} m)"

        # --- optional self collision ---
        if self.check_self_collision:
            # End-effector clearance check: keep tool0 from contacting/near-contacting
            # other non-adjacent links.
            if self.ee_link_index is not None and self._num_joints > 0 and self.ee_self_clearance > 0.0:
                ee = int(self.ee_link_index)
                for li in range(self._num_joints):
                    if li == ee:
                        continue
                    if self._are_adjacent_links(ee, li):
                        continue
                    pts = p.getClosestPoints(
                        bodyA=self.robot_id,
                        bodyB=self.robot_id,
                        distance=float(self.ee_self_clearance),
                        linkIndexA=ee,
                        linkIndexB=int(li),
                        physicsClientId=self.cid,
                    )
                    if pts:
                        dmin = min(float(pt[8]) for pt in pts)
                        return (
                            f"end-effector too close to link {li} "
                            f"(min distance {dmin:.4f} m, clearance {self.ee_self_clearance:.4f} m)"
                        )

            # Using getClosestPoints(robot, robot) is extremely expensive and can
            # stall planning. Instead, trigger collision detection and read
            # actual self-contact points.
            try:
                p.performCollisionDetection(physicsClientId=self.cid)
                pts = p.getContactPoints(
                    bodyA=self.robot_id,
                    bodyB=self.robot_id,
                    physicsClientId=self.cid,
                )
            except Exception:
                pts = []

            for pt in pts:
                link_a = int(pt[3])
                link_b = int(pt[4])
                # contact distance: negative when penetrating
                dist = float(pt[8]) if len(pt) > 8 else 0.0
                if self._are_adjacent_links(link_a, link_b):
                    continue
                # Treat *any* contact between non-adjacent links as invalid.
                # This prevents "self-contact" (touching) as well as penetration.
                if dist <= 0.0 + 1e-6:
                    return f"self-collision between links {link_a} and {link_b} (distance {dist:.4f} m)"

        # --- Ground penetration check ---
        try:
            num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.cid)
            link_indices = [-1] + list(range(num_joints))
            for li in link_indices:
                aabb = p.getAABB(self.robot_id, li, physicsClientId=self.cid)
                if aabb is None:
                    continue
                (amin, _amax) = aabb
                if amin[2] < margin:
                    return f"ground contact (link {li} below z={margin:.3f} m)"

            if self._grasped_object_id is not None:
                aabb = p.getAABB(self._grasped_object_id, -1, physicsClientId=self.cid)
                if aabb is not None:
                    (amin, _amax) = aabb
                    if amin[2] < margin:
                        return f"ground contact (carried object below z={margin:.3f} m)"
        except Exception:
            return "collision check failed (AABB query error)"

        return None

    def reset_obstacles(self, obstacles: List[Any]) -> None:
        # remove old
        for bid in list(self.obstacle_ids.values()):
            try:
                p.removeBody(bid, physicsClientId=self.cid)
            except Exception:
                pass
        self.obstacle_ids.clear()

        for obs in obstacles:
            bid = self._spawn_obstacle(obs)
            obs_id = getattr(obs, "id", None)
            if obs_id is None and isinstance(obs, dict):
                obs_id = obs.get("id")
            if obs_id is None:
                obs_id = str(len(self.obstacle_ids))
            self.obstacle_ids[str(obs_id)] = bid

    def set_grasped_object(self, obj: ObjectBox) -> None:
        """Create/update a collision body for a grasped object box."""
        # Recreate if size changed.
        if self._grasped_object_id is not None and self._grasped_object_box is not None:
            prev = self._grasped_object_box
            if (prev.sx, prev.sy, prev.sz) != (obj.sx, obj.sy, obj.sz):
                try:
                    p.removeBody(self._grasped_object_id, physicsClientId=self.cid)
                except Exception:
                    pass
                self._grasped_object_id = None
                self._grasped_object_box = None

        if self._grasped_object_id is None:
            half = [obj.sx / 2, obj.sy / 2, obj.sz / 2]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid)
            self._grasped_object_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=-1,
                basePosition=[obj.x, obj.y, obj.z],
                baseOrientation=[0, 0, 0, 1],
                physicsClientId=self.cid,
            )
        self._grasped_object_box = obj

    def clear_grasped_object(self) -> None:
        if self._grasped_object_id is not None:
            try:
                p.removeBody(self._grasped_object_id, physicsClientId=self.cid)
            except Exception:
                pass
        self._grasped_object_id = None
        self._grasped_object_box = None

    def _spawn_obstacle(self, obs: Any) -> int:
        # Single supported obstacle type: box
        def g(name: str, default: float = 0.0) -> float:
            if isinstance(obs, dict):
                return float(obs.get(name, default))
            return float(getattr(obs, name, default))

        half = [g("sx") / 2, g("sy") / 2, g("sz") / 2]
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[1, 0, 0, 0.35], physicsClientId=self.cid)

        bid = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[g("x"), g("y"), g("z")],
            baseOrientation=[0, 0, 0, 1],
            physicsClientId=self.cid,
        )
        return bid

    def set_joints(self, q_rad: List[float]) -> None:
        for idx, q in zip(self.joint_indices, q_rad):
            p.resetJointState(self.robot_id, idx, q, physicsClientId=self.cid)

        # If carrying an object, attach it to the end-effector pose.
        if self._grasped_object_id is not None and self.ee_link_index is not None:
            try:
                st = p.getLinkState(self.robot_id, int(self.ee_link_index), physicsClientId=self.cid)
                if st is not None:
                    pos = st[4] if len(st) > 4 else st[0]
                    orn = st[5] if len(st) > 5 else st[1]
                    p.resetBasePositionAndOrientation(
                        self._grasped_object_id,
                        posObj=list(pos),
                        ornObj=list(orn),
                        physicsClientId=self.cid,
                    )
            except Exception:
                pass

    def in_collision(self) -> bool:
        return self.collision_reason() is not None

    def is_state_valid(self, q_deg: List[float]) -> bool:
        q_rad = vec_deg2rad(q_deg)
        self.set_joints(q_rad)
        return not self.in_collision()

    def state_invalid_reason(self, q_deg: List[float]) -> Optional[str]:
        q_rad = vec_deg2rad(q_deg)
        self.set_joints(q_rad)
        return self.collision_reason()

    def get_end_effector_pose(self, q_deg: List[float]) -> Optional[dict]:
        """Return end-effector world pose (position + orientation quaternion) for given joint angles in degrees."""
        try:
            q_rad = vec_deg2rad(q_deg)
            self.set_joints(q_rad)
            if self.ee_link_index is None:
                return None
            st = p.getLinkState(self.robot_id, int(self.ee_link_index), physicsClientId=self.cid)
            if st is None:
                return None
            pos = st[4] if len(st) > 4 else st[0]
            orn = st[5] if len(st) > 5 else st[1]
            return {"position": list(pos), "orientation": list(orn)}
        except Exception:
            return None
