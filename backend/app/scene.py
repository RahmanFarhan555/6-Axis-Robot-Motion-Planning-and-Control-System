from __future__ import annotations

from typing import List

from .models import ObjectBox
from .collision_world import CollisionWorld

# --- Fixed scene geometry (meters) ---

# Pickup table
TABLE1_CENTER_X = 0.35
TABLE1_CENTER_Y = 0.35
TABLE1_TOP_Z = 0.5

# Place table
TABLE2_CENTER_X = -0.35
TABLE2_CENTER_Y = 0.35
TABLE2_TOP_Z = 0.5

# Table dimensions
TABLE_SX = 0.45
TABLE_SY = 0.60
TABLE_SZ = 0.04

# Vertical bar obstacle (between tables)
BAR_CENTER_X = 0.0
BAR_CENTER_Y = 0.35

# Bar footprint in XY (thin in X, longer in Y)
BAR_SX = 0.04
BAR_SY = 0.12

BAR_HEIGHT_DEFAULT = 0.60

# --- Pick & place motion parameters (meters) ---

# Clearance above the object/table used for approach/retreat waypoints.
PICK_APPROACH_CLEARANCE_Z = 0.25

# Extra clearance above the object's top surface when solving IK for grasp/place.
GRASP_CLEARANCE_Z = 0.05


def reset_scene_obstacles(world: CollisionWorld, *, bar_height_m: float) -> None:
    """Rebuild the fixed collision scene: two tables + a vertical bar."""
    h = float(bar_height_m)

    def _table_obstacle(obstacle_id: str, *, cx: float, cy: float, top_z: float) -> dict:
        return {
            "id": obstacle_id,
            "x": float(cx),
            "y": float(cy),
            "z": float(top_z - TABLE_SZ / 2.0),
            "sx": float(TABLE_SX),
            "sy": float(TABLE_SY),
            "sz": float(TABLE_SZ),
        }

    def _bar_obstacle(height_m: float) -> dict:
        return {
            "id": "bar",
            "x": float(BAR_CENTER_X),
            "y": float(BAR_CENTER_Y),
            # stand on the ground
            "z": float(height_m / 2.0),
            "sx": float(BAR_SX),
            "sy": float(BAR_SY),
            "sz": float(height_m),
        }

    world.reset_obstacles(
        [
            _table_obstacle("table1", cx=TABLE1_CENTER_X, cy=TABLE1_CENTER_Y, top_z=TABLE1_TOP_Z),
            _table_obstacle("table2", cx=TABLE2_CENTER_X, cy=TABLE2_CENTER_Y, top_z=TABLE2_TOP_Z),
            _bar_obstacle(h),
        ]
    )


def fixed_pick_xyz(obj: ObjectBox) -> List[float]:
    """Object center position on the pickup table top."""
    return [
        float(TABLE1_CENTER_X),
        float(TABLE1_CENTER_Y),
        float(TABLE1_TOP_Z + (obj.sz / 2.0)),
    ]


def fixed_place_xyz(obj: ObjectBox) -> List[float]:
    """Object center position on the place table top."""
    return [
        float(TABLE2_CENTER_X),
        float(TABLE2_CENTER_Y),
        float(TABLE2_TOP_Z + (obj.sz / 2.0)),
    ]
