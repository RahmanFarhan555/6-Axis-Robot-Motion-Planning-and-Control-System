from __future__ import annotations

from pydantic import BaseModel, Field
from typing import List, Optional


# ---------------- Existing models (keep) ----------------

class JointSpec(BaseModel):
    name: str
    min_deg: float
    max_deg: float
    current_deg: float = 0.0


class RobotState(BaseModel):
    joints_deg: List[float]
    moving: bool
    target_deg: Optional[List[float]] = None
    object: Optional["ObjectBox"] = None
    grasped: bool = False
    bar_height: Optional[float] = None


class SetTargetRequest(BaseModel):
    joints_deg: List[float] = Field(..., min_length=6, max_length=6)
    move_seconds: float = Field(2.0, ge=0.1, le=20.0)


class PickPlaceRequest(BaseModel):
    """Pick/Place uses joint targets (Option 1)."""
    joints_deg: List[float] = Field(..., min_length=6, max_length=6)
    move_seconds: float = Field(2.0, ge=0.1, le=20.0)


class PlaceRequest(BaseModel):
    """Place uses a fixed table target; only timing is configurable."""
    move_seconds: float = Field(2.0, ge=0.1, le=20.0)


class BarHeightRequest(BaseModel):
    height: float = Field(..., gt=0.05, le=0.90, description="Vertical bar obstacle height (meters)")


class CartesianPoseRequest(BaseModel):
    """Cartesian target in world frame (meters)."""
    x: float = Field(..., description="World X (meters)")
    y: float = Field(..., description="World Y (meters)")
    z: float = Field(..., description="World Z (meters)")
    move_seconds: float = Field(2.0, ge=0.1, le=20.0)


class JogRequest(BaseModel):
    joint_index: int = Field(..., ge=0, le=5)
    delta_deg: float = Field(..., ge=-45.0, le=45.0)
    move_seconds: float = Field(0.5, ge=0.1, le=10.0)


# ---------------- Pick & Place object (simple box) ----------------

class ObjectBox(BaseModel):
    id: str = Field("object", description="Object id")
    x: float = Field(..., description="World X (meters)")
    y: float = Field(..., description="World Y (meters)")
    z: float = Field(..., description="World Z (meters)")
    sx: float = Field(0.06, gt=0.0, description="Size X (meters)")
    sy: float = Field(0.06, gt=0.0, description="Size Y (meters)")
    sz: float = Field(0.06, gt=0.0, description="Size Z (meters)")


class SetObjectRequest(BaseModel):
    object: Optional[ObjectBox] = None
