from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional
import threading

@dataclass
class Joint:
    name: str
    min_deg: float
    max_deg: float
    current_deg: float = 0.0

    def clamp(self, deg: float) -> float:
        return max(self.min_deg, min(self.max_deg, deg))

class RobotArm:
    """
    Thread-safe 6-axis robot joint state container.
    """
    def __init__(self, joints: List[Joint]):
        if len(joints) != 6:
            raise ValueError("RobotArm requires exactly 6 joints")
        self._joints = joints
        self._lock = threading.RLock()
        self._moving = False
        self._target: Optional[List[float]] = None

    def get_joint_values(self) -> List[float]:
        with self._lock:
            return [j.current_deg for j in self._joints]

    def get_limits(self) -> List[tuple[float, float]]:
        with self._lock:
            return [(j.min_deg, j.max_deg) for j in self._joints]

    def set_joint_values(self, values: List[float]) -> None:
        if len(values) != 6:
            raise ValueError("Expected 6 joint values")
        with self._lock:
            for j, v in zip(self._joints, values):
                j.current_deg = j.clamp(v)

    def set_target(self, target: List[float]) -> List[float]:
        """
        Clamp target into joint limits and store it.
        Returns the clamped target.
        """
        if len(target) != 6:
            raise ValueError("Expected 6 joint values")
        with self._lock:
            clamped = [j.clamp(v) for j, v in zip(self._joints, target)]
            self._target = clamped
            return clamped

    def get_target(self) -> Optional[List[float]]:
        with self._lock:
            return None if self._target is None else list(self._target)

    def set_moving(self, moving: bool) -> None:
        with self._lock:
            self._moving = moving

    def is_moving(self) -> bool:
        with self._lock:
            return self._moving
