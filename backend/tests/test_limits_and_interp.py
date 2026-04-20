import pytest
from app.robot import RobotArm, Joint
from app.planner import interpolate_linear

def test_joint_clamp():
    j = Joint("J", -10, 10, 0)
    assert j.clamp(100) == 10
    assert j.clamp(-100) == -10
    assert j.clamp(5) == 5

def test_robot_target_clamped():
    robot = RobotArm([
        Joint("J1", -1, 1, 0),
        Joint("J2", -2, 2, 0),
        Joint("J3", -3, 3, 0),
        Joint("J4", -4, 4, 0),
        Joint("J5", -5, 5, 0),
        Joint("J6", -6, 6, 0),
    ])
    clamped = robot.set_target([10, -10, 2, -2, 0, 999])
    assert clamped == [1, -2, 2, -2, 0, 6]

def test_interpolate_linear_endpoints():
    current = [0, 0, 0, 0, 0, 0]
    target  = [6, 5, 4, 3, 2, 1]
    path = interpolate_linear(current, target, steps=5)
    assert path[0] == current
    assert path[-1] == target
    assert len(path) == 5

def test_interpolate_linear_invalid():
    with pytest.raises(ValueError):
        interpolate_linear([0]*6, [0]*6, steps=1)
