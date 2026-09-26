import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarmcontroller.servo_trajectory_input import validated_servo_target


def test_validates_complete_atomic_servo_target():
    message = JointTrajectory()
    message.joint_names = ["joint1", "joint2"]
    point = JointTrajectoryPoint()
    point.positions = [0.1, -0.2]
    message.points = [point]
    assert validated_servo_target(message, ["joint1", "joint2"]) == [0.1, -0.2]


def test_rejects_wrong_names_and_nonfinite_targets():
    message = JointTrajectory()
    message.joint_names = ["joint2", "joint1"]
    point = JointTrajectoryPoint()
    point.positions = [0.1, float("nan")]
    message.points = [point]
    with pytest.raises(ValueError):
        validated_servo_target(message, ["joint1", "joint2"])
