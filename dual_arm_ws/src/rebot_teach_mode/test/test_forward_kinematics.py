import math

import pytest

from rebot_teach_mode.forward_kinematics import SerialChainFK


URDF = """
<robot name="test">
  <link name="base"/><link name="arm"/><link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="tip_fixed" type="fixed">
    <parent link="arm"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_serial_chain_fk_applies_origin_then_joint_rotation():
    chain = SerialChainFK(URDF, "base", "tip")
    assert chain.position({"joint1": 0.0}) == pytest.approx((2.0, 0.0, 0.0))
    assert chain.position({"joint1": math.pi / 2}) == pytest.approx((1.0, 1.0, 0.0))


def test_serial_chain_fk_transforms_a_tool_point_from_tip_frame():
    chain = SerialChainFK(URDF, "base", "tip")
    assert chain.position(
        {"joint1": 0.0}, local_point=(-0.25, 0.0, 0.0)
    ) == pytest.approx((1.75, 0.0, 0.0))
    assert chain.position(
        {"joint1": math.pi / 2}, local_point=(-0.25, 0.0, 0.0)
    ) == pytest.approx((1.0, 0.75, 0.0))


def test_serial_chain_fk_returns_tip_orientation():
    chain = SerialChainFK(URDF, "base", "tip")
    position, orientation = chain.pose({"joint1": math.pi / 2})
    assert position == pytest.approx((1.0, 1.0, 0.0))
    assert orientation == pytest.approx(
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    )


def test_serial_chain_fk_rejects_invalid_tool_point():
    chain = SerialChainFK(URDF, "base", "tip")
    with pytest.raises(ValueError, match="three"):
        chain.position({}, local_point=(0.0, 0.0))


def test_serial_chain_fk_rejects_unconnected_tip():
    with pytest.raises(ValueError, match="no URDF chain"):
        SerialChainFK(URDF, "base", "missing")
