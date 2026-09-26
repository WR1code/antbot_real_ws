"""ROS interface for fail-safe reBotArm/Piper-H controller handoff."""

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .arm_ownership import ArmOwnership, VALID_ROBOTS


class ActiveArmManager(Node):
    def __init__(self) -> None:
        super().__init__("active_arm_manager")
        self.declare_parameter("initial_robot", "rebotarm")
        self.declare_parameter("offline_preview", False)
        initial = str(self.get_parameter("initial_robot").value).strip().lower()
        if initial not in VALID_ROBOTS:
            raise ValueError("initial_robot must be rebotarm or piperh")
        self._offline_preview = bool(self.get_parameter("offline_preview").value)
        self._ownership = ArmOwnership(selected=initial)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._selected_pub = self.create_publisher(String, "/dual_arm/selected", qos)
        self._status_pub = self.create_publisher(String, "/dual_arm/status", qos)
        self._cancel_clients = [
            self.create_client(CancelGoal, name)
            for name in (
                "/rebotarm/move_action/_action/cancel_goal",
                "/rebotarm/execute_trajectory/_action/cancel_goal",
                "/rebotarm/follow_joint_trajectory/_action/cancel_goal",
                "/piperh/move_action/_action/cancel_goal",
                "/piperh/execute_trajectory/_action/cancel_goal",
                "/piperh/arm_controller/follow_joint_trajectory/_action/cancel_goal",
            )
        ]
        self.create_subscription(
            String, "/dual_arm/select_request", self._select, 10
        )
        for robot in VALID_ROBOTS:
            self.create_subscription(
                Bool,
                f"/{robot}/xbox/armed",
                lambda msg, name=robot: self._armed(name, msg),
                qos,
            )
        self._publish_selected()
        if self._offline_preview:
            self._publish_status(
                f"离线模型预览：当前显示 {initial}；不会连接或控制真实机械臂"
            )
        else:
            self._publish_status(f"当前控制：{initial}；按 A 键后才会解锁")

    def _publish_selected(self) -> None:
        self._selected_pub.publish(String(data=self._ownership.selected))

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))

    def _select(self, msg: String) -> None:
        target = str(msg.data).strip().lower()
        if self._offline_preview:
            if target not in VALID_ROBOTS:
                self._publish_status(f"切换拒绝：未知机械臂 {target}")
                return
            self._ownership.selected = target
            self._ownership.pending = ""
            self._publish_selected()
            self._publish_status(
                f"离线模型预览：已切换到 {target}；真实机械臂输出保持禁用"
            )
            return
        try:
            previous = self._ownership.selected
            selected = self._ownership.request(target)
        except ValueError as error:
            self.get_logger().error(str(error))
            self._publish_status(f"切换拒绝：{error}")
            return
        if selected == previous and not self._ownership.pending:
            self._publish_status(f"{target} 已经是当前控制机械臂")
            return
        # "none" is latched first. Both Xbox nodes receive it and synchronously
        # enter LOCKED before either robot can become the new owner.
        self._publish_selected()
        # A plan started from the previous RViz context must not continue after
        # the operator changes ownership. A zero-valued CancelGoal means all.
        for client in self._cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
        self._publish_status(f"正在锁定两台机械臂，准备切换到 {target}…")

    def _armed(self, robot: str, msg: Bool) -> None:
        before = self._ownership.selected
        selected = self._ownership.report_armed(robot, msg.data)
        if selected != before:
            self._publish_selected()
            self._publish_status(
                f"已切换到 {selected}；控制器保持锁定，请确认摇杆回中后按 A"
            )
            self.get_logger().warning(
                f"controller ownership transferred to {selected}; still LOCKED"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActiveArmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
