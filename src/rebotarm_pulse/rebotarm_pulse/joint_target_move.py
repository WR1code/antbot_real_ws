"""Plan and execute a guarded joint-space motion away from singular poses."""

from __future__ import annotations

import sys
import time

from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from rebotarm_moveit_demos.demo_common import MoveItDemoBase

from .joint_target_validation import validate_joint_target


class JointTargetMove(MoveItDemoBase):
    """One-shot collision-checked joint target with Xbox ownership guards."""

    def __init__(self) -> None:
        super().__init__("joint_target_move")
        self.target = validate_joint_target(
            self._param("target_joint_values"),
            self.joint_names,
            self._param("joint_lower_limits"),
            self._param("joint_upper_limits"),
            self._param("singularity_guard_joint_indices"),
            self._param("singularity_guard_minimum_abs_rad"),
        )
        self._planner = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._xbox_known = False
        self._xbox_armed = True
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(
            Bool, "/rebot_xbox/armed", self._xbox_cb, state_qos
        )

    def _xbox_cb(self, message: Bool) -> None:
        self._xbox_known = True
        self._xbox_armed = bool(message.data)

    def _wait_for_xbox_locked(self, timeout_sec: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._xbox_known:
                if not self._xbox_armed:
                    return True
                break
        self.node.get_logger().error(
            "Xbox must report LOCKED before joint-space escape motion"
        )
        return False

    def run(self) -> bool:
        if not self._planner.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("MoveIt /plan_kinematic_path is unavailable")
            return False
        if not self.wait_for_execute_server() or not self._wait_for_xbox_locked():
            return False
        current = self.fresh_current_joint_values()
        if current is None:
            return False

        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.group_name
        motion.pipeline_id = str(self._param("pipeline_id"))
        motion.planner_id = str(self._param("planner_id"))
        motion.allowed_planning_time = float(self._param("planning_time"))
        motion.num_planning_attempts = int(self._param("planning_attempts"))
        motion.max_velocity_scaling_factor = float(self._param("velocity_scaling"))
        motion.max_acceleration_scaling_factor = float(
            self._param("acceleration_scaling")
        )
        motion.start_state = self._joint_state(current)
        tolerance = float(self._param("joint_tolerance"))
        motion.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=name,
                        position=value,
                        tolerance_above=tolerance,
                        tolerance_below=tolerance,
                        weight=1.0,
                    )
                    for name, value in zip(self.joint_names, self.target)
                ]
            )
        ]
        timeout = float(self._param("result_timeout"))
        self.node.get_logger().info(
            "planning guarded joint-space escape from "
            f"{[round(value, 4) for value in current]} to "
            f"{[round(value, 4) for value in self.target]}"
        )
        future = self._planner.call_async(request)
        if not self.wait(future, timeout):
            self.node.get_logger().error("MoveIt joint-space planning timed out")
            return False
        response = future.result()
        plan = response.motion_plan_response if response is not None else None
        if plan is None or plan.error_code.val != MoveItErrorCodes.SUCCESS:
            code = plan.error_code.val if plan is not None else "empty"
            message = plan.error_code.message if plan is not None else ""
            self.node.get_logger().error(
                f"MoveIt joint-space planning failed with code {code}: {message}"
            )
            return False

        self.preview_trajectory(plan.trajectory, current)
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if not self._xbox_known or self._xbox_armed:
            self.node.get_logger().error(
                "Xbox is no longer LOCKED; refusing trajectory execution"
            )
            return False
        self.node.get_logger().info("executing guarded joint-space escape motion")
        if not self.execute_trajectory(plan.trajectory, timeout):
            return False
        self.node.get_logger().info("joint-space escape motion completed")
        return True


def main() -> None:
    rclpy.init()
    operation = None
    ok = False
    try:
        operation = JointTargetMove()
        ok = operation.run()
    except Exception as error:
        if operation is not None:
            operation.node.get_logger().error(str(error))
        else:
            print(f"joint_target_move: {error}", file=sys.stderr)
    finally:
        if operation is not None:
            operation.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
