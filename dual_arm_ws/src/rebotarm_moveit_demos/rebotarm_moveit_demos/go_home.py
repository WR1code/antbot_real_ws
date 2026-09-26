from __future__ import annotations

import sys

from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import rclpy

from rebotarm_moveit_demos.demo_common import MoveItDemoBase


class GoHome(MoveItDemoBase):
    """Plan and execute a collision-checked motion to the model's home pose."""

    def __init__(self) -> None:
        super().__init__("go_home")
        self.home_joint_values = [
            float(value) for value in self._param("home_joint_values")
        ]
        self._planner = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")

    def run(self) -> bool:
        if len(self.home_joint_values) != len(self.joint_names):
            self.node.get_logger().error(
                "home_joint_values must contain one value for every arm joint"
            )
            return False
        if not self._planner.wait_for_service(timeout_sec=30.0):
            self.node.get_logger().error(
                "MoveIt service /plan_kinematic_path is not available"
            )
            return False
        if not self.wait_for_execute_server():
            return False

        current = self.fresh_current_joint_values()
        if current is None:
            return False

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = self.group_name
        motion_request.pipeline_id = str(self._param("pipeline_id"))
        motion_request.planner_id = str(self._param("planner_id"))
        motion_request.allowed_planning_time = float(self._param("planning_time"))
        motion_request.num_planning_attempts = 5
        motion_request.max_velocity_scaling_factor = float(
            self._param("velocity_scaling")
        )
        motion_request.max_acceleration_scaling_factor = float(
            self._param("acceleration_scaling")
        )
        motion_request.start_state = self._joint_state(current)
        tolerance = float(self._param("joint_tolerance"))
        motion_request.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=name,
                        position=position,
                        tolerance_above=tolerance,
                        tolerance_below=tolerance,
                        weight=1.0,
                    )
                    for name, position in zip(
                        self.joint_names, self.home_joint_values
                    )
                ]
            )
        ]

        result_timeout = float(self._param("result_timeout"))
        self.node.get_logger().info(
            f"planning home motion from {[round(value, 4) for value in current]} "
            f"to {[round(value, 4) for value in self.home_joint_values]}"
        )
        future = self._planner.call_async(request)
        if not self.wait(future, result_timeout):
            self.node.get_logger().error(
                f"MoveIt planner did not return within {result_timeout:.1f}s"
            )
            return False

        response = future.result()
        plan_response = response.motion_plan_response if response is not None else None
        if (
            plan_response is None
            or plan_response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            code = plan_response.error_code.val if plan_response is not None else "empty"
            message = (
                plan_response.error_code.message if plan_response is not None else ""
            )
            self.node.get_logger().error(
                f"MoveIt home planning failed with code {code}: {message}"
            )
            return False

        self.node.get_logger().info("home motion planned; publishing RViz preview")
        self.preview_trajectory(plan_response.trajectory, current)
        self.node.get_logger().info("executing home motion")
        if not self.execute_trajectory(plan_response.trajectory, result_timeout):
            return False
        self.node.get_logger().info("home motion completed")
        return True


def main() -> None:
    rclpy.init()
    operation = GoHome()
    try:
        ok = operation.run()
    except Exception as exc:
        operation.node.get_logger().error(str(exc))
        ok = False
    finally:
        operation.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
