"""MoveIt one-shot operation that moves only to a pulse-region pre-contact pose."""

from __future__ import annotations

import pathlib
import sys
import time

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from rebotarm_moveit_demos.demo_common import MoveItDemoBase

from .approach_geometry import inside_workspace, precontact_tcp_pose


class PulseApproach(MoveItDemoBase):
    """Wait for a stable target, transform it, and move to a standoff pose."""

    def __init__(self) -> None:
        super().__init__("pulse_approach")
        self._planner = self.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        self._latest_target: PointStamped | None = None
        self._target_received = 0.0
        self._xbox_known = False
        self._xbox_armed = True
        self.node.create_subscription(
            PointStamped,
            str(self._param("target_topic")),
            self._target_cb,
            qos_profile_sensor_data,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(Bool, "/rebot_xbox/armed", self._xbox_cb, state_qos)
        self._base_target_publisher = self.node.create_publisher(
            PointStamped, "/rebotarm/pulse/target", 10
        )
        self._precontact_publisher = self.node.create_publisher(
            PointStamped, "/rebotarm/pulse/precontact", 10
        )
        self._marker_publisher = self.node.create_publisher(
            Marker, "/rebotarm/pulse/markers", 10
        )

    def _target_cb(self, message: PointStamped) -> None:
        if not message.header.frame_id:
            return
        self._latest_target = message
        self._target_received = time.monotonic()

    def _xbox_cb(self, message: Bool) -> None:
        self._xbox_known = True
        self._xbox_armed = bool(message.data)

    def run(self) -> bool:
        if not self._calibration_exists():
            return False
        if not self._planner.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("MoveIt /plan_kinematic_path is unavailable")
            return False
        if not self.wait_for_ik_service() or not self.wait_for_execute_server():
            return False
        if not self._wait_for_locked_and_target():
            return False

        current = self.fresh_current_joint_values()
        if current is None:
            return False
        transformed = self._transform_target()
        if transformed is None:
            return False
        base_target, camera_origin = transformed
        target_xyz = (
            base_target.point.x, base_target.point.y, base_target.point.z
        )
        limits = tuple(float(value) for value in self._param("workspace_limits"))
        if not inside_workspace(target_xyz, limits):
            self.node.get_logger().error(
                "Pulse target is outside the configured base-frame workspace: "
                f"{[round(value, 4) for value in target_xyz]}"
            )
            return False

        try:
            tcp_xyz, quaternion, probe_tip = precontact_tcp_pose(
                camera_origin,
                target_xyz,
                float(self._param("standoff_m")),
                float(self._param("probe_tip_offset_m")),
            )
        except ValueError as error:
            self.node.get_logger().error(str(error))
            return False
        if not inside_workspace(probe_tip, limits) or not inside_workspace(tcp_xyz, limits):
            self.node.get_logger().error(
                "Computed pre-contact point or TCP is outside the workspace"
            )
            return False

        self._publish_points(base_target, probe_tip)
        qx, qy, qz, qw = quaternion
        pose = PoseStamped()
        pose.header.frame_id = str(self._param("base_frame"))
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose = Pose(
            position=Point(x=tcp_xyz[0], y=tcp_xyz[1], z=tcp_xyz[2]),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
        self.node.get_logger().info(
            "stable pulse region in base frame: "
            f"x={target_xyz[0]:.4f}, y={target_xyz[1]:.4f}, z={target_xyz[2]:.4f}; "
            f"planning {float(self._param('standoff_m')) * 1000.0:.0f} mm pre-contact pose"
        )
        goal = self.compute_ik_joint_target(
            pose,
            current,
            str(self._param("tcp_link")),
            float(self._param("ik_timeout")),
            True,
            "pulse pre-contact pose",
        )
        if goal is None:
            return False
        trajectory = self._plan(current, goal)
        if trajectory is None:
            return False
        self.preview_trajectory(trajectory, current)
        if not self._xbox_known or self._xbox_armed:
            self.node.get_logger().error(
                "Xbox control is no longer LOCKED; refusing trajectory execution"
            )
            return False
        self.node.get_logger().info(
            "executing pre-contact motion only; no force-controlled human contact is commanded"
        )
        return self.execute_trajectory(
            trajectory, float(self._param("result_timeout"))
        )

    def _calibration_exists(self) -> bool:
        name = str(self._param("calibration_name")).strip()
        path = pathlib.Path.home() / ".ros2/easy_handeye2/calibrations" / f"{name}.calib"
        if path.is_file():
            return True
        self.node.get_logger().error(
            f"hand-eye calibration is not saved: {path}"
        )
        return False

    def _wait_for_locked_and_target(self) -> bool:
        timeout = float(self._param("target_timeout_sec"))
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            fresh_target = (
                self._latest_target is not None
                and time.monotonic() - self._target_received
                <= float(self._param("maximum_target_age_sec"))
            )
            if self._xbox_known and not self._xbox_armed and fresh_target:
                return True
        if not self._xbox_known or self._xbox_armed:
            self.node.get_logger().error("Xbox control must report LOCKED")
        else:
            self.node.get_logger().error(
                "timed out waiting for a stable, fresh pulse-region point"
            )
        return False

    def _transform_target(self) -> tuple[PointStamped, tuple[float, float, float]] | None:
        target = self._latest_target
        if target is None:
            return None
        base_frame = str(self._param("base_frame"))
        try:
            transform = self._tf_buffer.lookup_transform(
                base_frame,
                target.header.frame_id,
                Time(),
                timeout=Duration(seconds=float(self._param("tf_timeout_sec"))),
            )
            transformed = do_transform_point(target, transform)
        except TransformException as error:
            self.node.get_logger().error(
                f"hand-eye TF {base_frame} <- {target.header.frame_id} unavailable: {error}"
            )
            return None
        translation = transform.transform.translation
        return transformed, (translation.x, translation.y, translation.z)

    def _plan(self, start: list[float], goal: list[float]):
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
        motion.start_state = self._joint_state(start)
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
                    for name, value in zip(self.joint_names, goal)
                ]
            )
        ]
        future = self._planner.call_async(request)
        timeout = float(self._param("result_timeout"))
        if not self.wait(future, timeout):
            self.node.get_logger().error("MoveIt pre-contact planning timed out")
            return None
        response = future.result()
        plan = response.motion_plan_response if response is not None else None
        if plan is None or plan.error_code.val != MoveItErrorCodes.SUCCESS:
            code = plan.error_code.val if plan is not None else "empty"
            message = plan.error_code.message if plan is not None else ""
            self.node.get_logger().error(
                f"MoveIt pre-contact planning failed with code {code}: {message}"
            )
            return None
        return plan.trajectory

    def _publish_points(
        self, base_target: PointStamped, probe_tip: tuple[float, float, float]
    ) -> None:
        self._base_target_publisher.publish(base_target)
        precontact = PointStamped()
        precontact.header = base_target.header
        precontact.point = Point(x=probe_tip[0], y=probe_tip[1], z=probe_tip[2])
        self._precontact_publisher.publish(precontact)
        for marker_id, point, color, scale in (
            (0, base_target.point, (1.0, 0.2, 0.05), 0.024),
            (1, precontact.point, (0.0, 0.8, 1.0), 0.018),
        ):
            marker = Marker()
            marker.header = base_target.header
            marker.ns = "pulse_approach"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.95
            self._marker_publisher.publish(marker)


def main() -> None:
    rclpy.init()
    operation = PulseApproach()
    try:
        ok = operation.run()
    except Exception as error:
        operation.node.get_logger().error(str(error))
        ok = False
    finally:
        operation.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
