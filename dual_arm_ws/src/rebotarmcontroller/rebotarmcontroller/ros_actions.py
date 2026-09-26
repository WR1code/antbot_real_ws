from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose

from .conversions import pose_to_xyz_rpy


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def set_duration_seconds(duration, seconds: float) -> None:
    """Populate a ROS duration without losing a nanosecond carry."""
    safe_seconds = max(0.0, float(seconds))
    whole_seconds = int(safe_seconds)
    nanoseconds = int(round((safe_seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    duration.sec = whole_seconds
    duration.nanosec = nanoseconds


def resolved_position_tolerances(specifications, joint_names, configured_max):
    """Resolve action tolerances without allowing a goal to weaken safety limits."""
    result = np.full(len(joint_names), float(configured_max), dtype=np.float64)
    indices = {name: index for index, name in enumerate(joint_names)}
    for specification in specifications:
        index = indices.get(specification.name)
        if index is None or specification.position <= 0.0:
            continue
        result[index] = min(result[index], float(specification.position))
    return result


def path_tolerance_violation(joint_names, target, positions, tolerances):
    """Describe the worst joint error when any path tolerance is exceeded."""
    errors = np.abs(np.asarray(target) - np.asarray(positions))
    limits = np.asarray(tolerances)
    violating = np.flatnonzero(errors > limits)
    if violating.size == 0:
        return None
    worst = int(violating[np.argmax(errors[violating])])
    return (
        "path tolerance violated: "
        f"joint={joint_names[worst]}, "
        f"error={float(errors[worst]):.6f} rad, "
        f"limit={float(limits[worst]):.6f} rad, "
        f"desired={float(target[worst]):.6f} rad, "
        f"actual={float(positions[worst]):.6f} rad"
    )


def tracking_speed_scale(
    target,
    positions,
    tolerances,
    slowdown_ratio=0.5,
    stop_ratio=0.75,
):
    """Slow the trajectory clock before feedback reaches the hard limit."""
    errors = np.abs(np.asarray(target) - np.asarray(positions))
    limits = np.maximum(np.asarray(tolerances, dtype=np.float64), 1e-9)
    worst_ratio = float(np.max(errors / limits)) if errors.size else 0.0
    if worst_ratio <= slowdown_ratio:
        return 1.0
    if worst_ratio >= stop_ratio:
        return 0.0
    return (stop_ratio - worst_ratio) / (stop_ratio - slowdown_ratio)


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._path_tolerance = max(
            1e-4, float(node.get_parameter("trajectory_path_tolerance").value)
        )
        self._goal_tolerance = max(
            1e-4, float(node.get_parameter("trajectory_goal_tolerance").value)
        )
        self._goal_time_tolerance = max(
            0.1,
            float(node.get_parameter("trajectory_goal_time_tolerance").value),
        )
        self._stopped_velocity_tolerance = max(
            1e-4,
            float(
                node.get_parameter("trajectory_stopped_velocity_tolerance").value
            ),
        )
        self._tracking_slowdown_ratio = min(
            0.9,
            max(
                0.05,
                float(
                    node.get_parameter(
                        "trajectory_tracking_slowdown_ratio"
                    ).value
                ),
            ),
        )
        self._tracking_stop_ratio = min(
            0.95,
            max(
                self._tracking_slowdown_ratio + 0.05,
                float(
                    node.get_parameter(
                        "trajectory_tracking_stop_ratio"
                    ).value
                ),
            ),
        )
        self._execution_timeout_scaling = max(
            1.0,
            float(
                node.get_parameter(
                    "trajectory_execution_timeout_scaling"
                ).value
            ),
        )
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )

    def arm_goal_callback(self, _goal_request):
        return self._gate_goal(
            (
                "TRAJ_RUNNING",
                "LOWLEVEL_STREAMING",
                "GRAVITY_COMP",
                "SAFE_HOMING",
            ),
            "arm motion",
        )

    def gripper_goal_callback(self, _goal_request):
        return self._gate_goal(("GRAVITY_COMP", "SAFE_HOMING"), "gripper")

    def _gate_goal(self, blocked, label):
        state = self._hardware.state_machine
        if state in blocked:
            self._node.get_logger().warn(f"rejecting {label} goal in state {state}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _fail_move_to_pose(self, goal_handle, result, message, *, canceled=False):
        if self._hardware.state_machine != "SAFE_HOMING":
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result.success = False
        result.message = message
        result.final_pose = self._hardware.current_pose()
        return result

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.move_to_pose_traj(
                x, y, z, roll, pitch, yaw, float(goal.duration)
            )
        except Exception as exc:
            self._hardware.hold_current_position()
            return self._fail_move_to_pose(goal_handle, result, str(exc))

        if not ok:
            return self._fail_move_to_pose(
                goal_handle, result, "trajectory planning failed"
            )
        self._node.publish_arm_status()

        deadline = time.monotonic() + max(float(goal.duration), 0.0) + 2.0
        while self._hardware.motion_active():
            if self._hardware.state_machine == "SAFE_HOMING":
                self._hardware.stop_motion()
                break
            if goal_handle.is_cancel_requested:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose canceled", canceled=True
                )
            if time.monotonic() > deadline:
                self._hardware.stop_motion()
                self._hardware.hold_current_position()
                return self._fail_move_to_pose(
                    goal_handle, result, "move_to_pose timeout"
                )
            time.sleep(0.02)

        if self._hardware.state_machine == "SAFE_HOMING":
            return self._fail_move_to_pose(
                goal_handle, result, "move_to_pose preempted by safe_home"
            )

        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.success = True
        result.message = (
            "move_to_traj accepted "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)

        if not joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        if joint_names != self._hardware.joint_names:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                f"trajectory joint_names must be {self._hardware.joint_names}"
            )
            return result

        targets = [
            np.array(point.positions, dtype=np.float64)
            for point in trajectory.points
        ]
        if any(len(target) != len(self._hardware.joint_names) for target in targets):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "point positions length must match joint_names"
            return result
        if any(not np.all(np.isfinite(target)) for target in targets):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory positions must be finite"
            return result

        point_times = [
            duration_seconds(point.time_from_start) for point in trajectory.points
        ]
        if any(time_value < 0.0 for time_value in point_times) or any(
            current <= previous
            for previous, current in zip(point_times, point_times[1:])
        ):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                "trajectory point times must be non-negative and strictly increasing"
            )
            return result

        path_tolerances = resolved_position_tolerances(
            goal.path_tolerance,
            joint_names,
            self._path_tolerance,
        )
        goal_tolerances = resolved_position_tolerances(
            goal.goal_tolerance,
            joint_names,
            self._goal_tolerance,
        )
        requested_goal_time = duration_seconds(goal.goal_time_tolerance)
        goal_time_tolerance = (
            min(requested_goal_time, self._goal_time_tolerance)
            if requested_goal_time > 0.0
            else self._goal_time_tolerance
        )

        try:
            self._hardware.begin_trajectory_stream()
            self._node.publish_arm_status()
            start = time.monotonic()
            execution_deadline = (
                start
                + max(point_times[-1], 0.1) * self._execution_timeout_scaling
                + goal_time_tolerance
            )
            slowdown_logged = False
            if point_times[0] > 0.0:
                targets.insert(0, self._hardware.get_joint_positions().copy())
                point_times.insert(0, 0.0)

            for index in range(1, len(targets)):
                q0 = targets[index - 1]
                q1 = targets[index]
                t0 = point_times[index - 1]
                t1 = max(point_times[index], t0)
                desired_velocities = np.zeros_like(q1)
                trajectory_time = t0
                last_tick = time.monotonic()
                speed_scale = 1.0

                while True:
                    if not self._hardware.control_loop_active:
                        details = "; ".join(self._hardware.error_codes)
                        raise RuntimeError(
                            "hardware control loop stopped"
                            + (f": {details}" if details else "")
                        )
                    if self._hardware.state_machine == "SAFE_HOMING":
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "follow_joint_trajectory preempted by safe_home"
                        )
                        return result
                    wall_now = time.monotonic()
                    wall_step = max(0.0, wall_now - last_tick)
                    last_tick = wall_now
                    trajectory_time = min(
                        t1,
                        trajectory_time + wall_step * speed_scale,
                    )
                    ratio = (
                        1.0
                        if t1 <= t0
                        else max(
                            0.0,
                            min(1.0, (trajectory_time - t0) / (t1 - t0)),
                        )
                    )
                    target = q0 + (q1 - q0) * ratio
                    self._hardware.set_joint_position_target(target)

                    positions = self._hardware.get_joint_positions()
                    velocities = self._hardware.get_joint_velocities()
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.header.stamp = self._node.get_clock().now().to_msg()
                    feedback.joint_names = self._hardware.joint_names
                    feedback.desired.positions = [float(v) for v in target]
                    feedback.desired.velocities = [float(v) for v in desired_velocities]
                    set_duration_seconds(
                        feedback.desired.time_from_start, trajectory_time
                    )
                    feedback.actual.positions = [float(v) for v in positions]
                    feedback.actual.velocities = [float(v) for v in velocities]
                    set_duration_seconds(
                        feedback.actual.time_from_start, trajectory_time
                    )
                    feedback.error.positions = [float(v) for v in target - positions]
                    feedback.error.velocities = [
                        float(v) for v in desired_velocities - velocities
                    ]
                    set_duration_seconds(
                        feedback.error.time_from_start, trajectory_time
                    )
                    goal_handle.publish_feedback(feedback)

                    violation = path_tolerance_violation(
                        joint_names,
                        target,
                        positions,
                        path_tolerances,
                    )
                    if violation is not None:
                        self._hardware.hold_current_position()
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = violation
                        return result

                    speed_scale = tracking_speed_scale(
                        target,
                        positions,
                        path_tolerances,
                        self._tracking_slowdown_ratio,
                        self._tracking_stop_ratio,
                    )
                    if speed_scale < 0.999 and not slowdown_logged:
                        normalized = np.abs(target - positions) / path_tolerances
                        worst = int(np.argmax(normalized))
                        self._node.get_logger().info(
                            "slowing trajectory to follow hardware: "
                            f"joint={joint_names[worst]}, "
                            f"error={float(abs(target[worst] - positions[worst])):.6f} rad, "
                            f"clock_scale={speed_scale:.3f}"
                        )
                        slowdown_logged = True

                    if goal_handle.is_cancel_requested:
                        self._hardware.hold_current_position()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "follow_joint_trajectory canceled"
                        return result

                    if wall_now >= execution_deadline:
                        self._hardware.hold_current_position()
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "trajectory tracking timed out before the planned "
                            f"endpoint: trajectory_time={trajectory_time:.3f}s, "
                            f"planned_time={point_times[-1]:.3f}s"
                        )
                        return result

                    if trajectory_time >= t1:
                        break
                    time.sleep(0.02)

            # Reaching the final trajectory timestamp is not the same as the
            # motors reaching the final target.  Keep commanding the endpoint
            # until both position error and measured velocity have settled.
            final_target = targets[-1]
            settle_deadline = time.monotonic() + goal_time_tolerance
            while True:
                if not self._hardware.control_loop_active:
                    details = "; ".join(self._hardware.error_codes)
                    raise RuntimeError(
                        "hardware control loop stopped"
                        + (f": {details}" if details else "")
                    )
                self._hardware.set_joint_position_target(final_target)
                positions = self._hardware.get_joint_positions()
                velocities = self._hardware.get_joint_velocities()
                position_error = np.abs(final_target - positions)

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self._node.get_clock().now().to_msg()
                feedback.joint_names = self._hardware.joint_names
                feedback.desired.positions = [float(v) for v in final_target]
                feedback.desired.velocities = [0.0] * len(final_target)
                set_duration_seconds(
                    feedback.desired.time_from_start, point_times[-1]
                )
                feedback.actual.positions = [float(v) for v in positions]
                feedback.actual.velocities = [float(v) for v in velocities]
                set_duration_seconds(
                    feedback.actual.time_from_start, point_times[-1]
                )
                feedback.error.positions = [
                    float(v) for v in final_target - positions
                ]
                feedback.error.velocities = [float(-v) for v in velocities]
                set_duration_seconds(
                    feedback.error.time_from_start, point_times[-1]
                )
                goal_handle.publish_feedback(feedback)

                if goal_handle.is_cancel_requested:
                    self._hardware.hold_current_position()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "follow_joint_trajectory canceled"
                    return result
                if np.all(position_error <= goal_tolerances) and np.all(
                    np.abs(velocities) <= self._stopped_velocity_tolerance
                ):
                    break
                if time.monotonic() >= settle_deadline:
                    self._hardware.hold_current_position()
                    goal_handle.abort()
                    result.error_code = (
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    )
                    result.error_string = (
                        "goal did not settle: "
                        f"max_error={float(np.max(position_error)):.6f} rad, "
                        f"max_velocity={float(np.max(np.abs(velocities))):.6f} rad/s"
                    )
                    return result
                time.sleep(0.02)

        except Exception as exc:
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"execution failed: {exc}"
            return result
        finally:
            if self._hardware.state_machine != "SAFE_HOMING":
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.error_string = (
            "joint target reached "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        return result

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.get_gripper_state()[0]
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                pos, _, effort, _ = self._hardware.get_gripper_state()
                result.position = pos
                result.effort = effort
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos, _, effort, _ = self._hardware.get_gripper_state()
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        pos, _, effort, _ = self._hardware.get_gripper_state()
        result.position = pos
        result.effort = effort
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result
