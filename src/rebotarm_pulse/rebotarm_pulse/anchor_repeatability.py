"""Real Piper-H feedback measurement using the existing guarded MoveIt mover."""
from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path
import statistics
import sys
import time
from zoneinfo import ZoneInfo

from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from .joint_target_move import JointTargetMove
from .joint_target_validation import validate_joint_target

ANCHOR = [-0.15216, 1.50210, -1.55853, -0.26415, 1.00740, -1.93523]
NAMES = [f"joint{i}" for i in range(1, 7)]
DEFAULTS = {
    "group_name": "arm", "joint_names": NAMES,
    "target_joint_values": ANCHOR,
    "joint_lower_limits": [-2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14],
    "joint_upper_limits": [2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14],
    "singularity_guard_joint_indices": [2, 4],
    "singularity_guard_minimum_abs_rad": [0.15, 0.15],
    "pipeline_id": "ompl", "planner_id": "RRTConnect",
    "planning_time": 5.0, "planning_attempts": 5,
    "velocity_scaling": 0.05, "acceleration_scaling": 0.05,
    "joint_tolerance": 0.0001, "result_timeout": 60.0,
    "iterations": 20, "settle_sec": 2.0, "feedback_timeout_sec": 0.75,
    "settle_timeout_sec": 15.0, "stationary_range_deg": 0.02,
    "departure_offset_rad": [math.radians(8.0), 0.0, 0.0, 0.0, 0.0, 0.0],
    "moveit_feedback_topic": "/piperh/joint_states",
    "output_dir": ".",
}


def joint_positions(message):
    """Accept only complete finite named feedback; never infer missing joints."""
    if len(message.name) != len(message.position):
        return None
    names = [name.split("/")[-1] for name in message.name]
    if any(names.count(name) != 1 for name in NAMES):
        return None
    values = tuple(float(message.position[names.index(name)]) for name in NAMES)
    return values if all(math.isfinite(value) for value in values) else None


def error_degrees(positions):
    return [math.degrees(value - target) for value, target in zip(positions, ANCHOR)]


def summarize(rows):
    if not rows:
        raise ValueError("No completed measurements")
    result = []
    for values in zip(*rows):
        result.append({
            "mean": statistics.mean(values), "std": statistics.pstdev(values),
            "max_absolute": max(abs(value) for value in values),
            "min": min(values), "max": max(values),
        })
    worst = max(range(6), key=lambda index: result[index]["max_absolute"])
    return result, worst, all(item["max_absolute"] < 0.1 for item in result)


class AnchorRepeatability(JointTargetMove):
    """Reuse JointTargetMove.run for every collision-checked departure/return."""

    def _param(self, name):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, DEFAULTS[name])
        return super()._param(name)

    def __init__(self):
        super().__init__()
        for name in DEFAULTS:
            self._param(name)
        if self.node.get_parameter("use_sim_time").value:
            raise ValueError("use_sim_time must be false: real feedback is required")
        if self.joint_names != NAMES:
            raise ValueError("Piper-H joint_names must be joint1 through joint6")
        self._raw = None
        self._raw_time = 0.0
        self._raw_stamp = 0
        self._sequence = 0
        self._moveit_time = 0.0
        self._moveit_stamp = 0
        self._active_goal = None
        self._guard_enabled = False
        self.csv_path = None
        self.node.create_subscription(
            JointState, "/piperh/driver_joint_states", self._raw_cb, qos_profile_sensor_data
        )
        self.node.create_subscription(
            JointState, str(self._param("moveit_feedback_topic")),
            self._moveit_cb, qos_profile_sensor_data,
        )
        for name in ("settle_sec", "feedback_timeout_sec", "settle_timeout_sec",
                     "stationary_range_deg", "result_timeout", "joint_tolerance"):
            value = float(self._param(name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if int(self._param("iterations")) < 1:
            raise ValueError("iterations must be positive")
        if float(self._param("settle_timeout_sec")) <= float(self._param("settle_sec")):
            raise ValueError("settle_timeout_sec must exceed settle_sec")
        for name in ("velocity_scaling", "acceleration_scaling"):
            if not 0 < float(self._param(name)) <= 1:
                raise ValueError(f"{name} must be in (0, 1]")
        offset = self._param("departure_offset_rad")
        if len(offset) != 6 or not all(math.isfinite(x) for x in offset) or not any(offset):
            raise ValueError("departure_offset_rad requires six finite values, at least one nonzero")
        self._departures = [self._validate([a + sign * b for a, b in zip(ANCHOR, offset)])
                            for sign in (1, -1)]
        self._validate(ANCHOR)

    def _validate(self, target):
        return validate_joint_target(
            target, NAMES, self._param("joint_lower_limits"), self._param("joint_upper_limits"),
            self._param("singularity_guard_joint_indices"),
            self._param("singularity_guard_minimum_abs_rad"),
        )

    def _raw_cb(self, message):
        values = joint_positions(message)
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        age = (now - stamp) / 1e9
        if (values is None or stamp <= self._raw_stamp or stamp <= 0
                or age < -0.1 or age > float(self._param("feedback_timeout_sec"))):
            return
        self._raw, self._raw_stamp = values, stamp
        self._raw_time = time.monotonic()
        self._sequence += 1

    def _moveit_cb(self, message):
        # MoveIt independently validates its monitored state before execution.
        # This stream is a prerequisite only; measurements still use raw feedback.
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if (joint_positions(message) is None or stamp <= self._moveit_stamp or stamp <= 0
                or age < -0.1 or age > float(self._param("feedback_timeout_sec"))):
            return
        self._moveit_time = time.monotonic()
        self._moveit_stamp = stamp

    def _guard(self):
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown")
        if not self._xbox_known or self._xbox_armed:
            raise RuntimeError("Xbox must remain LOCKED")
        if self._raw is None or time.monotonic() - self._raw_time > float(self._param("feedback_timeout_sec")):
            raise RuntimeError("Real /piperh/driver_joint_states feedback is missing or stale")
        if (self._moveit_stamp == 0 or time.monotonic() - self._moveit_time
                > float(self._param("feedback_timeout_sec"))):
            raise RuntimeError(
                f"MOVEIT_STATE_STALE: {self._param('moveit_feedback_topic')} has no fresh complete "
                "joint state; raw driver feedback alone cannot satisfy MoveIt execution validation. "
                "Check the existing Piper-H hardware adapter/state publisher before retrying."
            )

    def fresh_current_joint_values(self, timeout_sec=2.0, max_age_sec=1.0):
        # Planning starts from the same raw hardware feedback used for measurement.
        self._guard()
        return list(self._raw)

    def wait(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if self._guard_enabled:
                self._guard()
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self._guard_enabled:
            self._guard()
        return future.done()

    def cancel_motion(self):
        if self._active_goal is not None:
            future = self._active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if not future.done() or not future.result() or not future.result().goals_canceling:
                self.node.get_logger().error("Trajectory cancellation was not acknowledged")
            self._active_goal = None

    def execute_trajectory(self, trajectory, timeout_sec):
        # Same existing ExecuteTrajectory action, with cancellation on guard failure/timeout.
        self._guard()
        future = self._execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        abandoned = [False]

        def accepted(done):
            handle = done.result()
            if handle is not None and handle.accepted:
                if abandoned[0]:
                    handle.cancel_goal_async()
                else:
                    self._active_goal = handle

        future.add_done_callback(accepted)
        try:
            if not self.wait(future, 5.0):
                raise RuntimeError("Timed out sending ExecuteTrajectory goal")
            handle = future.result()
            if handle is None or not handle.accepted:
                raise RuntimeError("ExecuteTrajectory goal rejected")
            self._active_goal = handle
            result_future = handle.get_result_async()
            if not self.wait(result_future, timeout_sec):
                raise RuntimeError("ExecuteTrajectory result timeout")
            result = result_future.result()
            if result is not None and result.status in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED,
            ):
                # A terminal goal has already stopped; canceling it reports a false alarm.
                self._active_goal = None
            if (result is None or result.status != GoalStatus.STATUS_SUCCEEDED
                    or result.result.error_code.val != MoveItErrorCodes.SUCCESS):
                if result is not None:
                    code = result.result.error_code
                    label = "CONTROL_FAILED" if code.val == MoveItErrorCodes.CONTROL_FAILED else str(code.val)
                    raise RuntimeError(
                        f"ExecuteTrajectory failed: status={result.status}, {label} ({code.val}), "
                        f"message={code.message!r}, source={code.source!r}. "
                        "See the /piperh/move_group log for controller/state-validation details."
                    )
                raise RuntimeError("ExecuteTrajectory returned no result")
            self._active_goal = None
            return True
        except BaseException:
            abandoned[0] = True
            if self._active_goal is not None:
                self.cancel_motion()
            raise

    def preview_trajectory(self, trajectory, start_values):
        joint = trajectory.joint_trajectory
        if not joint.points or len(joint.joint_names) != 6 or set(joint.joint_names) != set(NAMES):
            raise RuntimeError("MoveIt returned an incomplete arm trajectory")
        endpoint = joint_positions(JointState(
            name=list(joint.joint_names), position=list(joint.points[-1].positions)))
        if endpoint is None or max(abs(a - b) for a, b in zip(endpoint, self.target)) > float(self._param("joint_tolerance")) + 1e-9:
            raise RuntimeError("MoveIt endpoint is outside requested joint tolerance")
        self._guard()
        super().preview_trajectory(trajectory, start_values)
        self._guard()

    def _move(self, target):
        self._guard()
        self.target = self._validate(target)
        if not super().run():
            raise RuntimeError("Existing guarded JointTargetMove failed")

    def _settled_feedback(self):
        started = time.monotonic()
        sequence = self._sequence
        samples = []
        while time.monotonic() - started < float(self._param("settle_timeout_sec")):
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self._guard()
            if self._sequence == sequence:
                continue
            sequence = self._sequence
            samples.append((self._raw_time, self._raw))
            # Require a full stationary window; shifting windows allow late settling.
            cutoff = self._raw_time - float(self._param("settle_sec"))
            while len(samples) > 1 and samples[1][0] <= cutoff:
                samples.pop(0)
            if samples[-1][0] - samples[0][0] < float(self._param("settle_sec")):
                continue
            ranges = [math.degrees(max(values) - min(values))
                      for values in zip(*(values for _, values in samples))]
            if max(ranges) <= float(self._param("stationary_range_deg")):
                return self._raw, self._raw_stamp
        raise RuntimeError("Robot did not settle with continuous real feedback before timeout")

    def run(self):
        deadline = time.monotonic() + 12.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (self._raw is not None and self._xbox_known and not self._xbox_armed
                    and self._moveit_stamp > 0
                    and time.monotonic() - self._moveit_time <= float(self._param("feedback_timeout_sec"))):
                break
        self._guard()
        self._guard_enabled = True
        output = Path(str(self._param("output_dir"))).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=True)
        self.csv_path = output / datetime.now(ZoneInfo("Asia/Shanghai")).strftime(
            "anchor_repeatability_%Y%m%d_%H%M.csv")
        rows = []
        # Exclusive creation prevents overwriting another run in the same minute.
        with self.csv_path.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["iteration", *(f"j{i}_error_deg" for i in range(1, 7)), "timestamp"])
            stream.flush()
            self.node.get_logger().info(f"CSV: {self.csv_path}")
            self._move(ANCHOR)
            self._settled_feedback()
            for iteration in range(1, int(self._param("iterations")) + 1):
                departure = self._departures[(iteration - 1) % 2]
                self._move(departure)
                departed, _ = self._settled_feedback()
                if max(abs(a - b) for a, b in zip(departed, ANCHOR)) < math.radians(0.1):
                    raise RuntimeError("Real feedback did not confirm departure from Anchor")
                self._move(ANCHOR)
                positions, stamp = self._settled_feedback()
                errors = error_degrees(positions)
                timestamp = datetime.fromtimestamp(stamp / 1e9, ZoneInfo("Asia/Shanghai")).isoformat()
                writer.writerow([iteration, *errors, timestamp])
                stream.flush()
                rows.append(errors)
                self.node.get_logger().info(f"iteration {iteration}: " + ", ".join(
                    f"J{i}={value:+.6f} deg" for i, value in enumerate(errors, 1)))
        stats, worst, passed = summarize(rows)
        for index, item in enumerate(stats, 1):
            self.node.get_logger().info(
                f"J{index}:\nmean: {item['mean']:.6f} deg\nstd: {item['std']:.6f} deg\n"
                f"max_absolute: {item['max_absolute']:.6f} deg\n"
                f"min: {item['min']:.6f} deg\nmax: {item['max']:.6f} deg")
        self.node.get_logger().info(
            f"{'PASS' if passed else 'FAIL'}: 最大误差关节 J{worst + 1}, "
            f"{stats[worst]['max_absolute']:.6f} deg; CSV: {self.csv_path}")
        return passed


def main():
    # Match demo_panel's remappings also for direct `ros2 run` invocation.
    # Explicit user/launch remappings appear first and retain ROS precedence.
    remaps = {
        "/joint_states": "/piperh/driver_joint_states",
        "/rebotarm/joint_states": "/piperh/driver_joint_states",
        "/rebot_xbox/armed": "/piperh/xbox/armed",
        "/execute_trajectory": "/piperh/execute_trajectory",
        "/compute_ik": "/piperh/compute_ik",
        "/plan_kinematic_path": "/piperh/plan_kinematic_path",
        "/display_planned_path": "/piperh/display_planned_path",
    }
    args = [*sys.argv, "--ros-args"]
    for source, target in remaps.items():
        args.extend(["-r", f"{source}:={target}"])
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    operation = None
    passed = False
    try:
        operation = AnchorRepeatability()
        passed = operation.run()
    except (Exception, KeyboardInterrupt) as error:
        message = f"FAIL (测试未完成): {error}"
        if operation is not None:
            operation.node.get_logger().error(message)
        else:
            print(message, file=sys.stderr)
    finally:
        if operation is not None:
            try:
                if rclpy.ok():
                    operation.cancel_motion()
            finally:
                operation.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
