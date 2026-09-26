"""Safe, simulation-only Xbox-to-MoveIt-Servo command node."""

import signal
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger

from rebot_xbox_servo.safety import SafetyState, SpeedSelector, bounded_tier_speed


class RebotXboxTwist(Node):
    """Translate Joy messages into guarded TwistStamped simulation commands."""

    AXIS_NAMES = (
        "linear_x",
        "linear_y",
        "linear_z",
        "angular_yaw",
        "angular_pitch",
    )
    BUTTON_NAMES = (
        "toggle_arm",
        "emergency_stop",
        "roll_left",
        "roll_right",
        "speed_down",
        "speed_up",
        "frame_switch",
        "control_switch",
    )
    TRIGGER_NAMES = ("gripper_open", "gripper_close")

    def __init__(self) -> None:
        super().__init__("rebot_xbox_twist")
        self._declare_parameters()
        self.axes = {
            name: int(self.get_parameter(f"axes.{name}").value)
            for name in self.AXIS_NAMES
        }
        self.buttons = {
            name: int(self.get_parameter(f"buttons.{name}").value)
            for name in self.BUTTON_NAMES
        }
        self.trigger_axes = {
            name: int(self.get_parameter(f"triggers.{name}_axis").value)
            for name in self.TRIGGER_NAMES
        }
        self.trigger_deadzone = float(
            self.get_parameter("triggers.deadzone").value
        )
        if not 0.0 <= self.trigger_deadzone < 1.0:
            raise ValueError("triggers.deadzone must be in [0, 1)")
        self.invert = {
            name: bool(self.get_parameter(f"invert.{name}").value)
            for name in self.AXIS_NAMES
        }

        self.deadzone = float(self.get_parameter("safety.deadzone").value)
        self.joy_timeout = float(self.get_parameter("safety.joy_timeout").value)
        self.require_centered = bool(
            self.get_parameter("safety.require_centered_sticks_to_arm").value
        )
        self.require_released_triggers = bool(
            self.get_parameter("safety.require_released_triggers_to_arm").value
        )
        self.publish_rate = float(self.get_parameter("safety.publish_rate").value)
        self.shutdown_zero_frames = max(
            5,
            int(self.get_parameter("safety.shutdown_zero_frames").value),
        )
        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError("safety.deadzone must be in [0, 1)")
        if self.joy_timeout <= 0.0 or self.publish_rate <= 0.0:
            raise ValueError("joy_timeout and publish_rate must be positive")

        levels = [float(level) for level in self.get_parameter("speed.levels").value]
        self.speed = SpeedSelector(levels)
        self.state = SafetyState()
        self.base_frame = str(self.get_parameter("frames.base").value)
        self.ee_frame = str(self.get_parameter("frames.end_effector").value)
        self.command_frame = self.base_frame
        self.control_target = "arm"
        self.robot_name = str(self.get_parameter("robot_name").value).strip().lower()
        self.robot_selected = not self.robot_name

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("topics.twist").value),
            10,
        )
        self.armed_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("topics.armed").value),
            state_qos,
        )
        self.gripper_open_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("topics.gripper_open").value),
            10,
        )
        self.gripper_close_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("topics.gripper_close").value),
            10,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            str(self.get_parameter("topics.joy").value),
            self._joy_callback,
            20,
        )
        self.control_target_subscription = self.create_subscription(
            String,
            str(self.get_parameter("topics.control_target").value),
            self._control_target_callback,
            state_qos,
        )
        self.robot_selection_subscription = None
        if self.robot_name:
            self.robot_selection_subscription = self.create_subscription(
                String,
                str(self.get_parameter("topics.selected_robot").value),
                self._robot_selection_callback,
                state_qos,
            )
        self.preset_service = self.create_service(
            Trigger,
            str(self.get_parameter("topics.preset_service").value),
            self._preset_callback,
        )
        self.armed_service = self.create_service(
            SetBool,
            str(self.get_parameter("topics.armed_service").value),
            self._set_armed_callback,
        )
        self.servo_mode_client = self.create_client(
            ServoCommandType,
            str(self.get_parameter("topics.servo_switch_service").value),
        )
        self.servo_pause_client = self.create_client(
            SetBool,
            str(self.get_parameter("topics.servo_pause_service").value),
        )
        self.previous_buttons: list[int] = []
        self._last_joy_message: Optional[Joy] = None
        self._triggers_blocked_until_release = False
        self.joy_seen = False
        self.last_joy_time = self.get_clock().now()
        self.last_command = self._zero_message()
        self._reported_mapping_errors: set[str] = set()
        self._servo_mode_request_pending = False
        self._servo_mode_selected = False
        self._last_servo_mode_attempt = 0.0
        self._servo_pause_target = True
        self._servo_pause_applied: Optional[bool] = None
        self._servo_pause_request_pending = False
        self._last_servo_pause_attempt = 0.0
        self._stop_requested = False
        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)

        self._validate_mapping()
        self._publish_armed_state()
        self.get_logger().info("[XBOX SERVO] LOCKED")
        self.get_logger().info(f"[XBOX SERVO] SPEED: {self.speed.percentage}%")
        self.get_logger().info(
            "[XBOX SERVO] DIRECTIONS: left up=-linear.x, left right=+linear.y, "
            "right up=-linear.z, right right=+angular.z"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("device", "/dev/input/js0")
        for name in self.AXIS_NAMES:
            self.declare_parameter(f"axes.{name}", -1)
            self.declare_parameter(f"invert.{name}", False)
        for name in self.BUTTON_NAMES:
            self.declare_parameter(f"buttons.{name}", -1)
        for name in self.TRIGGER_NAMES:
            self.declare_parameter(f"triggers.{name}_axis", -1)
        self.declare_parameter("triggers.deadzone", 0.05)

        self.declare_parameter("limits.linear_speed", 0.08)
        self.declare_parameter("limits.angular_speed", 0.40)
        self.declare_parameter("limits.minimum_linear_speed", 0.005)
        self.declare_parameter("limits.maximum_linear_speed", 0.08)
        self.declare_parameter("limits.minimum_angular_speed", 0.05)
        self.declare_parameter("limits.maximum_angular_speed", 0.50)
        self.declare_parameter("speed.levels", [0.25, 0.50, 1.00])

        self.declare_parameter("safety.deadzone", 0.10)
        self.declare_parameter("safety.joy_timeout", 0.30)
        self.declare_parameter("safety.require_centered_sticks_to_arm", True)
        self.declare_parameter("safety.require_released_triggers_to_arm", True)
        self.declare_parameter("safety.require_armed_for_gripper", True)
        self.declare_parameter("safety.publish_rate", 20.0)
        self.declare_parameter("safety.shutdown_zero_frames", 5)

        self.declare_parameter("frames.base", "base_link")
        self.declare_parameter("frames.end_effector", "gripper_tcp")
        # Empty keeps the historical single-arm behavior.  A non-empty name
        # enables the dual-arm ownership interlock.
        self.declare_parameter("robot_name", "")
        self.declare_parameter("topics.joy", "/joy")
        self.declare_parameter("topics.twist", "/servo_node/delta_twist_cmds")
        self.declare_parameter("topics.armed", "/rebot_xbox/armed")
        self.declare_parameter("topics.armed_service", "/rebot_xbox/set_armed")
        self.declare_parameter("topics.gripper_open", "/rebot_xbox/gripper_open")
        self.declare_parameter("topics.gripper_close", "/rebot_xbox/gripper_close")
        self.declare_parameter("topics.control_target", "/xbox/control_target")
        self.declare_parameter("topics.selected_robot", "/dual_arm/selected")
        self.declare_parameter("topics.preset_service", "/rebot_xbox/go_to_preset")
        self.declare_parameter(
            "topics.servo_switch_service", "/servo_node/switch_command_type"
        )
        self.declare_parameter(
            "topics.servo_pause_service", "/servo_node/pause_servo"
        )

    def _validate_mapping(self) -> None:
        unassigned_axes = [name for name, index in self.axes.items() if index < 0]
        unassigned_buttons = [name for name, index in self.buttons.items() if index < 0]
        unassigned_triggers = (
            [name for name, index in self.trigger_axes.items() if index < 0]
            if self.require_released_triggers
            else []
        )
        if unassigned_axes or unassigned_buttons or unassigned_triggers:
            self.get_logger().warning(
                "[XBOX SERVO] Mapping is intentionally disabled until inspect_joy "
                "confirms real indices. Unassigned axes=%s buttons=%s triggers=%s"
                % (unassigned_axes, unassigned_buttons, unassigned_triggers)
            )

    def _button_value(self, msg: Joy, index: int, name: str) -> int:
        if index < 0:
            return 0
        if index >= len(msg.buttons):
            self._report_mapping_error(
                f"button:{name}",
                f"button {name} index {index} is outside buttons[0:{len(msg.buttons)}]",
            )
            return 0
        return int(msg.buttons[index])

    def _previous_button_value(self, index: int) -> int:
        if index < 0 or index >= len(self.previous_buttons):
            return 0
        return int(self.previous_buttons[index])

    def _axis_value(
        self, msg: Joy, name: str, *, reject_invalid: bool = False
    ) -> Optional[float]:
        index = self.axes[name]
        if index < 0:
            return None if reject_invalid else 0.0
        if index >= len(msg.axes):
            self._report_mapping_error(
                f"axis:{name}",
                f"axis {name} index {index} is outside axes[0:{len(msg.axes)}]",
            )
            return None if reject_invalid else 0.0
        value = float(msg.axes[index])
        if self.invert[name]:
            value = -value
        return max(-1.0, min(1.0, value))

    def _report_mapping_error(self, key: str, text: str) -> None:
        if key not in self._reported_mapping_errors:
            self._reported_mapping_errors.add(key)
            self.get_logger().error(f"[XBOX SERVO] {text}")

    def _rising_edge(self, msg: Joy, name: str) -> bool:
        index = self.buttons[name]
        current = self._button_value(msg, index, name)
        previous = self._previous_button_value(index)
        return current == 1 and previous == 0

    def _trigger_amount(self, msg: Joy, name: str) -> float:
        index = self.trigger_axes[name]
        if index < 0:
            return 0.0
        if index >= len(msg.axes):
            self._report_mapping_error(
                f"trigger:{name}",
                f"trigger {name} axis {index} is outside axes[0:{len(msg.axes)}]",
            )
            return 0.0
        # Measured Generic X-Box pad: released=+1.0, fully pressed=-1.0.
        raw_value = max(-1.0, min(1.0, float(msg.axes[index])))
        amount = (1.0 - raw_value) / 2.0
        return 0.0 if amount <= self.trigger_deadzone else amount

    def _trigger_raw_values(self, msg: Joy) -> Optional[dict[str, float]]:
        """Return clamped trigger axes, or None when the mapping is invalid."""
        values: dict[str, float] = {}
        for name in self.TRIGGER_NAMES:
            index = self.trigger_axes[name]
            if index < 0 or index >= len(msg.axes):
                self._report_mapping_error(
                    f"trigger:{name}",
                    f"trigger {name} axis {index} is outside axes[0:{len(msg.axes)}]",
                )
                return None
            values[name] = max(-1.0, min(1.0, float(msg.axes[index])))
        return values

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy_time = self.get_clock().now()
        self._last_joy_message = msg
        recovering = self.state.joystick_timed_out
        self.joy_seen = True
        current_triggers = {
            name: self._trigger_amount(msg, name)
            for name in self.TRIGGER_NAMES
        }

        # Keep an edge baseline while inactive.  Consequently a held A button
        # can never arm the newly selected robot during a handoff.
        if not self.robot_selected:
            self.previous_buttons = list(msg.buttons)
            self.state.armed = False
            self._set_servo_pause_target(True)
            self.last_command = self._zero_message()
            self._publish_gripper_velocity(0.0, 0.0)
            return

        # Holding A while reconnecting must not unlock without release + press.
        if not self.previous_buttons or recovering:
            self.previous_buttons = list(msg.buttons)
            self._triggers_blocked_until_release = any(current_triggers.values())
            self.state.reconnect()
            self.last_command = self._zero_message()
            self._publish_gripper_velocity(0.0, 0.0)
            return

        if self.control_target != "arm":
            self.previous_buttons = list(msg.buttons)
            self.last_command = self._zero_message()
            self._publish_gripper_velocity(0.0, 0.0)
            return

        a_index = self.buttons["toggle_arm"]
        current_a = self._button_value(msg, a_index, "toggle_arm")
        previous_a = self._previous_button_value(a_index)
        a_pressed = current_a == 1 and previous_a == 0

        emergency_pressed = self._rising_edge(msg, "emergency_stop")
        if emergency_pressed:
            self._emergency_stop()
        elif a_pressed:
            self._toggle_armed(msg)

        if a_pressed or emergency_pressed:
            self._triggers_blocked_until_release = any(current_triggers.values())
        elif not any(current_triggers.values()):
            self._triggers_blocked_until_release = False

        if self._rising_edge(msg, "speed_up") and self.speed.increase():
            self._log_speed()
        if self._rising_edge(msg, "speed_down") and self.speed.decrease():
            self._log_speed()
        if self._rising_edge(msg, "frame_switch"):
            self.command_frame = (
                self.ee_frame if self.command_frame == self.base_frame else self.base_frame
            )
            self.get_logger().info(f"[XBOX SERVO] FRAME: {self.command_frame}")
        require_armed_for_gripper = bool(
            self.get_parameter("safety.require_armed_for_gripper").value
        )
        # Never combine an ownership/safety-button transition and a gripper
        # command from the same controller report.
        gripper_enabled = (
            (self.state.armed or not require_armed_for_gripper)
            and not a_pressed
            and not emergency_pressed
        )
        open_amount = current_triggers["gripper_open"]
        close_amount = current_triggers["gripper_close"]
        triggers_conflict = open_amount > 0.0 and close_amount > 0.0
        if (
            gripper_enabled
            and not self._triggers_blocked_until_release
            and not triggers_conflict
        ):
            self._publish_gripper_velocity(open_amount, close_amount)
        else:
            self._publish_gripper_velocity(0.0, 0.0)

        self.last_command = (
            self._command_from_joy(msg) if self.state.armed else self._zero_message()
        )
        self.previous_buttons = list(msg.buttons)

    def _control_target_callback(self, msg: String) -> None:
        """Lock immediately when shared Xbox ownership moves to the base."""
        target = str(msg.data).strip().lower()
        if target not in ("arm", "base"):
            self.get_logger().warning(
                f"[XBOX SERVO] Ignoring invalid control target: {msg.data!r}"
            )
            return
        if target == self.control_target:
            return
        self.control_target = target
        if target == "base":
            self._lock_with_log("[XBOX SERVO] CONTROL: BASE - LOCKED")
        else:
            self._lock_with_log(
                "[XBOX SERVO] CONTROL: ARM - press A to arm"
            )

    def _robot_selection_callback(self, msg: String) -> None:
        """Lock immediately unless this instance owns the shared controller."""
        selected = str(msg.data).strip().lower()
        is_selected = selected == self.robot_name
        if is_selected == self.robot_selected:
            return
        self.robot_selected = is_selected
        if is_selected:
            self.previous_buttons = []
            self._lock_with_log(
                f"[XBOX SERVO] SELECTED: {self.robot_name} - press A to arm"
            )
        else:
            self._lock_with_log(
                f"[XBOX SERVO] DESELECTED: {self.robot_name} - LOCKED"
            )

    def _publish_gripper_velocity(
        self, open_amount: float, close_amount: float
    ) -> None:
        self.gripper_open_publisher.publish(Float64(data=float(open_amount)))
        self.gripper_close_publisher.publish(Float64(data=float(close_amount)))

    def _toggle_armed(self, msg: Joy) -> None:
        result = self.state.toggle(self._sticks_centered(msg), self.require_centered)
        if result == "center_required":
            self.get_logger().warning("[XBOX SERVO] 请先将摇杆回中")
            return
        if result == "armed":
            self.last_command = self._zero_message()
            self._set_servo_pause_target(False)
            self._publish_armed_state()
            self.get_logger().info("[XBOX SERVO] ARMED")
            return
        self._lock_with_log("[XBOX SERVO] LOCKED")

    def _emergency_stop(self) -> None:
        self.state.emergency_stop()
        self._lock_with_log("[XBOX SERVO] EMERGENCY STOP")

    def _lock_with_log(self, log_text: str) -> None:
        self.state.armed = False
        self._set_servo_pause_target(True)
        self.last_command = self._zero_message()
        self._publish_zero()
        self._publish_gripper_velocity(0.0, 0.0)
        self._publish_armed_state()
        self.get_logger().warning(log_text)

    def _sticks_centered(self, msg: Joy) -> bool:
        for name in self.AXIS_NAMES:
            value = self._axis_value(msg, name, reject_invalid=True)
            if value is None or abs(value) > self.deadzone:
                return False
        return True

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) <= self.deadzone else value

    def _effective_speeds(self) -> tuple[float, float]:
        linear = bounded_tier_speed(
            float(self.get_parameter("limits.linear_speed").value),
            self.speed.multiplier,
            float(self.get_parameter("limits.minimum_linear_speed").value),
            float(self.get_parameter("limits.maximum_linear_speed").value),
        )
        angular = bounded_tier_speed(
            float(self.get_parameter("limits.angular_speed").value),
            self.speed.multiplier,
            float(self.get_parameter("limits.minimum_angular_speed").value),
            float(self.get_parameter("limits.maximum_angular_speed").value),
        )
        return linear, angular

    def _command_from_joy(self, msg: Joy) -> TwistStamped:
        linear_speed, angular_speed = self._effective_speeds()
        command = self._zero_message()
        command.twist.linear.x = self._apply_deadzone(
            self._axis_value(msg, "linear_x") or 0.0
        ) * linear_speed
        command.twist.linear.y = self._apply_deadzone(
            self._axis_value(msg, "linear_y") or 0.0
        ) * linear_speed
        command.twist.linear.z = self._apply_deadzone(
            self._axis_value(msg, "linear_z") or 0.0
        ) * linear_speed
        command.twist.angular.y = self._apply_deadzone(
            self._axis_value(msg, "angular_pitch") or 0.0
        ) * angular_speed
        command.twist.angular.z = self._apply_deadzone(
            self._axis_value(msg, "angular_yaw") or 0.0
        ) * angular_speed

        roll_left = self._button_value(msg, self.buttons["roll_left"], "roll_left")
        roll_right = self._button_value(msg, self.buttons["roll_right"], "roll_right")
        command.twist.angular.x = (roll_right - roll_left) * angular_speed
        return command

    def _zero_message(self) -> TwistStamped:
        message = TwistStamped()
        message.header.frame_id = getattr(self, "command_frame", "base_link")
        return message

    def _stamp(self, message: TwistStamped) -> TwistStamped:
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.command_frame
        return message

    def _publish_zero(self) -> None:
        self.twist_publisher.publish(self._stamp(self._zero_message()))

    def _publish_armed_state(self) -> None:
        self.armed_publisher.publish(Bool(data=self.state.armed))

    def _timer_callback(self) -> None:
        now = self.get_clock().now()
        if self.joy_seen and not self.state.joystick_timed_out:
            age = (now - self.last_joy_time).nanoseconds / 1_000_000_000.0
            if age > self.joy_timeout:
                self.state.timeout()
                self._lock_with_log("[XBOX SERVO] JOYSTICK TIMEOUT - LOCKED")

        # Servo rejects every command received before a command type is selected.
        # Select TWIST first and do not even publish a zero frame during that
        # startup window.  A zero frame is unnecessary before Servo has accepted
        # any input and used to produce a 100 Hz warning storm.
        self._try_select_servo_twist_mode()
        self._try_set_servo_pause()
        if not self._servo_mode_selected:
            return

        if self.state.armed and self.control_target == "arm" and self.robot_selected:
            self.twist_publisher.publish(self._stamp(self.last_command))
        else:
            self._publish_zero()

    def _try_select_servo_twist_mode(self) -> None:
        if self._servo_mode_selected or self._servo_mode_request_pending:
            return
        now = time.monotonic()
        if now - self._last_servo_mode_attempt < 1.0:
            return
        self._last_servo_mode_attempt = now
        if not self.servo_mode_client.service_is_ready():
            return
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        self._servo_mode_request_pending = True
        future = self.servo_mode_client.call_async(request)
        future.add_done_callback(self._servo_mode_response)

    def _servo_mode_response(self, future) -> None:
        self._servo_mode_request_pending = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                f"[XBOX SERVO] Failed to select Servo TWIST mode: {error}"
            )
            return
        if response.success:
            self._servo_mode_selected = True
            self.get_logger().info("[XBOX SERVO] MoveIt Servo TWIST mode ready")

    def _set_servo_pause_target(self, paused: bool) -> None:
        """Select which command source is allowed to own the arm controller."""
        self._servo_pause_target = paused

    def _try_set_servo_pause(self) -> None:
        """Pause for RViz; unpause only while Xbox is armed."""
        if self._servo_pause_request_pending:
            return
        if self._servo_pause_applied == self._servo_pause_target:
            return
        now = time.monotonic()
        if now - self._last_servo_pause_attempt < 0.25:
            return
        self._last_servo_pause_attempt = now
        if not self.servo_pause_client.service_is_ready():
            return
        paused = self._servo_pause_target
        self._servo_pause_request_pending = True
        request = SetBool.Request(data=paused)
        future = self.servo_pause_client.call_async(request)
        future.add_done_callback(
            lambda completed, requested=paused: self._servo_pause_response(
                completed, requested
            )
        )

    def _servo_pause_response(self, future, paused: bool) -> None:
        self._servo_pause_request_pending = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover
            self.get_logger().error(
                f"[XBOX SERVO] Failed to switch controller owner: {error}"
            )
            return
        if not response.success:
            text = response.message
            self.get_logger().error(
                f"[XBOX SERVO] Servo pause request rejected: {text}"
            )
            return
        self._servo_pause_applied = paused
        owner = "RVIZ / MOVEGROUP" if paused else "XBOX SERVO"
        self.get_logger().info(f"[XBOX SERVO] CONTROL: {owner}")

    def _preset_callback(self, request, response):
        del request
        response.success = False
        response.message = (
            "Reserved simulation-only preset interface; no Twist command is used."
        )
        return response

    def _set_armed_callback(self, request, response):
        """Safely transfer motion ownership between RViz and the Xbox pad."""
        if not bool(request.data):
            self._lock_with_log("[XBOX SERVO] LOCKED from RViz")
            response.success = True
            response.message = "Xbox control locked; zero command published"
            return response

        if self.state.armed:
            response.success = True
            response.message = "Xbox control is already armed"
            return response
        if self.control_target != "arm":
            response.success = False
            response.message = "Xbox control target must be ARM"
            return response
        if not self.robot_selected:
            response.success = False
            response.message = f"Robot {self.robot_name} is not selected"
            return response
        if not self.joy_seen or self._last_joy_message is None:
            response.success = False
            response.message = "Xbox controller has not reported a fresh input"
            return response
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if age > self.joy_timeout or self.state.joystick_timed_out:
            response.success = False
            response.message = "Xbox controller input is stale; move and center the sticks"
            return response
        if self.require_centered and not self._sticks_centered(
            self._last_joy_message
        ):
            response.success = False
            response.message = "center all Xbox sticks before enabling control"
            return response
        if self.require_released_triggers:
            trigger_raw = self._trigger_raw_values(self._last_joy_message)
            if trigger_raw is None:
                response.success = False
                response.message = "Xbox trigger axis mapping is invalid"
                return response
            # Linux joystick drivers commonly report both trigger axes as 0.0
            # until each trigger has moved once. Treat that ambiguous midpoint as
            # unsafe instead of silently arming or calling it a released trigger.
            uninitialized = [
                name for name, value in trigger_raw.items()
                if abs(value) <= self.trigger_deadzone
            ]
            if uninitialized:
                response.success = False
                response.message = (
                    "trigger axes are not initialized; fully press and release "
                    "LT and RT once, then enable Xbox control again "
                    f"(LT={trigger_raw['gripper_open']:.2f}, "
                    f"RT={trigger_raw['gripper_close']:.2f})"
                )
                return response
            trigger_amounts = (
                self._trigger_amount(self._last_joy_message, name)
                for name in self.TRIGGER_NAMES
            )
            if any(amount > 0.0 for amount in trigger_amounts):
                response.success = False
                response.message = "release both Xbox triggers before enabling control"
                return response

        self.state.armed = True
        self.state.joystick_timed_out = False
        self.previous_buttons = list(self._last_joy_message.buttons)
        self._triggers_blocked_until_release = False
        self.last_command = self._zero_message()
        self._set_servo_pause_target(False)
        self._publish_armed_state()
        self.get_logger().info("[XBOX SERVO] ARMED from RViz")
        response.success = True
        response.message = (
            "Xbox control armed; sticks and triggers verified centered"
            if self.require_released_triggers
            else "Xbox control armed; sticks verified centered"
        )
        return response

    def _log_speed(self) -> None:
        self.get_logger().info(f"[XBOX SERVO] SPEED: {self.speed.percentage}%")

    def request_stop(self) -> None:
        self._stop_requested = True

    def publish_shutdown_zeros(self) -> None:
        was_armed = self.state.armed
        self.state.armed = False
        self._set_servo_pause_target(True)
        self._publish_armed_state()
        if was_armed:
            self.get_logger().warning("[XBOX SERVO] LOCKED")
        if self._servo_mode_selected:
            period = 1.0 / self.publish_rate
            for _ in range(self.shutdown_zero_frames):
                self._publish_zero()
                time.sleep(period)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RebotXboxTwist()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def handle_signal(_signum, _frame) -> None:
        node.request_stop()

    old_sigint = signal.signal(signal.SIGINT, handle_signal)
    old_sigterm = signal.signal(signal.SIGTERM, handle_signal)
    try:
        while rclpy.ok() and not node._stop_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.publish_shutdown_zeros()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == "__main__":
    main()
