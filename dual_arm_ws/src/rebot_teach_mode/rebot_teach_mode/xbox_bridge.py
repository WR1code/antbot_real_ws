"""Map unused Xbox X/B buttons to opt-in teaching services."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class TeachXboxBridge(Node):
    """Hold X to record, press B to replay, and Start to request cancellation."""

    def __init__(self) -> None:
        super().__init__("rebot_teach_xbox_bridge")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("armed_topic", "/rebot_xbox/armed")
        self.declare_parameter("control_target_topic", "/xbox/control_target")
        self.declare_parameter("teach_status_topic", "/rebotarm/teach/status")
        self.declare_parameter("record_button", 2)
        self.declare_parameter("replay_button", 1)
        self.declare_parameter("cancel_button", 7)
        self.declare_parameter("joy_timeout_sec", 0.30)
        namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._record_button = int(self.get_parameter("record_button").value)
        self._replay_button = int(self.get_parameter("replay_button").value)
        self._cancel_button = int(self.get_parameter("cancel_button").value)
        self._joy_timeout = float(self.get_parameter("joy_timeout_sec").value)
        if min(self._record_button, self._replay_button, self._cancel_button) < 0:
            raise ValueError("teach button indices must be non-negative")
        if self._joy_timeout <= 0.0:
            raise ValueError("joy_timeout_sec must be positive")

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._start = self.create_client(
            Trigger, f"/{namespace}/teach/start_recording"
        )
        self._stop = self.create_client(
            Trigger, f"/{namespace}/teach/stop_recording"
        )
        self._replay = self.create_client(Trigger, f"/{namespace}/teach/replay")
        self._cancel = self.create_client(Trigger, f"/{namespace}/teach/cancel")
        self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._joy_callback,
            20,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("armed_topic").value),
            self._armed_callback,
            state_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("control_target_topic").value),
            self._control_target_callback,
            state_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("teach_status_topic").value),
            self._teach_status_callback,
            state_qos,
        )
        self._previous_buttons: list[int] | None = None
        self._xbox_armed: bool | None = None
        self._control_target = "arm"
        self._teach_state = "UNKNOWN"
        self._last_joy = 0.0
        self._record_requested = False
        self._request_pending = False
        self._stop_after_start = False
        self._cancel_after_start = False
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            "[TEACH XBOX] disabled until Xbox reports LOCKED; hold X=record, "
            "B=replay, Start=cancel"
        )

    @staticmethod
    def _button(message: Joy, index: int) -> int:
        return int(message.buttons[index]) if index < len(message.buttons) else 0

    def _edge(self, message: Joy, index: int) -> tuple[bool, bool]:
        current = self._button(message, index)
        previous = (
            int(self._previous_buttons[index])
            if self._previous_buttons is not None and index < len(self._previous_buttons)
            else current
        )
        return current == 1 and previous == 0, current == 0 and previous == 1

    def _joy_callback(self, message: Joy) -> None:
        self._last_joy = time.monotonic()
        if self._previous_buttons is None:
            self._previous_buttons = list(message.buttons)
            return
        record_rise, record_fall = self._edge(message, self._record_button)
        replay_rise, _ = self._edge(message, self._replay_button)
        cancel_rise, _ = self._edge(message, self._cancel_button)
        self._previous_buttons = list(message.buttons)

        if cancel_rise:
            self._record_requested = False
            self._call(self._cancel, "cancel")
            return
        allowed = self._xbox_armed is False and self._control_target == "arm"
        if record_rise:
            if not allowed:
                self.get_logger().warning(
                    "[TEACH XBOX] X ignored: Xbox must be LOCKED and target ARM"
                )
            else:
                self._record_requested = True
                self._call(self._start, "start recording")
        if record_fall and self._record_requested:
            self._record_requested = False
            if self._request_pending:
                self._stop_after_start = True
            else:
                self._call(self._stop, "stop recording")
        if replay_rise and not self._record_requested:
            if not allowed:
                self.get_logger().warning(
                    "[TEACH XBOX] B ignored: Xbox must be LOCKED and target ARM"
                )
            elif not self._request_pending:
                self._call(self._replay, "replay")

    def _armed_callback(self, message: Bool) -> None:
        self._xbox_armed = bool(message.data)
        if self._xbox_armed and (
            self._record_requested
            or self._teach_state in {"RECORDING", "REPLAYING", "CANCELLING"}
        ):
            self._record_requested = False
            if self._request_pending:
                self._cancel_after_start = True
            else:
                self._call(self._cancel, "cancel after Xbox ARMED")

    def _control_target_callback(self, message: String) -> None:
        target = str(message.data).strip().lower()
        if target in {"arm", "base"}:
            self._control_target = target
        if target == "base" and (
            self._record_requested
            or self._teach_state in {"RECORDING", "REPLAYING", "CANCELLING"}
        ):
            self._record_requested = False
            if self._request_pending:
                self._cancel_after_start = True
            else:
                self._call(self._cancel, "cancel after target changed to BASE")

    def _teach_status_callback(self, message: String) -> None:
        try:
            state = str(json.loads(message.data).get("state", "UNKNOWN"))
        except (json.JSONDecodeError, AttributeError):
            return
        self._teach_state = state

    def _call(self, client, label: str) -> None:
        if self._request_pending:
            self.get_logger().warning(f"[TEACH XBOX] {label} ignored: request pending")
            return
        if not client.service_is_ready():
            self.get_logger().error(f"[TEACH XBOX] {label} service unavailable")
            return
        self._request_pending = True
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed, requested=label: self._response(completed, requested)
        )

    def _response(self, future, label: str) -> None:
        self._request_pending = False
        success = False
        try:
            response = future.result()
            success = bool(response.success)
            logger = self.get_logger().info if response.success else self.get_logger().error
            logger(f"[TEACH XBOX] {label}: {response.message}")
        except Exception as error:
            self.get_logger().error(f"[TEACH XBOX] {label} failed: {error}")
        if label == "start recording" and self._cancel_after_start:
            self._cancel_after_start = False
            self._stop_after_start = False
            if success:
                self._call(self._cancel, "cancel interrupted recording")
        elif label == "start recording" and self._stop_after_start:
            self._stop_after_start = False
            if success:
                self._call(self._stop, "stop recording")

    def _watchdog(self) -> None:
        if (
            self._record_requested
            and self._last_joy > 0.0
            and time.monotonic() - self._last_joy > self._joy_timeout
        ):
            self._record_requested = False
            if self._request_pending:
                self._cancel_after_start = True
            else:
                self._call(self._cancel, "cancel after joystick timeout")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachXboxBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
