"""Print only changed Joy axes and buttons to discover a real mapping."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyInspector(Node):
    def __init__(self) -> None:
        super().__init__("inspect_joy")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("noise_threshold", 0.05)
        self.noise_threshold = float(self.get_parameter("noise_threshold").value)
        if self.noise_threshold <= 0.0:
            raise ValueError("noise_threshold must be positive")
        self.previous_buttons: list[int] | None = None
        self.reported_axes: list[float] | None = None
        self.subscription = self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._joy_callback,
            20,
        )
        self.get_logger().info(
            "Operate one control at a time; only changed indices are printed."
        )

    def _joy_callback(self, msg: Joy) -> None:
        if self.previous_buttons is None or self.reported_axes is None:
            self.previous_buttons = list(msg.buttons)
            self.reported_axes = list(msg.axes)
            self.get_logger().info(
                f"JOY READY: {len(msg.axes)} axes, {len(msg.buttons)} buttons"
            )
            return

        old_buttons = self.previous_buttons
        for index, current in enumerate(msg.buttons):
            previous = old_buttons[index] if index < len(old_buttons) else 0
            if current != previous:
                state = "PRESSED" if current else "RELEASED"
                self.get_logger().info(f"BUTTON {index}: {state}")

        if len(self.reported_axes) < len(msg.axes):
            self.reported_axes.extend(msg.axes[len(self.reported_axes) :])
        for index, current in enumerate(msg.axes):
            if abs(current - self.reported_axes[index]) >= self.noise_threshold:
                self.get_logger().info(f"AXIS {index}: {current:.2f}")
                self.reported_axes[index] = current

        self.previous_buttons = list(msg.buttons)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyInspector()
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
