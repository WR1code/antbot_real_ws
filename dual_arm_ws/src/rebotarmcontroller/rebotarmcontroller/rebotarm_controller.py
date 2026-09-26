from __future__ import annotations

import signal

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .servo_trajectory_input import ServoTrajectoryInput


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("hardware_config", "")
        self.declare_parameter("model", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("servo_joint_trajectory_topic", "")
        self.declare_parameter("servo_command_timeout", 0.15)
        self.declare_parameter("trajectory_path_tolerance", 0.03)
        self.declare_parameter("trajectory_goal_tolerance", 0.005)
        self.declare_parameter("trajectory_goal_time_tolerance", 1.5)
        self.declare_parameter("trajectory_stopped_velocity_tolerance", 0.02)
        self.declare_parameter("trajectory_tracking_slowdown_ratio", 0.5)
        self.declare_parameter("trajectory_tracking_stop_ratio", 0.75)
        self.declare_parameter("trajectory_execution_timeout_scaling", 4.0)

        hardware_config = self.get_parameter("hardware_config").value or None
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            hardware_config=hardware_config,
            model=model,
            channel=channel,
        )
        self.hardware.connect()
        self.hardware.initialize()

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )
        servo_topic = str(
            self.get_parameter("servo_joint_trajectory_topic").value or ""
        )
        self.servo_trajectory_input = None
        if servo_topic:
            self.servo_trajectory_input = ServoTrajectoryInput(
                self,
                self.hardware,
                servo_topic,
                float(self.get_parameter("servo_command_timeout").value),
            )
            self.get_logger().info(
                f"hardware Servo input enabled: {servo_topic}"
            )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def shutdown(self) -> None:
        self.hardware.shutdown(reason="normal", request_home=False)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = reBotArmController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    shutdown_error = None
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal Ctrl-C reaches the whole foreground process group, then
        # ros2 launch forwards SIGINT to every child as well.  Block the
        # duplicate signal while the hardware is homed, disabled and closed;
        # otherwise a second KeyboardInterrupt can cut disable_all() short.
        if hasattr(signal, "pthread_sigmask"):
            signal.pthread_sigmask(
                signal.SIG_BLOCK,
                {signal.SIGINT, signal.SIGTERM},
            )
        try:
            node.shutdown()
        except BaseException as exc:  # cleanup must continue after any failure
            shutdown_error = exc
        try:
            executor.shutdown()
        finally:
            try:
                node.destroy_node()
            finally:
                rclpy.try_shutdown()
        if shutdown_error is not None:
            raise RuntimeError("hardware shutdown did not complete cleanly") from shutdown_error


if __name__ == "__main__":
    main()
