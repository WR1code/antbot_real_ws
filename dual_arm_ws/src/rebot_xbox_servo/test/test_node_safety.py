"""Exercise the real ROS node callbacks with deterministic Joy messages."""

from copy import deepcopy
import os

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rebot_xbox_servo.xbox_twist import RebotXboxTwist


class RecordingPublisher:
    """Minimal publisher double that preserves message snapshots."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deepcopy(message))


def joy(axes=None, pressed=()):
    """Construct a synthetic controller snapshot."""
    message = Joy()
    message.axes = list(axes or [0.0] * 5)
    message.axes.extend([1.0] * (7 - len(message.axes)))
    message.buttons = [0] * 9
    for index in pressed:
        message.buttons[index] = 1
    return message


def all_zero(message):
    """Return whether every Cartesian command component is zero."""
    values = (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )
    return all(value == 0.0 for value in values)


def test_node_toggle_timeout_emergency_and_center_interlock():
    """Verify the complete safety sequence against the actual node methods."""
    os.environ['ROS_LOG_DIR'] = '/tmp/rebot_xbox_servo_test_logs'
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)
    parameters = [
        'axes.linear_x:=0',
        'axes.linear_y:=1',
        'axes.linear_z:=2',
        'axes.angular_yaw:=3',
        'axes.angular_pitch:=4',
        'buttons.toggle_arm:=0',
        'buttons.emergency_stop:=1',
        'buttons.roll_left:=4',
        'buttons.roll_right:=5',
        'buttons.speed_down:=6',
        'buttons.speed_up:=7',
        'buttons.frame_switch:=8',
        'triggers.gripper_open_axis:=5',
        'triggers.gripper_close_axis:=6',
    ]
    arguments = ['--ros-args']
    for parameter in parameters:
        arguments.extend(['-p', parameter])
    rclpy.init(args=arguments)
    node = RebotXboxTwist()
    twist = RecordingPublisher()
    armed = RecordingPublisher()
    node.twist_publisher = twist
    node.armed_publisher = armed
    gripper_open = RecordingPublisher()
    gripper_close = RecordingPublisher()
    node.gripper_open_publisher = gripper_open
    node.gripper_close_publisher = gripper_close
    try:
        assert node.state.armed is False
        assert node._servo_pause_target is True
        assert node.timer.timer_period_ns == 50_000_000

        # No command, including zero, may reach Servo before it has accepted
        # the TWIST command type.  This prevents the startup warning storm.
        node._timer_callback()
        assert twist.messages == []
        node._servo_mode_selected = True
        node._timer_callback()
        assert all_zero(twist.messages[-1])

        response = node._set_armed_callback(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success is False
        assert node.state.armed is False

        node._joy_callback(joy())  # neutral startup baseline
        response = node._set_armed_callback(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success is True
        assert response.message == (
            "Xbox control armed; sticks and triggers verified centered"
        )
        assert node.state.armed is True
        response = node._set_armed_callback(
            SetBool.Request(data=False), SetBool.Response()
        )
        assert response.success is True
        assert node.state.armed is False
        assert all_zero(twist.messages[-1])

        uninitialized_triggers = joy()
        uninitialized_triggers.axes[5] = 0.0
        uninitialized_triggers.axes[6] = 0.0
        node._joy_callback(uninitialized_triggers)
        response = node._set_armed_callback(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success is False
        assert "fully press and release LT and RT once" in response.message
        assert node.state.armed is False
        node._joy_callback(joy())

        node._joy_callback(joy([0.8, 0.0, 0.0, 0.0, 0.0]))
        response = node._set_armed_callback(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success is False
        assert "center" in response.message
        node._timer_callback()
        assert all_zero(twist.messages[-1])

        arm_and_close = joy(pressed=[0])
        arm_and_close.axes[6] = -1.0
        node._joy_callback(arm_and_close)
        assert node.state.armed is True
        assert node._servo_pause_target is False
        assert gripper_close.messages[-1].data == 0.0
        node._joy_callback(joy())

        lt_pressed = joy()
        lt_pressed.axes[5] = 0.0  # 50% travel -> 50% opening speed
        node._joy_callback(lt_pressed)
        node._joy_callback(lt_pressed)
        assert gripper_open.messages[-1].data == 0.5
        lt_pressed.axes[5] = -1.0
        node._joy_callback(lt_pressed)
        assert gripper_open.messages[-1].data == 1.0
        node._joy_callback(joy())
        assert gripper_open.messages[-1].data == 0.0

        rt_pressed = joy()
        rt_pressed.axes[6] = -1.0
        node._joy_callback(rt_pressed)
        node._joy_callback(rt_pressed)
        assert gripper_close.messages[-1].data == 1.0
        node._joy_callback(joy())
        assert gripper_close.messages[-1].data == 0.0

        assert node.state.armed is True

        node._joy_callback(joy())
        node._joy_callback(joy([0.8, 0.0, 0.0, 0.0, 0.0]))
        node._timer_callback()
        assert twist.messages[-1].twist.linear.x != 0.0

        node._control_target_callback(String(data='base'))
        assert node.control_target == 'base'
        assert node.state.armed is False
        assert all_zero(twist.messages[-1])
        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is False
        node._joy_callback(joy())
        node._control_target_callback(String(data='arm'))
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is True
        node._joy_callback(joy())

        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is False
        assert node._servo_pause_target is True
        assert all_zero(twist.messages[-1])
        node._joy_callback(rt_pressed)
        assert gripper_close.messages[-1].data == 0.0
        node._joy_callback(joy())
        node._joy_callback(joy([0.8, 0.0, 0.0, 0.0, 0.0]))
        node._timer_callback()
        assert all_zero(twist.messages[-1])

        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is True
        node.last_joy_time = node.get_clock().now() - Duration(seconds=1.0)
        node._timer_callback()
        assert node.state.armed is False
        assert node.state.joystick_timed_out is True
        assert all_zero(twist.messages[-1])

        node._joy_callback(joy(pressed=[0]))  # held on reconnect: baseline only
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is False
        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is True

        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[1]))
        assert node.state.armed is False
        assert all_zero(twist.messages[-1])

        node._joy_callback(joy())
        node._joy_callback(joy([0.5, 0.0, 0.0, 0.0, 0.0], pressed=[0]))
        assert node.state.armed is False
        assert armed.messages[-1].data is False

        before_shutdown = len(twist.messages)
        node.shutdown_zero_frames = 5
        node.publish_shutdown_zeros()
        shutdown_messages = twist.messages[before_shutdown:]
        assert len(shutdown_messages) == 5
        assert all(all_zero(message) for message in shutdown_messages)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_dual_arm_selection_locks_and_requires_a_fresh_a_press():
    """A held button cannot carry ARMED state across an ownership handoff."""
    arguments = [
        '--ros-args',
        '-p', 'robot_name:=rebotarm',
        '-p', 'axes.linear_x:=0',
        '-p', 'axes.linear_y:=1',
        '-p', 'axes.linear_z:=2',
        '-p', 'axes.angular_yaw:=3',
        '-p', 'axes.angular_pitch:=4',
        '-p', 'buttons.toggle_arm:=0',
        '-p', 'buttons.emergency_stop:=1',
        '-p', 'buttons.roll_left:=4',
        '-p', 'buttons.roll_right:=5',
        '-p', 'buttons.speed_down:=6',
        '-p', 'buttons.speed_up:=7',
        '-p', 'buttons.frame_switch:=8',
        '-p', 'triggers.gripper_open_axis:=5',
        '-p', 'triggers.gripper_close_axis:=6',
    ]
    rclpy.init(args=arguments)
    node = RebotXboxTwist()
    node.twist_publisher = RecordingPublisher()
    node.armed_publisher = RecordingPublisher()
    node.gripper_open_publisher = RecordingPublisher()
    node.gripper_close_publisher = RecordingPublisher()
    try:
        assert node.robot_selected is False
        node._robot_selection_callback(String(data='rebotarm'))
        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is True

        node._robot_selection_callback(String(data='piperh'))
        assert node.state.armed is False
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is False

        node._robot_selection_callback(String(data='rebotarm'))
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is False
        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.state.armed is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_no_gripper_robot_can_arm_without_trigger_axes():
    """A no-gripper robot must not require nonexistent LT/RT mappings."""
    arguments = [
        '--ros-args',
        '-p', 'safety.require_released_triggers_to_arm:=false',
        '-p', 'axes.linear_x:=0',
        '-p', 'axes.linear_y:=1',
        '-p', 'axes.linear_z:=2',
        '-p', 'axes.angular_yaw:=3',
        '-p', 'axes.angular_pitch:=4',
        '-p', 'buttons.toggle_arm:=0',
        '-p', 'buttons.emergency_stop:=1',
        '-p', 'buttons.roll_left:=4',
        '-p', 'buttons.roll_right:=5',
        '-p', 'buttons.speed_down:=6',
        '-p', 'buttons.speed_up:=7',
        '-p', 'buttons.frame_switch:=8',
    ]
    rclpy.init(args=arguments)
    node = RebotXboxTwist()
    node.twist_publisher = RecordingPublisher()
    node.armed_publisher = RecordingPublisher()
    node.gripper_open_publisher = RecordingPublisher()
    node.gripper_close_publisher = RecordingPublisher()
    try:
        assert node.trigger_axes == {
            'gripper_open': -1,
            'gripper_close': -1,
        }
        node._joy_callback(joy())
        response = node._set_armed_callback(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success is True
        assert response.message == "Xbox control armed; sticks verified centered"
        assert node.state.armed is True
    finally:
        node.destroy_node()
        rclpy.shutdown()
