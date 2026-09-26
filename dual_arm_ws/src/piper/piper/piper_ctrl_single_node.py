#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import time
import threading
import argparse
import json
import math
import can
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd, PiperTeachJointState
from piper_msgs.srv import Enable
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip
from builtin_interfaces.msg import Time
from .teach_can_grouper import JOINT_IDS, TeachCanGrouper
from .command_authority import CommandAuthority
from .command_writer import PiperCommandWriter


def joint_speed_percent(joint_data):
    """Read the vendor global speed field carried in velocity[6]."""
    if len(joint_data.velocity) == 7 and any(value != 0 for value in joint_data.velocity):
        return int(clip(round(joint_data.velocity[6]), 1, 100))
    return 100

class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('gripper_val_mutiple', 1)
        self.declare_parameter('allow_unsafe_command_topics', False)
        self.declare_parameter('authority_topic', '/piperh/command_authority')

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))
        self.allow_unsafe_command_topics = bool(
            self.get_parameter('allow_unsafe_command_topics').value)

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_single', 1)
        self.joint_feedback_pub = self.create_publisher(JointState, 'joint_states_feedback', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_ctrl', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, 'arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, 'end_pose', 1)
        self.end_pose_stamped_pub = self.create_publisher(PoseStamped, 'end_pose_stamped', 1)
        self.teach_joint_pub = self.create_publisher(
            PiperTeachJointState, 'teach_joint_states_raw', 50)
        # Service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7

        self.joint_states_feedback = JointState()
        self.joint_states_feedback.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states_feedback.position = [0.0] * 7
        self.joint_states_feedback.velocity = [0.0] * 7
        self.joint_states_feedback.effort = [0.0] * 7
        # Joint ctrl
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # Enable flag
        self.__enable_flag = False
        self._last_motion_command = None
        # Create piper class and open CAN interface
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()
        self._command_authority = CommandAuthority()
        self._authorized_writer = PiperCommandWriter(
            self.piper, self._command_authority)

        # A filtered receive-only SocketCAN socket gets copies of the three
        # joint frames. It never transmits and does not alter the SDK socket.
        self._teach_grouper = TeachCanGrouper()
        self._teach_can_stop = threading.Event()
        self._teach_can_bus = can.Bus(
            interface='socketcan', channel=self.can_port,
            can_filters=[{'can_id': value, 'can_mask': 0x7FF, 'extended': False}
                         for value in JOINT_IDS],
        )
        self._teach_can_thread = threading.Thread(
            target=self._teach_can_receive, daemon=True)
        self._teach_can_thread.start()

        # Start subscription thread
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)
        self.create_subscription(
            String, str(self.get_parameter('authority_topic').value),
            self.authority_callback, 10)

        self._publisher_stop = threading.Event()
        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def _teach_can_receive(self):
        last_warning = 0.0
        while not self._teach_can_stop.is_set():
            try:
                frame = self._teach_can_bus.recv(timeout=0.2)
            except can.CanError as exc:
                # shutdown() closes the socket to wake recv(); that is the
                # expected exit path for this receive-only thread.
                if self._teach_can_stop.is_set():
                    break
                now = time.monotonic()
                if now - last_warning >= 1.0:
                    self.get_logger().warning(
                        f'teach CAN feedback receive failed: {exc}')
                    last_warning = now
                continue
            if frame is None or frame.is_extended_id:
                continue
            sample = self._teach_grouper.add(
                frame.arbitration_id, frame.timestamp, bytes(frame.data))
            if sample is None:
                continue
            message = PiperTeachJointState()
            message.sample_timestamp = self.float_to_ros_time(sample.timestamp)
            message.timestamp_j12 = self.float_to_ros_time(sample.pair_timestamps[0])
            message.timestamp_j34 = self.float_to_ros_time(sample.pair_timestamps[1])
            message.timestamp_j56 = self.float_to_ros_time(sample.pair_timestamps[2])
            message.callback_monotonic_time = time.monotonic()
            message.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            message.position = list(sample.positions)
            message.intra_cycle_span = sample.span
            message.dropped_cycles = sample.dropped_cycles
            message.wide_span_cycles = sample.wide_span_cycles
            message.timing_valid = sample.timing_valid
            self.teach_joint_pub.publish(message)
            if sample.span > self._teach_grouper.maximum_span_sec:
                now = time.monotonic()
                if now - last_warning >= 1.0:
                    self.get_logger().warning(
                        f'teach CAN feedback intra-cycle span {sample.span * 1000:.3f} ms '
                        f'exceeds {self._teach_grouper.maximum_span_sec * 1000:.1f} ms')
                    last_warning = now

    def destroy_node(self):
        self._publisher_stop.set()
        self._teach_can_stop.set()
        if hasattr(self, '_teach_can_bus'):
            self._teach_can_bus.shutdown()
        if hasattr(self, '_teach_can_thread'):
            self._teach_can_thread.join(timeout=1.0)
        if hasattr(self, 'publisher_thread'):
            self.publisher_thread.join(timeout=1.0)
        return super().destroy_node()

    def GetEnableFlag(self):
        return self.__enable_flag

    def authority_callback(self, message: String):
        """Acquire or renew the adapter lease; malformed claims fail closed."""
        try:
            claim = json.loads(message.data)
            owner = str(claim['owner'])
            lease = float(claim['lease_seconds'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not owner.startswith('piperh_adapter:') or not 0.05 <= lease <= 2.0:
            return
        now = time.monotonic()
        token = self._command_authority.token_for(owner, now=now)
        if token is None:
            self._command_authority.acquire(
                owner, now=now, lease_seconds=lease)
        else:
            self._command_authority.renew(
                token, now=now, lease_seconds=lease)

    def _authorized_token(self, message):
        header = getattr(message, 'header', None)
        owner = str(getattr(header, 'frame_id', ''))
        return self._command_authority.token_for(owner)

    def _set_motion_mode(self, move_mode, speed_percent):
        """Configure MOVE mode once; repeated mode frames disturb position streaming."""
        command = (int(move_mode), int(speed_percent))
        if command == self._last_motion_command:
            return
        self.piper.MotionCtrl_2(0x01, command[0], command[1])
        self._last_motion_command = command
        self.get_logger().info(
            f"MOVE mode configured: mode={command[0]} speed={command[1]}%")

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        can_was_healthy = False
        last_loss_log = 0.0
        # Set timeout (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok() and not self._publisher_stop.is_set():
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    self.get_logger().info("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    self.get_logger().info(f"Enable status:{enable_flag}")
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    self.get_logger().info("--------------------")
                    # Check if the timeout has been exceeded
                    if elapsed_time > timeout:
                        self.get_logger().info("Timeout....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                self.get_logger().info("Automatic enable timeout, exiting program")
                rclpy.shutdown()
            
            if self.piper.isOk():
                if not can_was_healthy:
                    self.get_logger().info(
                        f"{self.can_port} feedback recovered; motors remain disabled until explicitly enabled"
                    )
                can_was_healthy = True
                self.PublishArmState()
                self.PublishArmJointAndGripper()
                self.PublishArmCtrlAndGripper()
                self.PublishArmEndPose()
            else:
                # Power-cycling the arm makes the SDK report isOk() == False
                # while SocketCAN itself remains usable.  Shutting down here
                # permanently strands the rest of the workbench with stale
                # feedback.  Keep the SDK receive threads alive so fresh CAN
                # frames can recover this node without restarting ROS.
                self.__enable_flag = False
                can_was_healthy = False
                now = time.monotonic()
                if now - last_loss_log >= 2.0:
                    self.get_logger().warning(
                        f"{self.can_port} feedback is unavailable; waiting for the arm to power up"
                    )
                    last_loss_log = now

            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def float_to_ros_time(self, t: float) -> Time:
        ros_time = Time()
        ros_time.sec = int(t)
        ros_time.nanosec = int((t - ros_time.sec) * 1e9)
        return ros_time

    def PublishArmJointAndGripper(self):
        # Assign timestamp
        # self.joint_states.header.stamp = self.get_clock().now().to_msg()
        # JointState.position comes from ArmJointMsgs (0x2Ax).  Motor speed
        # frames can advance independently and must not make a frozen joint
        # position appear fresh to trajectory tracking.
        new_time = self.piper.GetArmJointMsgs().time_stamp
        self.joint_states.header.stamp = self.float_to_ros_time(new_time)
        # Here, you can set the joint positions to any value you want
        # The raw data obtained is in degrees multiplied by 1000. To convert to radians, divide by 1000, multiply by π/180, and limit to 5 decimal places
        joint_0: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
        self.joint_states.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        
        self.joint_states_feedback.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]
        self.joint_states_feedback.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
        self.joint_states_feedback.effort = [effort_0, effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
        self.joint_states_feedback.header.stamp = self.joint_states.header.stamp
        # 发布所有消息
        if any(abs(pos) > 3.5 for pos in self.joint_states_feedback.position):
            self.get_logger().warn("Joint state abnormal: value exceeds ±3.5 rad")
        else:
            self.joint_feedback_pub.publish(self.joint_states_feedback)
            self.joint_pub.publish(self.joint_states)

    def PublishArmCtrlAndGripper(self):
        # self.joint_ctrl.header.stamp = self.get_clock().now().to_msg()
        new_time = max(self.piper.GetArmJointCtrl().time_stamp, self.piper.GetArmGripperCtrl().time_stamp)
        self.joint_ctrl.header.stamp = self.float_to_ros_time(new_time)
        joint_0: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        self.joint_ctrl.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]  # Example values
        if any(abs(pos) > 3.5 for pos in self.joint_ctrl.position):
            self.get_logger().warn("Joint state abnormal: value exceeds ±3.5 rad")
        else:
            self.joint_ctrl_pub.publish(self.joint_ctrl)
    
    def PublishArmEndPose(self):
        new_time = self.piper.GetArmEndPoseMsgs().time_stamp
        # End effector pose
        endpos = Pose()
        endpos.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        endpos.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        endpos.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)
        #  时间戳的endpose
        end_pos_stamp = PoseStamped()
        end_pos_stamp.pose = endpos
        end_pos_stamp.header.stamp = self.float_to_ros_time(new_time)
        # end_pos_stamp.header.stamp = self.get_clock().now().to_msg()
        self.end_pose_stamped_pub.publish(end_pos_stamp)

    def pos_callback(self, pos_data):
        """Callback function for subscribing to the end effector pose

        Args:
            pos_data (): The position data
        """
        # Phase 1 deliberately leaves the subscription present for interface
        # compatibility, but no parameter may turn it back into an SDK bypass.
        self.get_logger().error(
            'unsafe Piper command topics are permanently disabled; '
            'use the guarded Piper-H adapter',
            throttle_duration_sec=1.0)
        return

    def joint_callback(self, joint_data):
        """Callback function for joint angles

        Args:
            joint_data (): The joint data
        """
        authority = self._authorized_token(joint_data)
        if authority is None:
            self.get_logger().error(
                'joint command rejected at final Piper writer: no live authority',
                throttle_duration_sec=1.0)
            return
        factor = 57324.840764  # 1000*180/3.14
        # self.get_logger().info(f"Received Joint States:")

        # 创建一个字典来存储关节名称与位置的映射
        joint_positions = {}
        joint_6 = 0

        # 遍历joint_data.name来映射位置
        for idx, joint_name in enumerate(joint_data.name):
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)
        
        # 获取第7个关节的位置
        if len(joint_data.position) >= 7:
            # self.get_logger().info(f"joint_7: {joint_data.position[6]}")
            joint_6 = round(joint_data.position[6] * 1000 * 1000)
            joint_6 = joint_6 * self.gripper_val_mutiple

        # 控制电机速度
        if self.GetEnableFlag():
            if not self._authorized_writer.call(
                authority,
                lambda: self._set_motion_mode(
                    0x01, joint_speed_percent(joint_data))):
                return

            # 使用关节名称来动态控制关节
            if not self._authorized_writer.joints(authority, (
                joint_positions.get('joint1', 0),
                joint_positions.get('joint2', 0),
                joint_positions.get('joint3', 0),
                joint_positions.get('joint4', 0),
                joint_positions.get('joint5', 0),
                joint_positions.get('joint6', 0)
            )):
                return

            # 夹爪控制
            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    if not math.isnan(gripper_effort):
                        gripper_effort = round(gripper_effort * 1000)
                    else:
                        # self.get_logger().warning("Gripper effort is NaN, using default value.")
                        gripper_effort = 1000  # 设置默认值
                    self._authorized_writer.call(
                        authority,
                        lambda: self.piper.GripperCtrl(
                            abs(joint_6), gripper_effort, 0x01, 0))
                else:
                    self._authorized_writer.call(
                        authority,
                        lambda: self.piper.GripperCtrl(
                            abs(joint_6), 1000, 0x01, 0))


    def enable_callback(self, enable_flag: Bool):
        """Callback function for enabling the robotic arm

        Args:
            enable_flag (): Boolean flag
        """
        self.get_logger().error(
            'unsafe Piper command topics are permanently disabled; '
            'use the guarded motor service',
            throttle_duration_sec=1.0)
        return

    def handle_enable_service(self, req, resp):
        """Handle enable service for the robotic arm"""
        self._last_motion_command = None
        self.get_logger().info(f"Received request: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # Set timeout duration (seconds)
        timeout = 5
        # Record the time before entering the loop
        start_time = time.time()
        while not loop_flag:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)

            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)

            self.get_logger().info(f"Enable status: {enable_flag}")
            self.__enable_flag = enable_flag
            self.get_logger().info(f"--------------------")

            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False

            # Check if timeout duration has been exceeded
            if elapsed_time > timeout:
                self.get_logger().info(f"Timeout...")
                enable_flag = False
                loop_flag = True
                break

            time.sleep(0.5)

        resp.enable_response = enable_flag
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
