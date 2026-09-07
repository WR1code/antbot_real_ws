#!/usr/bin/env python3
# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""RViz interactive waypoint editor and Nav2 route dispatcher for ANTBot."""

import math
from pathlib import Path

from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
import yaml


def normalized_pose(pose):
    """Return a planar pose with a valid quaternion."""
    result = Pose()
    result.position.x = float(pose.position.x)
    result.position.y = float(pose.position.y)
    result.position.z = 0.0
    norm = math.hypot(pose.orientation.z, pose.orientation.w)
    if norm < 1e-9:
        result.orientation.w = 1.0
    else:
        result.orientation.z = pose.orientation.z / norm
        result.orientation.w = pose.orientation.w / norm
    return result


class WaypointEditor(Node):

    def __init__(self):
        super().__init__('antbot_waypoint_editor')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('autoload', True)

        configured = str(self.get_parameter('waypoints_file').value)
        self.waypoints_file = Path(
            configured or '~/.ros/antbot_waypoints.yaml').expanduser()
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.poses = {}
        self.next_index = 1
        self.route_goal_handle = None

        self.server = InteractiveMarkerServer(self, 'antbot_waypoints')
        self.menu = MenuHandler()
        self.menu.insert('导航到此航点', callback=self.navigate_one)
        self.menu.insert('删除此航点', callback=self.delete_one)

        self.create_subscription(
            PointStamped, '/clicked_point', self.add_clicked_point, 10)
        self.route_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.create_service(Trigger, '~/save', self.save_service)
        self.create_service(Trigger, '~/start', self.start_service)
        self.create_service(Trigger, '~/cancel', self.cancel_service)
        self.create_service(Empty, '~/clear', self.clear_service)
        self.create_service(Trigger, '~/reload', self.reload_service)

        if self.get_parameter('autoload').value:
            self.load()
        self.get_logger().info(
            f'航点编辑器已启动；文件：{self.waypoints_file}')

    def marker_for(self, name, pose):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = name
        marker.description = name
        marker.scale = 0.55
        marker.pose = normalized_pose(pose)

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = 0.42
        arrow.scale.y = 0.13
        arrow.scale.z = 0.13
        arrow.color.r = 0.1
        arrow.color.g = 0.75
        arrow.color.b = 1.0
        arrow.color.a = 1.0

        display = InteractiveMarkerControl()
        display.always_visible = True
        display.interaction_mode = InteractiveMarkerControl.MENU
        display.markers.append(arrow)
        marker.controls.append(display)

        move = InteractiveMarkerControl()
        move.name = 'move_xy'
        move.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move.orientation.w = math.sqrt(0.5)
        move.orientation.y = math.sqrt(0.5)
        marker.controls.append(move)

        rotate = InteractiveMarkerControl()
        rotate.name = 'rotate_z'
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate.orientation.w = math.sqrt(0.5)
        rotate.orientation.y = math.sqrt(0.5)
        marker.controls.append(rotate)
        return marker

    def insert_marker(self, name, pose):
        self.poses[name] = normalized_pose(pose)
        self.server.insert(self.marker_for(name, pose), feedback_callback=self.feedback)
        self.menu.apply(self.server, name)
        self.server.applyChanges()

    def add_clicked_point(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f'忽略 {msg.header.frame_id} 坐标系点；请在 RViz 使用 {self.frame_id}')
            return
        pose = Pose()
        pose.position = msg.point
        pose.orientation.w = 1.0
        name = f'wp_{self.next_index:02d}'
        self.next_index += 1
        self.insert_marker(name, pose)
        self.get_logger().info(f'已添加 {name}')

    def feedback(self, feedback):
        if feedback.event_type in (
                feedback.POSE_UPDATE, feedback.MOUSE_UP):
            self.poses[feedback.marker_name] = normalized_pose(feedback.pose)

    def delete_one(self, feedback):
        name = feedback.marker_name
        self.poses.pop(name, None)
        self.server.erase(name)
        self.server.applyChanges()
        self.get_logger().info(f'已删除 {name}')

    def stamped(self, pose):
        result = PoseStamped()
        result.header.frame_id = self.frame_id
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose = normalized_pose(pose)
        return result

    def navigate_one(self, feedback):
        if not self.pose_client.server_is_ready():
            self.get_logger().error('Nav2 navigate_to_pose 动作服务器尚未就绪')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.stamped(self.poses[feedback.marker_name])
        self.pose_client.send_goal_async(goal)
        self.get_logger().info(f'开始导航到 {feedback.marker_name}')

    def ordered_names(self):
        def key(name):
            suffix = name.rsplit('_', 1)[-1]
            return (0, int(suffix)) if suffix.isdigit() else (1, name)
        return sorted(self.poses, key=key)

    def start_service(self, _request, response):
        names = self.ordered_names()
        if not names:
            response.success = False
            response.message = '没有航点'
            return response
        if not self.route_client.server_is_ready():
            response.success = False
            response.message = 'Nav2 navigate_through_poses 尚未就绪'
            return response
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.stamped(self.poses[name]) for name in names]
        future = self.route_client.send_goal_async(goal)
        future.add_done_callback(self.route_accepted)
        response.success = True
        response.message = f'已发送 {len(names)} 个航点'
        return response

    def route_accepted(self, future):
        self.route_goal_handle = future.result()
        if not self.route_goal_handle.accepted:
            self.get_logger().error('Nav2 拒绝了航点路线')
            self.route_goal_handle = None
            return
        self.get_logger().info('Nav2 已接受航点路线')
        result = self.route_goal_handle.get_result_async()
        result.add_done_callback(self.route_finished)

    def route_finished(self, future):
        self.get_logger().info(f'航点路线结束，状态码：{future.result().status}')
        self.route_goal_handle = None

    def cancel_service(self, _request, response):
        if self.route_goal_handle is None:
            response.success = False
            response.message = '当前没有由航点编辑器启动的路线'
        else:
            self.route_goal_handle.cancel_goal_async()
            response.success = True
            response.message = '已请求取消路线'
        return response

    def clear_service(self, _request, response):
        self.poses.clear()
        self.server.clear()
        self.server.applyChanges()
        return response

    def save_service(self, _request, response):
        try:
            self.save()
            response.success = True
            response.message = f'已保存到 {self.waypoints_file}'
        except OSError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def reload_service(self, _request, response):
        try:
            self.load()
            response.success = True
            response.message = f'已从 {self.waypoints_file} 重新加载'
        except (OSError, ValueError, yaml.YAMLError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def save(self):
        data = {'frame_id': self.frame_id, 'waypoints': []}
        for name in self.ordered_names():
            pose = self.poses[name]
            yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
            data['waypoints'].append({
                'name': name,
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': yaw,
            })
        self.waypoints_file.parent.mkdir(parents=True, exist_ok=True)
        with self.waypoints_file.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)

    def load(self):
        if not self.waypoints_file.is_file():
            return
        with self.waypoints_file.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if data.get('frame_id', self.frame_id) != self.frame_id:
            raise ValueError('航点文件 frame_id 与节点参数不一致')
        self.poses.clear()
        self.server.clear()
        highest = 0
        for item in data.get('waypoints', []):
            pose = Pose()
            pose.position.x = float(item['x'])
            pose.position.y = float(item['y'])
            yaw = float(item.get('yaw', 0.0))
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            name = str(item['name'])
            self.insert_marker(name, pose)
            suffix = name.rsplit('_', 1)[-1]
            highest = max(highest, int(suffix) if suffix.isdigit() else 0)
        self.next_index = highest + 1
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointEditor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
