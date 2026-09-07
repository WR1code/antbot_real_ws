#!/usr/bin/env python3
"""Named speed-limit drawing shapes backed by a Nav2 SpeedFilter mask."""

import json
import math
import os
from pathlib import Path
import tempfile

from geometry_msgs.msg import Point
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robotcar_navigation.msg import SpeedZone
from robotcar_navigation.srv import GetWaypointNames, RenameWaypoint, WaypointFile
from std_msgs.msg import String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray


TRANSIENT_QOS = QoSProfile(depth=1)
TRANSIENT_QOS.reliability = ReliabilityPolicy.RELIABLE
TRANSIENT_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL

SUPPORTED_SHAPES = {"rectangle", "square", "ellipse", "circle"}
SHAPE_LABELS = {
    "rectangle": "矩形",
    "square": "正方形",
    "ellipse": "椭圆",
    "circle": "圆形",
}


def normalized_shape(value):
    shape = str(value or "rectangle").strip().lower()
    if shape not in SUPPORTED_SHAPES:
        raise ValueError(f"不支持的限速区形状：{shape}")
    return shape


def normalized_dimensions(shape, width, height):
    if shape in {"square", "circle"}:
        side = max(width, height)
        return side, side
    return width, height


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class SpeedZoneManager(Node):
    """Store speed zones and turn them into an absolute m/s costmap mask."""

    def __init__(self):
        super().__init__("speed_zone_manager")
        self.declare_parameter("speed_zone_file", "")
        self.active_file = str(self.get_parameter("speed_zone_file").value)
        self.zones = []
        self.map = None
        self.enabled = True

        self.mask_pub = self.create_publisher(
            OccupancyGrid, "/speed_filter_mask", TRANSIENT_QOS
        )
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, "/speed_costmap_filter_info", TRANSIENT_QOS
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/waterplus/speed_zone_markers", TRANSIENT_QOS
        )
        self.status_pub = self.create_publisher(
            String, "/waterplus/speed_zone_status", 10
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, TRANSIENT_QOS)
        self.create_subscription(SpeedZone, "/waterplus/add_speed_zone", self.add_zone, 10)
        self.create_subscription(String, "/waterplus/delete_speed_zone", self.delete_zone, 10)
        self.create_service(
            GetWaypointNames, "/waterplus/get_speed_zone_names", self.get_names
        )
        self.create_service(
            RenameWaypoint, "/waterplus/rename_speed_zone", self.rename_zone
        )
        self.create_service(
            WaypointFile, "/waterplus/save_speed_zone_group", self.save_group
        )
        self.create_service(
            WaypointFile, "/waterplus/load_speed_zone_group", self.load_group
        )
        self.create_service(
            SetBool, "/waterplus/enable_speed_zones", self.set_enabled
        )
        if self.active_file and Path(self.active_file).is_file():
            self.load_file(self.active_file)
        self.publish_info()

    def publish_status(self, success, message):
        payload = {"success": success, "message": message}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def map_callback(self, message):
        changed = (
            self.map is None
            or self.map.info.width != message.info.width
            or self.map.info.height != message.info.height
            or self.map.info.resolution != message.info.resolution
            or self.map.info.origin != message.info.origin
        )
        self.map = message
        if changed:
            self.publish_all()

    def unique_name(self, requested):
        base = requested.strip() or "低速区"
        names = {zone["name"] for zone in self.zones}
        if base not in names:
            return base
        index = 2
        while f"{base}_{index}" in names:
            index += 1
        return f"{base}_{index}"

    def add_zone(self, message):
        if message.width <= 0.0 or message.height <= 0.0:
            self.publish_status(False, "限速区宽度和高度必须大于零")
            return
        if not 0.01 <= message.max_speed <= 1.0:
            self.publish_status(False, "最大速度必须在 0.01 到 1.00 m/s 之间")
            return
        if message.frame_id and message.frame_id != "map":
            self.publish_status(False, "限速区必须在 map 坐标系中添加")
            return
        try:
            shape = normalized_shape(message.shape)
        except ValueError as error:
            self.publish_status(False, str(error))
            return
        width, height = normalized_dimensions(
            shape, float(message.width), float(message.height)
        )
        zone = {
            "name": self.unique_name(message.name),
            "x": float(message.pose.position.x),
            "y": float(message.pose.position.y),
            "yaw": yaw_from_quaternion(message.pose.orientation),
            "width": width,
            "height": height,
            "shape": shape,
            "max_speed": float(message.max_speed),
            "enabled": bool(message.enabled),
        }
        self.zones.append(zone)
        self.publish_all()
        self.publish_status(
            True, f"已添加限速区：{zone['name']}（{zone['max_speed']:.2f} m/s）"
        )

    def delete_zone(self, message):
        old_count = len(self.zones)
        self.zones = [zone for zone in self.zones if zone["name"] != message.data]
        success = len(self.zones) != old_count
        self.publish_all()
        detail = (
            f"已删除限速区：{message.data}"
            if success else f"找不到限速区：{message.data}"
        )
        self.publish_status(success, detail)

    def get_names(self, _request, response):
        response.names = [zone["name"] for zone in self.zones]
        response.active_file = self.active_file
        return response

    def rename_zone(self, request, response):
        old_name = request.old_name.strip()
        new_name = request.new_name.strip()
        if not old_name or not new_name:
            response.success = False
            response.message = "原名称和新名称都不能为空"
        elif old_name != new_name and any(
            zone["name"] == new_name for zone in self.zones
        ):
            response.success = False
            response.message = f"限速区名称已存在：{new_name}"
        else:
            zone = next(
                (item for item in self.zones if item["name"] == old_name), None
            )
            if zone is None:
                response.success = False
                response.message = f"找不到限速区：{old_name}"
            elif old_name == new_name:
                response.success = True
                response.message = "限速区名称未改变"
            else:
                zone["name"] = new_name
                response.success = True
                response.message = f"限速区已重命名：{old_name} → {new_name}"
                self.publish_all()
        self.publish_status(response.success, response.message)
        return response

    def set_enabled(self, request, response):
        self.enabled = bool(request.data)
        self.publish_all()
        response.success = True
        response.message = "限速区已启用" if self.enabled else "限速区已暂时关闭"
        self.publish_status(True, response.message)
        return response

    def save_file(self, filename):
        path = Path(filename).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(
                    {"version": 2, "enabled": self.enabled, "zones": self.zones},
                    stream, ensure_ascii=False, indent=2,
                )
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
        except Exception:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)
            raise
        self.active_file = str(path)

    def load_file(self, filename):
        path = Path(filename).expanduser().resolve()
        with path.open(encoding="utf-8") as stream:
            document = json.load(stream)
        raw_zones = document.get("zones", [])
        if not isinstance(raw_zones, list):
            raise ValueError("zones 必须是数组")
        validated = []
        names = set()
        for raw in raw_zones:
            name = str(raw.get("name", "")).strip()
            width = float(raw.get("width", 0.0))
            height = float(raw.get("height", 0.0))
            shape = normalized_shape(raw.get("shape", "rectangle"))
            max_speed = float(raw.get("max_speed", 0.0))
            if (
                not name or name in names or width <= 0.0 or height <= 0.0
                or not 0.01 <= max_speed <= 1.0
            ):
                raise ValueError("限速区包含空名称、重复名称、无效尺寸或速度")
            width, height = normalized_dimensions(shape, width, height)
            names.add(name)
            validated.append({
                "name": name,
                "x": float(raw.get("x", 0.0)),
                "y": float(raw.get("y", 0.0)),
                "yaw": float(raw.get("yaw", 0.0)),
                "width": width,
                "height": height,
                "shape": shape,
                "max_speed": max_speed,
                "enabled": bool(raw.get("enabled", True)),
            })
        self.zones = validated
        self.enabled = bool(document.get("enabled", True))
        self.active_file = str(path)
        self.publish_all()

    def save_group(self, request, response):
        try:
            if not request.filename:
                raise ValueError("限速区组文件名不能为空")
            self.save_file(request.filename)
            response.success = True
            response.message = f"限速区组已保存：{self.active_file}"
        except Exception as error:
            response.success = False
            response.message = f"保存限速区组失败：{error}"
        self.publish_status(response.success, response.message)
        return response

    def load_group(self, request, response):
        try:
            self.load_file(request.filename)
            response.success = True
            response.message = f"已打开限速区组：{self.active_file}"
        except Exception as error:
            response.success = False
            response.message = f"打开限速区组失败：{error}"
        self.publish_status(response.success, response.message)
        return response

    def publish_info(self):
        info = CostmapFilterInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "map"
        info.type = 2
        info.filter_mask_topic = "/speed_filter_mask"
        info.base = 0.0
        info.multiplier = 0.01
        self.info_pub.publish(info)

    def world_to_grid(self, x, y):
        origin = self.map.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return (
            (cosine * dx + sine * dy) / self.map.info.resolution,
            (-sine * dx + cosine * dy) / self.map.info.resolution,
        )

    def grid_to_world(self, gx, gy):
        origin = self.map.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        local_x = gx * self.map.info.resolution
        local_y = gy * self.map.info.resolution
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return (
            origin.position.x + cosine * local_x - sine * local_y,
            origin.position.y + sine * local_x + cosine * local_y,
        )

    def rasterize_zone(self, data, zone):
        half_width = zone["width"] * 0.5
        half_height = zone["height"] * 0.5
        cosine, sine = math.cos(zone["yaw"]), math.sin(zone["yaw"])
        corners = []
        for local_x, local_y in (
            (-half_width, -half_height), (-half_width, half_height),
            (half_width, -half_height), (half_width, half_height),
        ):
            world_x = zone["x"] + cosine * local_x - sine * local_y
            world_y = zone["y"] + sine * local_x + cosine * local_y
            corners.append(self.world_to_grid(world_x, world_y))
        min_x = max(0, int(math.floor(min(point[0] for point in corners))) - 1)
        max_x = min(
            self.map.info.width - 1,
            int(math.ceil(max(point[0] for point in corners))) + 1,
        )
        min_y = max(0, int(math.floor(min(point[1] for point in corners))) - 1)
        max_y = min(
            self.map.info.height - 1,
            int(math.ceil(max(point[1] for point in corners))) + 1,
        )
        encoded_speed = max(1, min(100, int(round(zone["max_speed"] * 100.0))))
        for grid_y in range(min_y, max_y + 1):
            for grid_x in range(min_x, max_x + 1):
                world_x, world_y = self.grid_to_world(grid_x + 0.5, grid_y + 0.5)
                dx, dy = world_x - zone["x"], world_y - zone["y"]
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                inside = abs(local_x) <= half_width and abs(local_y) <= half_height
                if zone.get("shape", "rectangle") in {"ellipse", "circle"}:
                    inside = (
                        (local_x / half_width) ** 2
                        + (local_y / half_height) ** 2
                        <= 1.0
                    )
                if inside:
                    index = grid_y * self.map.info.width + grid_x
                    if data[index] == 0 or encoded_speed < data[index]:
                        data[index] = encoded_speed

    def publish_mask(self):
        if self.map is None:
            return
        mask = OccupancyGrid()
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = self.map.header.frame_id or "map"
        mask.info = self.map.info
        data = [0] * (mask.info.width * mask.info.height)
        if self.enabled:
            for zone in self.zones:
                if zone["enabled"]:
                    self.rasterize_zone(data, zone)
        mask.data = data
        self.mask_pub.publish(mask)

    def publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, zone in enumerate(self.zones):
            area = Marker()
            area.header = clear.header
            area.ns = "speed_zones"
            area.id = index * 3
            shape = zone.get("shape", "rectangle")
            area.type = (
                Marker.CYLINDER if shape in {"ellipse", "circle"} else Marker.CUBE
            )
            area.action = Marker.ADD
            area.pose.position.x = zone["x"]
            area.pose.position.y = zone["y"]
            area.pose.position.z = 0.035
            area.pose.orientation.z = math.sin(zone["yaw"] * 0.5)
            area.pose.orientation.w = math.cos(zone["yaw"] * 0.5)
            area.scale.x = zone["width"]
            area.scale.y = zone["height"]
            area.scale.z = 0.06
            area.color.r = 0.12
            area.color.g = 0.55
            area.color.b = 1.0
            area.color.a = 0.34 if self.enabled and zone["enabled"] else 0.13
            markers.markers.append(area)
            outline = Marker()
            outline.header = clear.header
            outline.ns = "speed_zone_boundaries"
            outline.id = index * 3 + 1
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose = area.pose
            outline.scale.x = 0.045
            half_width = zone["width"] * 0.5
            half_height = zone["height"] * 0.5
            if shape in {"ellipse", "circle"}:
                boundary = [
                    (
                        half_width * math.cos(2.0 * math.pi * step / 64),
                        half_height * math.sin(2.0 * math.pi * step / 64),
                    )
                    for step in range(65)
                ]
            else:
                boundary = (
                    (-half_width, -half_height), (half_width, -half_height),
                    (half_width, half_height), (-half_width, half_height),
                    (-half_width, -half_height),
                )
            for x, y in boundary:
                point = Point()
                point.x, point.y = x, y
                outline.points.append(point)
            outline.color.r = 0.10
            outline.color.g = 0.65
            outline.color.b = 1.0
            outline.color.a = 0.95
            markers.markers.append(outline)
            label = Marker()
            label.header = clear.header
            label.ns = "speed_zone_labels"
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = zone["x"]
            label.pose.position.y = zone["y"]
            label.pose.position.z = 0.28
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r = 0.25
            label.color.g = 0.75
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = (
                f"限速：{zone['name']}  {SHAPE_LABELS[shape]}  {zone['width']:.2f} × "
                f"{zone['height']:.2f} m  ≤ {zone['max_speed']:.2f} m/s"
            )
            markers.markers.append(label)
        self.marker_pub.publish(markers)

    def publish_all(self):
        self.publish_info()
        self.publish_mask()
        self.publish_markers()


def main():
    rclpy.init()
    node = SpeedZoneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
