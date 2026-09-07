#!/usr/bin/env python3
"""Named keepout drawing shapes backed by a live Nav2 keepout mask."""

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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotcar_navigation.msg import KeepoutZone
from robotcar_navigation.srv import GetWaypointNames, RenameWaypoint, WaypointFile
from std_msgs.msg import String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

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
        raise ValueError(f"不支持的禁区形状：{shape}")
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


class KeepoutZoneManager(Node):
    def __init__(self):
        super().__init__("keepout_zone_manager")
        self.declare_parameter("keepout_file", "")
        self.zones = []
        self.map = None
        self.enabled = True
        self.active_file = str(self.get_parameter("keepout_file").value)

        self.mask_pub = self.create_publisher(
            OccupancyGrid, "/keepout_filter_mask", TRANSIENT_QOS
        )
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, "/costmap_filter_info", TRANSIENT_QOS
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/waterplus/keepout_markers", TRANSIENT_QOS
        )
        self.status_pub = self.create_publisher(
            String, "/waterplus/keepout_status", 10
        )
        self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, TRANSIENT_QOS
        )
        self.create_subscription(
            KeepoutZone, "/waterplus/add_keepout", self.add_zone, 10
        )
        self.create_subscription(
            String, "/waterplus/delete_keepout", self.delete_zone, 10
        )
        self.create_service(
            GetWaypointNames, "/waterplus/get_keepout_names", self.get_names
        )
        self.create_service(
            RenameWaypoint, "/waterplus/rename_keepout", self.rename_zone
        )
        self.create_service(
            WaypointFile, "/waterplus/save_keepout_group", self.save_group
        )
        self.create_service(
            WaypointFile, "/waterplus/load_keepout_group", self.load_group
        )
        self.create_service(
            SetBool, "/waterplus/enable_keepouts", self.set_enabled
        )

        if self.active_file and Path(self.active_file).is_file():
            self.load_file(self.active_file)
        self.publish_info()

    def publish_status(self, success, message):
        self.status_pub.publish(
            String(data=json.dumps({"success": success, "message": message}, ensure_ascii=False))
        )

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
        base = requested.strip() or "临时禁区"
        existing = {zone["name"] for zone in self.zones}
        if base not in existing:
            return base
        index = 2
        while f"{base}_{index}" in existing:
            index += 1
        return f"{base}_{index}"

    def add_zone(self, message):
        if message.width <= 0.0 or message.height <= 0.0:
            self.publish_status(False, "禁区宽度和高度必须大于零")
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
            "frame_id": message.frame_id or "map",
            "x": float(message.pose.position.x),
            "y": float(message.pose.position.y),
            "yaw": yaw_from_quaternion(message.pose.orientation),
            "width": width,
            "height": height,
            "shape": shape,
            "enabled": bool(message.enabled),
        }
        if zone["frame_id"] != "map":
            self.publish_status(False, "临时禁区必须在 map 坐标系中添加")
            return
        self.zones.append(zone)
        self.publish_all()
        self.publish_status(True, f"已添加临时禁区：{zone['name']}")

    def delete_zone(self, message):
        old_count = len(self.zones)
        self.zones = [zone for zone in self.zones if zone["name"] != message.data]
        success = len(self.zones) != old_count
        self.publish_all()
        self.publish_status(
            success,
            f"已删除临时禁区：{message.data}" if success else f"找不到临时禁区：{message.data}",
        )

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
            response.message = f"禁区名称已存在：{new_name}"
        else:
            zone = next(
                (item for item in self.zones if item["name"] == old_name), None
            )
            if zone is None:
                response.success = False
                response.message = f"找不到临时禁区：{old_name}"
            elif old_name == new_name:
                response.success = True
                response.message = "临时禁区名称未改变"
            else:
                zone["name"] = new_name
                response.success = True
                response.message = f"临时禁区已重命名：{old_name} → {new_name}"
                self.publish_all()
        self.publish_status(response.success, response.message)
        return response

    def set_enabled(self, request, response):
        self.enabled = bool(request.data)
        self.publish_all()
        response.success = True
        response.message = "临时禁区已启用" if self.enabled else "临时禁区已暂时关闭"
        self.publish_status(True, response.message)
        return response

    def serialized(self):
        return {"version": 2, "enabled": self.enabled, "zones": self.zones}

    def save_file(self, filename):
        path = Path(filename).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(self.serialized(), stream, ensure_ascii=False, indent=2)
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
        zones = document.get("zones", [])
        if not isinstance(zones, list):
            raise ValueError("zones 必须是数组")
        validated = []
        names = set()
        for raw in zones:
            name = str(raw.get("name", "")).strip()
            width = float(raw.get("width", 0.0))
            height = float(raw.get("height", 0.0))
            shape = normalized_shape(raw.get("shape", "rectangle"))
            if not name or name in names or width <= 0.0 or height <= 0.0:
                raise ValueError("禁区包含空名称、重复名称或无效尺寸")
            width, height = normalized_dimensions(shape, width, height)
            names.add(name)
            validated.append(
                {
                    "name": name,
                    "frame_id": "map",
                    "x": float(raw.get("x", 0.0)),
                    "y": float(raw.get("y", 0.0)),
                    "yaw": float(raw.get("yaw", 0.0)),
                    "width": width,
                    "height": height,
                    "shape": shape,
                    "enabled": bool(raw.get("enabled", True)),
                }
            )
        self.zones = validated
        self.enabled = bool(document.get("enabled", True))
        self.active_file = str(path)
        self.publish_all()

    def save_group(self, request, response):
        try:
            if not request.filename:
                raise ValueError("禁区组文件名不能为空")
            self.save_file(request.filename)
            response.success = True
            response.message = f"禁区组已保存：{self.active_file}"
        except Exception as error:
            response.success = False
            response.message = f"保存禁区组失败：{error}"
        self.publish_status(response.success, response.message)
        return response

    def load_group(self, request, response):
        try:
            self.load_file(request.filename)
            response.success = True
            response.message = f"已打开禁区组：{self.active_file}"
        except Exception as error:
            response.success = False
            response.message = f"打开禁区组失败：{error}"
        self.publish_status(response.success, response.message)
        return response

    def publish_info(self):
        info = CostmapFilterInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = "map"
        info.type = 0
        info.filter_mask_topic = "/keepout_filter_mask"
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)

    def world_to_grid(self, x, y):
        origin = self.map.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        gx = (cosine * dx + sine * dy) / self.map.info.resolution
        gy = (-sine * dx + cosine * dy) / self.map.info.resolution
        return gx, gy

    def grid_to_world(self, gx, gy):
        origin = self.map.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        local_x = gx * self.map.info.resolution
        local_y = gy * self.map.info.resolution
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            origin.position.x + cosine * local_x - sine * local_y,
            origin.position.y + sine * local_x + cosine * local_y,
        )

    def rasterize_zone(self, data, zone):
        half_width = zone["width"] * 0.5
        half_height = zone["height"] * 0.5
        cosine = math.cos(zone["yaw"])
        sine = math.sin(zone["yaw"])
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
        for grid_y in range(min_y, max_y + 1):
            for grid_x in range(min_x, max_x + 1):
                world_x, world_y = self.grid_to_world(grid_x + 0.5, grid_y + 0.5)
                dx = world_x - zone["x"]
                dy = world_y - zone["y"]
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
                    data[grid_y * self.map.info.width + grid_x] = 100

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
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = clear.header.stamp
            marker.ns = "keepout_zones"
            marker.id = index * 3
            shape = zone.get("shape", "rectangle")
            marker.type = (
                Marker.CYLINDER if shape in {"ellipse", "circle"} else Marker.CUBE
            )
            marker.action = Marker.ADD
            marker.pose.position.x = zone["x"]
            marker.pose.position.y = zone["y"]
            marker.pose.position.z = 0.025
            marker.pose.orientation.z = math.sin(zone["yaw"] * 0.5)
            marker.pose.orientation.w = math.cos(zone["yaw"] * 0.5)
            marker.scale.x = zone["width"]
            marker.scale.y = zone["height"]
            marker.scale.z = 0.05
            marker.color.r = 0.95
            marker.color.g = 0.12 if self.enabled and zone["enabled"] else 0.55
            marker.color.b = 0.12
            marker.color.a = 0.42 if self.enabled and zone["enabled"] else 0.18
            markers.markers.append(marker)
            outline = Marker()
            outline.header = marker.header
            outline.ns = "keepout_boundaries"
            outline.id = index * 3 + 1
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose = marker.pose
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
            outline.color.r = 1.0
            outline.color.g = 0.12
            outline.color.b = 0.08
            outline.color.a = 0.95
            markers.markers.append(outline)
            label = Marker()
            label.header = marker.header
            label.ns = "keepout_labels"
            label.id = index * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = zone["x"]
            label.pose.position.y = zone["y"]
            label.pose.position.z = 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.20
            label.color.r = 1.0
            label.color.g = 0.35
            label.color.b = 0.25
            label.color.a = 1.0
            label.text = (
                f"禁区：{zone['name']}  {SHAPE_LABELS[shape]}  "
                f"{zone['width']:.2f} × {zone['height']:.2f} m"
            )
            markers.markers.append(label)
        self.marker_pub.publish(markers)

    def publish_all(self):
        self.publish_info()
        self.publish_mask()
        self.publish_markers()


def main():
    rclpy.init()
    node = KeepoutZoneManager()
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
