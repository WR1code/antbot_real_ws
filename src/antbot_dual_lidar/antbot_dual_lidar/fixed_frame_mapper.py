"""Accumulate two filtered clouds in odom and publish the odometry trajectory."""

from functools import partial

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .filter_core import transform_xyz
from .map_core import BoundedVoxelMap


class FixedFrameMapper(Node):
    def __init__(self):
        super().__init__("antbot_fixed_frame_mapper")
        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter(
            "front_topic", "/antbot/lidar/front_left/points_filtered"
        )
        self.declare_parameter(
            "rear_topic", "/antbot/lidar/rear_right/points_filtered"
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/antbot/lidar/map_points")
        self.declare_parameter("trajectory_topic", "/antbot/trajectory")
        self.declare_parameter("voxel_leaf_size", 0.05)
        self.declare_parameter("max_map_points", 300000)
        self.declare_parameter("publish_frequency", 2.0)
        self.declare_parameter("trajectory_min_distance", 0.05)
        self.declare_parameter("trajectory_max_poses", 10000)
        self.declare_parameter("tf_timeout_sec", 0.10)

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        leaf = float(self.get_parameter("voxel_leaf_size").value)
        max_points = int(self.get_parameter("max_map_points").value)
        frequency = float(self.get_parameter("publish_frequency").value)
        self.trajectory_min_distance = float(
            self.get_parameter("trajectory_min_distance").value
        )
        self.trajectory_max_poses = int(
            self.get_parameter("trajectory_max_poses").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        if (
            not self.fixed_frame
            or frequency <= 0.0
            or self.trajectory_min_distance < 0.0
            or self.trajectory_max_poses <= 0
            or self.tf_timeout <= 0.0
        ):
            raise ValueError("invalid fixed-frame mapper parameters")

        self.voxels = BoundedVoxelMap(leaf, max_points)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("map_topic").value), latched
        )
        self.path_publisher = self.create_publisher(
            Path, str(self.get_parameter("trajectory_topic").value), latched
        )
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame
        self.last_cloud_stamp_ns = None
        self.last_path_xyz = None
        self.received = {"front": 0, "rear": 0}

        for name, parameter in (
            ("front", "front_topic"),
            ("rear", "rear_topic"),
        ):
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter(parameter).value),
                partial(self._cloud, name),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom,
            50,
        )
        self.create_timer(1.0 / frequency, self._publish_map)
        self.get_logger().info(
            f"accumulating front+rear clouds in {self.fixed_frame}; "
            "truth_deskew/world_reference are not subscribed"
        )

    def _reset(self, reason):
        self.voxels.clear()
        self.path.poses.clear()
        self.last_path_xyz = None
        self.get_logger().warning(f"map and trajectory reset: {reason}")

    def _cloud(self, source, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if (
            self.last_cloud_stamp_ns is not None
            and stamp_ns + 1_000_000_000 < self.last_cloud_stamp_ns
        ):
            self._reset("simulation time moved backwards")
        self.last_cloud_stamp_ns = stamp_ns
        self.received[source] += 1
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            ).transform
            names = {field.name for field in msg.fields}
            field_names = ["x", "y", "z"]
            if "intensity" in names:
                field_names.append("intensity")
            structured = point_cloud2.read_points(
                msg, field_names=field_names, skip_nans=True
            )
            xyz = np.column_stack(
                (structured["x"], structured["y"], structured["z"])
            )
            q = transform.rotation
            t = transform.translation
            xyz = transform_xyz(
                xyz, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
            )
            if "intensity" in structured.dtype.names:
                values = np.column_stack(
                    (xyz, structured["intensity"])
                ).astype(np.float32)
            else:
                values = xyz
            self.voxels.update(values)
        except (TransformException, ValueError, AssertionError) as error:
            self.get_logger().warning(
                f"{source} cloud skipped: {error}", throttle_duration_sec=2.0
            )

    def _odom(self, msg):
        pose = msg.pose.pose
        xyz = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        )
        if (
            self.last_path_xyz is not None
            and np.linalg.norm(xyz - self.last_path_xyz)
            < self.trajectory_min_distance
        ):
            return
        stamped = PoseStamped()
        stamped.header = msg.header
        stamped.header.frame_id = self.fixed_frame
        stamped.pose = pose
        self.path.poses.append(stamped)
        if len(self.path.poses) > self.trajectory_max_poses:
            del self.path.poses[: len(self.path.poses) - self.trajectory_max_poses]
        self.last_path_xyz = xyz
        self.path.header.stamp = msg.header.stamp
        self.path_publisher.publish(self.path)

    def _publish_map(self):
        xyz = self.voxels.points()
        if not len(xyz):
            return
        header = Header()
        header.frame_id = self.fixed_frame
        header.stamp = self.get_clock().now().to_msg()
        if xyz.shape[1] == 4:
            fields = [
                PointField(
                    name="x", offset=0,
                    datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="y", offset=4,
                    datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="z", offset=8,
                    datatype=PointField.FLOAT32, count=1),
                PointField(
                    name="intensity", offset=12,
                    datatype=PointField.FLOAT32, count=1),
            ]
            cloud = point_cloud2.create_cloud(header, fields, xyz)
        else:
            cloud = point_cloud2.create_cloud_xyz32(header, xyz)
        self.map_publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = FixedFrameMapper()
        # Point-cloud voxelization is deliberately serialized by the node's
        # default mutually-exclusive callback group.  Use a second executor
        # thread so TransformListener's reentrant callback group can continue
        # consuming /tf while a large cloud is being voxelized.  A
        # SingleThreadedExecutor lets the TF queue fall behind the 10 Hz lidar
        # stream, making every exact-time lookup fail as "future" data.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
