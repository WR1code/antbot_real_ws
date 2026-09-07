"""ROS PointCloud2 construction for RGB uint8 point arrays."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def colored_cloud_message(points, colors, frame_id, stamp):
    """Build an XYZ + packed-RGB PointCloud2 message."""
    points = np.asarray(points, dtype=np.float32)
    colors = np.asarray(colors, dtype=np.uint8)
    if points.ndim != 2 or points.shape[1] != 3 or colors.shape != points.shape:
        raise ValueError("colored point cloud requires matching Nx3 arrays")
    cloud = PointCloud2()
    cloud.header.frame_id = str(frame_id)
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    packed = np.empty(
        len(points),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")],
    )
    if len(points):
        packed["x"], packed["y"], packed["z"] = points.T
        colors_u32 = colors.astype(np.uint32)
        packed["rgb"] = (
            (colors_u32[:, 0] << 16) | (colors_u32[:, 1] << 8) | colors_u32[:, 2]
        )
    cloud.is_bigendian = False
    cloud.point_step = packed.dtype.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = packed.tobytes()
    cloud.is_dense = bool(len(points)) and bool(np.isfinite(points).all())
    return cloud
