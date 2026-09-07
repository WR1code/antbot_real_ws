"""Project a horizontal slice of an XYZ cloud into a planar laser scan."""

import math

import numpy as np


def project_xyz_to_laserscan(
    points,
    *,
    min_height=-0.20,
    max_height=0.20,
    min_range=0.10,
    max_range=20.0,
    angle_increment=math.radians(0.1),
    sensor_translation=(0.0, 0.0, 0.0),
    sensor_yaw=0.0,
    body_bounds=None,
):
    """Return a full-circle scan, optionally rejecting points inside the robot.

    ``points`` are in the lidar frame.  ``sensor_translation`` and
    ``sensor_yaw`` describe that frame in ``base_link`` so the exclusion box
    remains correct for both the front and the 180-degree rear lidar.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if (
        not min_height < max_height
        or min_range < 0.0
        or not min_range < max_range
        or angle_increment <= 0.0
        or not np.isfinite(sensor_yaw)
    ):
        raise ValueError("invalid point-cloud laser projection limits")
    translation = np.asarray(sensor_translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("sensor_translation must contain three finite values")
    bounds = None
    if body_bounds is not None:
        bounds = np.asarray(body_bounds, dtype=np.float64)
        if (
            bounds.shape != (6,)
            or not np.isfinite(bounds).all()
            or not (
                bounds[0] < bounds[1]
                and bounds[2] < bounds[3]
                and bounds[4] < bounds[5]
            )
        ):
            raise ValueError("body_bounds must be finite min/max XYZ limits")

    angle_min = -math.pi
    bin_count = int(round(2.0 * math.pi / angle_increment))
    angle_increment = 2.0 * math.pi / bin_count
    angle_max = angle_min + angle_increment * (bin_count - 1)
    ranges = np.full(bin_count, np.inf, dtype=np.float32)
    if not len(xyz):
        return angle_min, angle_max, ranges

    horizontal_range = np.hypot(xyz[:, 0], xyz[:, 1])
    angles = np.arctan2(xyz[:, 1], xyz[:, 0])
    valid = (
        np.isfinite(xyz).all(axis=1)
        & (xyz[:, 2] >= min_height)
        & (xyz[:, 2] <= max_height)
        & (horizontal_range >= min_range)
        & (horizontal_range <= max_range)
    )
    if bounds is not None:
        cosine = math.cos(sensor_yaw)
        sine = math.sin(sensor_yaw)
        base_x = translation[0] + cosine * xyz[:, 0] - sine * xyz[:, 1]
        base_y = translation[1] + sine * xyz[:, 0] + cosine * xyz[:, 1]
        base_z = translation[2] + xyz[:, 2]
        inside_body = (
            (base_x >= bounds[0])
            & (base_x <= bounds[1])
            & (base_y >= bounds[2])
            & (base_y <= bounds[3])
            & (base_z >= bounds[4])
            & (base_z <= bounds[5])
        )
        valid &= ~inside_body
    indices = np.floor(
        (angles[valid] - angle_min) / angle_increment
    ).astype(np.int64)
    indices %= bin_count
    np.minimum.at(ranges, indices, horizontal_range[valid])
    return angle_min, angle_max, ranges
