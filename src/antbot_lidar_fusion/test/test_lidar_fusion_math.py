import math
from types import SimpleNamespace

from sensor_msgs.msg import LaserScan

from antbot_lidar_fusion.lidar_fusion_node import scan_xy, transform_xy


def test_scan_xy_filters_invalid_ranges():
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 2
    scan.range_min = 0.1
    scan.range_max = 5.0
    scan.ranges = [1.0, float("inf"), float("nan"), 0.05, 2.0, 6.0]
    points = scan_xy(scan)
    assert len(points) == 2
    assert points[0] == (1.0, 0.0, 0.0)
    assert math.isclose(points[1][0], 2.0)
    assert math.isclose(points[1][1], 0.0, abs_tol=1e-12)


def test_transform_xy_yaw_and_translation():
    half = math.sqrt(0.5)
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=half, w=half),
    )
    point = transform_xy([(1.0, 0.0, 0.0)], transform)[0]
    assert math.isclose(point[0], 1.0, abs_tol=1e-12)
    assert math.isclose(point[1], 3.0, abs_tol=1e-12)
