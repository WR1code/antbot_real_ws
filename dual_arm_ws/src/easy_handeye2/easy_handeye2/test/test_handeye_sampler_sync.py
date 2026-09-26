from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

from easy_handeye2.handeye_sampler import HandeyeSampler


class _Buffer:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, target, source, when, timeout):
        self.calls.append((target, source, when))
        result = TransformStamped()
        if len(self.calls) == 1:
            result.header.stamp.sec = 123
            result.header.stamp.nanosec = 456
        return result


def _sampler(calibration_type="eye_in_hand"):
    sampler = object.__new__(HandeyeSampler)
    sampler.node = SimpleNamespace()
    sampler.tfBuffer = _Buffer()
    sampler.handeye_parameters = SimpleNamespace(
        calibration_type=calibration_type,
        robot_base_frame="base",
        robot_effector_frame="tool",
        tracking_base_frame="camera",
        tracking_marker_frame="marker",
    )
    return sampler


def test_default_sample_uses_latest_tracking_stamp_for_robot_tf():
    sampler = _sampler()

    sample = sampler._get_transforms()

    assert sample is not None
    assert sampler.tfBuffer.calls[0][:2] == ("camera", "marker")
    assert sampler.tfBuffer.calls[0][2].nanoseconds == 0
    assert sampler.tfBuffer.calls[1][:2] == ("base", "tool")
    assert sampler.tfBuffer.calls[1][2].nanoseconds == 123_000_000_456


def test_explicit_sample_time_is_used_for_both_transforms():
    sampler = _sampler()
    requested = Time(nanoseconds=9_876_543_210)

    sample = sampler._get_transforms(requested)

    assert sample is not None
    assert [call[2].nanoseconds for call in sampler.tfBuffer.calls] == [
        requested.nanoseconds,
        requested.nanoseconds,
    ]
