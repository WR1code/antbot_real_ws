"""Rosbag source contract.

The actual storage reader is deliberately not coupled to rosbag2_py here. A future
adapter feeds decoded color/depth/CameraInfo/pose messages through the same
ApproximateRGBDSynchronizer and validation pipeline used by the live node.
"""


class RosbagRGBDSource:
    def __init__(self, bag_path, configuration):
        self.bag_path = bag_path
        self.configuration = configuration

    def start(self) -> None:
        raise NotImplementedError(
            "rosbag2 storage adapter is not implemented yet; use ROS 2 bag play "
            "with phase4b_capture_node to exercise the production sync pipeline"
        )

    def read(self):
        return None

    def stop(self) -> None:
        return None
