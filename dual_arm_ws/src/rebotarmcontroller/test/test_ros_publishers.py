from builtin_interfaces.msg import Time

from rebotarmcontroller.ros_publishers import JointStatePublisher


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    class Now:
        @staticmethod
        def to_msg():
            return Time()

    @staticmethod
    def now():
        return FakeClock.Now()


class FakeNode:
    sensor_qos = object()
    reentrant_group = object()

    def __init__(self):
        self.publishers = {}

    def create_publisher(self, _message_type, topic, _qos, **_kwargs):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    @staticmethod
    def create_timer(_period, callback, **_kwargs):
        return callback

    @staticmethod
    def get_clock():
        return FakeClock()


class FakeHardware:
    joint_names = [f"joint{index}" for index in range(1, 7)]
    has_gripper = False
    mode = "posvel"
    enabled = True
    control_loop_active = True
    state_machine = "IDLE"
    error_codes = []

    status_reads = 0

    @staticmethod
    def get_joint_state():
        return [0.0] * 6, [0.0] * 6, [0.0] * 6

    def get_joint_status_codes(self):
        self.status_reads += 1
        return [0] * 6


def test_arm_status_is_refreshed_from_successful_joint_feedback():
    node = FakeNode()
    hardware = FakeHardware()
    publisher = JointStatePublisher(node, hardware, "rebotarm", 100.0)
    status_publisher = node.publishers["/rebotarm/arm_status"]
    assert len(status_publisher.messages) == 1

    publisher._last_status_publish = 0.0
    publisher.publish()

    assert len(status_publisher.messages) == 2
    assert status_publisher.messages[-1].state_machine == "IDLE"
    assert status_publisher.messages[-1].enabled
    # One read for initial status and one shared by joint + periodic status.
    assert hardware.status_reads == 2
