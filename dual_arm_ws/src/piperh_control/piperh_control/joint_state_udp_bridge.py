"""Forward six-axis Piper-H joint states to the Isaac Sim UDP receiver."""

from __future__ import annotations

import json
import socket
import time
from collections.abc import Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))


def ordered_positions(
    names: Sequence[str], positions: Sequence[float], required: Sequence[str]
) -> list[float] | None:
    """Return positions in required order, rejecting malformed messages."""
    if len(names) != len(positions) or len(set(names)) != len(names):
        return None
    values = dict(zip(names, positions))
    if not all(name in values for name in required):
        return None
    return [float(values[name]) for name in required]


def encode_packet(sequence: int, arm: Sequence[float]) -> bytes:
    """Encode the small versioned protocol shared with the Isaac receiver."""
    if len(arm) != 6:
        raise ValueError("Piper-H packets require exactly six arm joints")
    payload = {
        "version": 1,
        "sequence": int(sequence),
        "timestamp": time.time(),
        "joint_names": list(ARM_JOINTS),
        "joint_positions": [float(value) for value in arm],
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


class JointStateUdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("piperh_joint_state_udp_bridge")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5015)
        self.declare_parameter("send_rate_hz", 60.0)
        self.declare_parameter("stale_timeout_sec", 0.5)

        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        rate = float(self.get_parameter("send_rate_hz").value)
        self._timeout = float(self.get_parameter("stale_timeout_sec").value)
        if rate <= 0.0 or self._timeout <= 0.0:
            raise ValueError("send rate and stale timeout must be positive")

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._latest: list[float] | None = None
        self._received_at = 0.0
        self._sequence = 0
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / rate, self._send)
        self.get_logger().info(
            f"simulation-only bridge ready: udp://{self._host}:{self._port}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        arm = ordered_positions(message.name, message.position, ARM_JOINTS)
        if arm is None:
            return
        self._latest = arm
        self._received_at = time.monotonic()

    def _send(self) -> None:
        if self._latest is None:
            return
        if time.monotonic() - self._received_at > self._timeout:
            return
        packet = encode_packet(self._sequence, self._latest)
        self._socket.sendto(packet, (self._host, self._port))
        self._sequence += 1

    def destroy_node(self) -> bool:
        self._socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        except Exception:
            if rclpy.ok():
                raise
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
