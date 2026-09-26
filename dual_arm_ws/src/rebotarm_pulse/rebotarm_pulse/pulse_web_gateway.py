"""Serve the pulse dashboard and relay ROS pressure topics to the browser.

The pressure_serial_bridge remains the only owner of the physical serial port.
This node only subscribes to ROS topics, so the dashboard and robot safety stack
can consume the same samples concurrently.
"""

from __future__ import annotations

from collections import deque
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Bool


class PulseEventBuffer:
    """Thread-safe, bounded event history for Server-Sent Events clients."""

    def __init__(self, maxlen: int = 4096) -> None:
        self._events: deque[tuple[int, dict]] = deque(maxlen=maxlen)
        self._next_sequence = 1
        self._condition = threading.Condition()
        self.status = {"serial_connected": False, "zero_calibrated": False}

    def publish(self, event: dict) -> int:
        with self._condition:
            sequence = self._next_sequence
            self._next_sequence += 1
            self._events.append((sequence, event))
            self._condition.notify_all()
            return sequence

    def update_status(self, key: str, value: bool) -> None:
        with self._condition:
            self.status[key] = bool(value)
        self.publish({"type": "status", **self.snapshot()})

    def snapshot(self) -> dict:
        with self._condition:
            return dict(self.status)

    def latest_sequence(self) -> int:
        with self._condition:
            return self._events[-1][0] if self._events else 0

    def events_after(self, sequence: int, timeout: float = 10.0):
        with self._condition:
            if not self._events or self._events[-1][0] <= sequence:
                self._condition.wait(timeout=timeout)
            return [item for item in self._events if item[0] > sequence]


class PulseDashboardHandler(SimpleHTTPRequestHandler):
    server_version = "PulseDashboard/1.0"

    def __init__(self, *args, directory: str, event_buffer: PulseEventBuffer, **kwargs):
        self.event_buffer = event_buffer
        super().__init__(*args, directory=directory, **kwargs)

    def log_message(self, format, *args):
        # Avoid printing one line per long-lived SSE reconnect/asset request.
        return

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            body = json.dumps(self.event_buffer.snapshot()).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/stream":
            self._serve_events()
            return
        super().do_GET()

    def _serve_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        sequence = self.event_buffer.latest_sequence()
        initial = {"type": "status", **self.event_buffer.snapshot()}
        try:
            self.wfile.write(f"data: {json.dumps(initial)}\n\n".encode("utf-8"))
            self.wfile.flush()
            while True:
                events = self.event_buffer.events_after(sequence)
                if not events:
                    self.wfile.write(b": keepalive\n\n")
                for sequence, event in events:
                    payload = json.dumps(event, separators=(",", ":"))
                    self.wfile.write(f"id: {sequence}\ndata: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return


class PulseWebGateway(Node):
    def __init__(self) -> None:
        super().__init__("pulse_web_gateway")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("topic_prefix", "/piperh/pulse")
        self.declare_parameter("site_directory", "")
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        site_directory = str(self.get_parameter("site_directory").value).strip()
        if not site_directory:
            site_directory = os.path.join(
                get_package_share_directory("rebotarm_pulse"), "web"
            )
        if not os.path.isfile(os.path.join(site_directory, "index.html")):
            raise ValueError(f"pulse dashboard not found: {site_directory}")
        if not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")

        self.events = PulseEventBuffer()
        self.latest_temperature = {channel: None for channel in ("S1", "S2", "S3")}
        for channel in ("S1", "S2", "S3"):
            lower = channel.lower()
            self.create_subscription(
                Temperature,
                f"{prefix}/raw/{lower}/temperature",
                partial(self._temperature_callback, channel),
                100,
            )
            self.create_subscription(
                FluidPressure,
                f"{prefix}/raw/{lower}/pressure",
                partial(self._pressure_callback, channel),
                100,
            )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            f"{prefix}/serial_connected",
            lambda message: self.events.update_status(
                "serial_connected", message.data
            ),
            status_qos,
        )
        self.create_subscription(
            Bool,
            f"{prefix}/zero_calibrated",
            lambda message: self.events.update_status("zero_calibrated", message.data),
            status_qos,
        )

        handler = partial(
            PulseDashboardHandler,
            directory=site_directory,
            event_buffer=self.events,
        )
        self.http_server = ThreadingHTTPServer((host, port), handler)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="pulse-dashboard-http",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(f"pulse dashboard available at http://{host}:{port}/")

    def _temperature_callback(self, channel: str, message: Temperature) -> None:
        self.latest_temperature[channel] = float(message.temperature)

    def _pressure_callback(self, channel: str, message: FluidPressure) -> None:
        pressure_pa = float(message.fluid_pressure)
        self.events.publish(
            {
                "type": "sample",
                "channel": channel,
                "pressure_pa": pressure_pa,
                "pressure_hpa": pressure_pa / 100.0,
                "temperature_c": self.latest_temperature[channel],
                "received_at_ms": time.time_ns() // 1_000_000,
            }
        )

    def close(self) -> None:
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)


def main() -> None:
    rclpy.init()
    node = PulseWebGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
