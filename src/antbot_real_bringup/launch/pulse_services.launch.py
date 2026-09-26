"""Launch read-only pulse services as children of the real operator session."""

import os
import socket
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def prepare_services(context):
    workspace = Path(LaunchConfiguration("pulse_workspace").perform(context))
    sensor_port = LaunchConfiguration("pulse_port").perform(context)
    http_port = int(LaunchConfiguration("pulse_web_port").perform(context))
    chassis_port = LaunchConfiguration("port").perform(context)
    if os.path.realpath(sensor_port) == os.path.realpath(chassis_port):
        raise RuntimeError("压力串口不能与 H743 底盘串口相同")
    if not os.path.exists(sensor_port):
        raise RuntimeError(f"请先连接压力传感器：{sensor_port}；仅启动底盘可设置 ANTBOT_START_PULSE=false")
    source = workspace / "src" / "rebotarm_pulse"
    for path in (source / "web" / "index.html",
                 workspace / "scripts" / "start_pulse_web.sh",
                 workspace / "scripts" / "start_pulse_pressure.sh"):
        if not path.is_file():
            raise RuntimeError(f"缺少把脉启动文件：{path}")
    sys.path.insert(0, str(source))
    try:
        from rebotarm_pulse.pressure_serial_owner import serial_owner_pids
        # Validate runtime imports without opening the sensor or starting nodes.
        import rebotarm_pulse.pressure_serial_bridge  # noqa: F401
        import rebotarm_pulse.pulse_web_gateway  # noqa: F401
        owners = serial_owner_pids(sensor_port)
    finally:
        sys.path.remove(str(source))
    if owners:
        raise RuntimeError(f"压力串口已被 PID {owners} 占用，请先关闭独立采集终端")
    with socket.socket() as probe:
        # Match HTTPServer.allow_reuse_address: an old TIME_WAIT connection
        # must not prevent restarting the dashboard, but a listener still does.
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", http_port))
        except OSError as error:
            raise RuntimeError(f"网页端口 {http_port} 不可用，请先关闭独立网页终端") from error
    return [
        LogInfo(msg=f"把脉大屏：http://127.0.0.1:{http_port}/#report"),
        ExecuteProcess(cmd=[str(workspace / "scripts" / "start_pulse_web.sh"),
                            "-p", f"port:={http_port}"], output="screen"),
        ExecuteProcess(cmd=[str(workspace / "scripts" / "start_pulse_pressure.sh"),
                            sensor_port], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pulse_workspace", default_value=os.environ.get("ANTBOT_REAL_WS", "")),
        DeclareLaunchArgument("pulse_port", default_value=os.environ.get(
            "ANTBOT_PULSE_PORT", "/dev/robot_serial")),
        DeclareLaunchArgument("pulse_web_port", default_value=os.environ.get("ANTBOT_PULSE_WEB_PORT", "8765")),
        OpaqueFunction(function=prepare_services),
    ])
