"""Desktop selector for starting exactly one supported real robot arm."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import sys

from rebot_xbox_hardware.launcher_command import (
    ARM_PROFILES,
    build_launch_arguments,
)


def _can_exists(channel: str) -> bool:
    return Path("/sys/class/net", channel).exists()


def main() -> int:
    # Import Qt only for the executable. Pure selection/command tests therefore
    # do not need a display server or create a QApplication.
    from PyQt5.QtCore import QProcess, QTimer
    from PyQt5.QtGui import QFont, QTextCursor
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QFormLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QVBoxLayout,
        QWidget,
    )

    class ArmSelectorWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("机械臂统一启动器")
            self.resize(900, 700)
            self._process = QProcess(self)
            self._process.setProcessChannelMode(QProcess.MergedChannels)
            self._process.readyReadStandardOutput.connect(self._read_output)
            self._process.started.connect(self._process_started)
            self._process.finished.connect(self._process_finished)
            self._process.errorOccurred.connect(self._process_error)
            self._closing = False
            self._previous_default_channel = ""

            root = QWidget(self)
            layout = QVBoxLayout(root)
            title = QLabel("真实机械臂统一控制入口", root)
            title.setFont(QFont("Sans", 18, QFont.Bold))
            subtitle = QLabel(
                "选择机械臂后，驱动、模型、MoveIt、RViz、关节限位和 Xbox 配置会一起切换。",
                root,
            )
            subtitle.setWordWrap(True)
            layout.addWidget(title)
            layout.addWidget(subtitle)

            selection_box = QGroupBox("启动配置", root)
            form = QFormLayout(selection_box)
            self.arm_combo = QComboBox(selection_box)
            for profile in ARM_PROFILES:
                self.arm_combo.addItem(profile.label, profile.key)
            self.channel_edit = QLineEdit(selection_box)
            self.joy_edit = QLineEdit("/dev/input/js0", selection_box)
            self.profile_detail = QLabel(selection_box)
            self.profile_detail.setWordWrap(True)
            form.addRow("机械臂", self.arm_combo)
            form.addRow("通信通道", self.channel_edit)
            form.addRow("Xbox 设备", self.joy_edit)
            form.addRow("将启动", self.profile_detail)
            layout.addWidget(selection_box)

            options_box = QGroupBox("复用功能", root)
            options = QVBoxLayout(options_box)
            self.rviz_check = QCheckBox("启动 RViz 与匹配的机器人模型", options_box)
            self.rviz_check.setChecked(True)
            self.zone_check = QCheckBox("启用 MoveIt 三维禁区", options_box)
            self.zone_check.setChecked(True)
            self.teach_check = QCheckBox("启用拖动示教与动作回放", options_box)
            self.teach_check.setChecked(True)
            options.addWidget(self.rviz_check)
            options.addWidget(self.zone_check)
            options.addWidget(self.teach_check)
            layout.addWidget(options_box)

            self.safety_check = QCheckBox(
                "我已确认实体急停可用、机械臂固定可靠、工作区无人且无障碍物。",
                root,
            )
            self.safety_check.setStyleSheet("font-weight: 600; color: #b23a2b;")
            layout.addWidget(self.safety_check)

            buttons = QHBoxLayout()
            self.start_button = QPushButton("启动所选机械臂", root)
            self.stop_button = QPushButton("停止当前控制栈", root)
            self.stop_button.setEnabled(False)
            self.start_button.clicked.connect(self._start)
            self.stop_button.clicked.connect(self._stop)
            buttons.addWidget(self.start_button)
            buttons.addWidget(self.stop_button)
            layout.addLayout(buttons)

            self.status_label = QLabel("状态：等待选择", root)
            self.status_label.setStyleSheet("font-weight: 600;")
            layout.addWidget(self.status_label)
            self.log = QPlainTextEdit(root)
            self.log.setReadOnly(True)
            self.log.setMaximumBlockCount(4000)
            self.log.setPlaceholderText("启动日志将在这里显示。")
            layout.addWidget(self.log, 1)
            self.setCentralWidget(root)

            self.arm_combo.currentIndexChanged.connect(self._profile_changed)
            self._profile_changed()

        def _profile(self):
            return ARM_PROFILES[self.arm_combo.currentIndex()]

        def _profile_changed(self) -> None:
            profile = self._profile()
            current = self.channel_edit.text().strip()
            if not current or current == self._previous_default_channel:
                self.channel_edit.setText(profile.default_channel)
            self._previous_default_channel = profile.default_channel
            self.teach_check.setEnabled(profile.supports_drag_teach)
            self.teach_check.setChecked(profile.supports_drag_teach)
            limitations = []
            if not profile.supports_gripper:
                limitations.append("无夹爪")
            if not profile.supports_drag_teach:
                limitations.append("使用预设动作组，不启用重力补偿拖动示教")
            suffix = "；" + "，".join(limitations) if limitations else ""
            self.profile_detail.setText(profile.detail + suffix)
            self.safety_check.setChecked(False)

        def _validate_hardware(self, profile, channel: str, joy: str) -> bool:
            if not self.safety_check.isChecked():
                QMessageBox.warning(self, "安全确认", "请先完成并勾选实体安全确认。")
                return False
            if profile.robot == "piperh" or profile.model == "rs":
                if "/" in channel or not _can_exists(channel):
                    QMessageBox.critical(
                        self,
                        "CAN 不可用",
                        f"没有找到 CAN 接口 {channel!r}。请先配置 1 Mbit/s CAN。",
                    )
                    return False
            elif not Path(channel).exists():
                QMessageBox.critical(
                    self,
                    "串口不可用",
                    f"没有找到串口 {channel!r}。请检查设备名和权限。",
                )
                return False
            if not Path(joy).exists():
                answer = QMessageBox.question(
                    self,
                    "Xbox 未连接",
                    f"没有找到 {joy!r}。仍要启动以便只使用 RViz/MoveIt 吗？",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No,
                )
                if answer != QMessageBox.Yes:
                    return False
            return True

        def _start(self) -> None:
            if self._process.state() != QProcess.NotRunning:
                return
            profile = self._profile()
            channel = self.channel_edit.text().strip()
            joy = self.joy_edit.text().strip()
            try:
                arguments = build_launch_arguments(
                    profile,
                    channel=channel,
                    joy_device=joy,
                    use_rviz=self.rviz_check.isChecked(),
                    use_forbidden_zones=self.zone_check.isChecked(),
                    use_teach=self.teach_check.isChecked(),
                )
            except ValueError as error:
                QMessageBox.warning(self, "配置无效", str(error))
                return
            if not self._validate_hardware(profile, channel, joy):
                return
            self.log.clear()
            self.log.appendPlainText("$ ros2 " + " ".join(arguments))
            self.log.appendPlainText("注意：启动器不会替代实体急停。\n")
            self.status_label.setText(f"状态：正在启动 {profile.label}…")
            self._set_configuration_enabled(False)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self._process.setProgram("ros2")
            self._process.setArguments(arguments)
            self._process.start()

        def _set_configuration_enabled(self, enabled: bool) -> None:
            for widget in (
                self.arm_combo,
                self.channel_edit,
                self.joy_edit,
                self.rviz_check,
                self.zone_check,
                self.safety_check,
            ):
                widget.setEnabled(enabled)
            self.teach_check.setEnabled(enabled and self._profile().supports_drag_teach)

        def _stop(self) -> None:
            if self._process.state() == QProcess.NotRunning:
                return
            self.status_label.setText("状态：正在安全停止控制栈…")
            self.stop_button.setEnabled(False)
            process_id = int(self._process.processId())
            if process_id > 1:
                try:
                    # ros2 launch handles SIGINT by shutting down every child node.
                    os.kill(process_id, signal.SIGINT)
                except OSError:
                    self._process.terminate()
            else:
                # QProcess may still be in Starting state and have no PID yet.
                self._process.terminate()
            QTimer.singleShot(7000, self._force_stop_if_needed)

        def _force_stop_if_needed(self) -> None:
            if self._process.state() != QProcess.NotRunning:
                self.log.appendPlainText("正常停止超时，终止 ros2 launch 主进程。")
                self._process.kill()

        def _read_output(self) -> None:
            text = bytes(self._process.readAllStandardOutput()).decode(
                errors="replace"
            )
            if text:
                self.log.moveCursor(QTextCursor.End)
                self.log.insertPlainText(text)
                self.log.ensureCursorVisible()

        def _process_started(self) -> None:
            self.status_label.setText(f"状态：{self._profile().label} 控制栈运行中")

        def _process_finished(self, exit_code: int, _exit_status) -> None:
            self._read_output()
            self._set_configuration_enabled(True)
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.safety_check.setChecked(False)
            if self._closing:
                QApplication.instance().quit()
                return
            if exit_code == 0:
                self.status_label.setText("状态：控制栈已停止")
            else:
                self.status_label.setText(f"状态：控制栈退出，代码 {exit_code}")

        def _process_error(self, _error) -> None:
            self.log.appendPlainText("无法启动 ros2 命令：" + self._process.errorString())
            if self._process.state() == QProcess.NotRunning:
                self._set_configuration_enabled(True)
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                self.safety_check.setChecked(False)
                self.status_label.setText("状态：启动失败")

        def shutdown_from_signal(self) -> None:
            """Honor ros2 launch shutdown and also stop the nested stack."""
            self._closing = True
            if self._process.state() != QProcess.NotRunning:
                self._stop()
            else:
                self.close()

        def closeEvent(self, event) -> None:
            if self._process.state() != QProcess.NotRunning:
                answer = QMessageBox.question(
                    self,
                    "停止机械臂控制",
                    "关闭启动器会停止当前机械臂控制栈，是否继续？",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No,
                )
                if answer != QMessageBox.Yes:
                    event.ignore()
                    return
                self._closing = True
                self._stop()
                if not self._process.waitForFinished(8000):
                    self._process.kill()
                    self._process.waitForFinished(2000)
            event.accept()

    app = QApplication([sys.argv[0]])
    app.setApplicationName("reBot/Piper-H 统一启动器")
    window = ArmSelectorWindow()
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.shutdown_from_signal())
    signal.signal(signal.SIGTERM, lambda *_: window.shutdown_from_signal())
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
