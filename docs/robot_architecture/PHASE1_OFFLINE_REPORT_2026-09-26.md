# Phase 1 离线安全改造报告（2026-09-26）

本报告记录 Phase 0 刷新、迁移前 baseline、Phase 1 实施和无硬件验收。全过程未打开
真实 CAN/串口、未启动 Nav2、未 enable 或移动底盘/机械臂、未修改 MCU 固件。

## 1. Phase 0 刷新结果

在分支 `main`、HEAD `ad487d319debaebc920bf79c06ce3b6a8c0a64e2` 的 dirty
worktree 上重新扫描了 Piper Action/Servo/vendor/MIT 路径、reBot 生命周期、底盘
producer/consumer、launch 默认值以及 shutdown/signal/reconnect 行为。改造前确认 S01、
S02、S03、C01、C02、C04 仍存在，完整证据见
[`PHASE0_REFRESH_2026-09-26.md`](PHASE0_REFRESH_2026-09-26.md)。该文件是改造前快照，
其“仍存在”状态不覆盖本报告的实施后结论。

刷新文档创建前统计为 14 个 tracked change、1691 个 untracked 文件；把刷新文档纳入
baseline 时为 14 个 tracked change、1692 个 untracked 文件。这一差异只来自新增的
Phase 0 刷新文档。

## 2. baseline 与恢复

baseline 位于：

`/home/w/project/antbot_real_ws_baselines/phase1_pre_20260926_01`

包含 binary-capable tracked worktree patch、空的 staged patch、1692 个 untracked 文件的
tar、精确路径清单、Git status、branch/HEAD/origin 元数据、SHA256 和恢复说明。SHA256
复核全部通过。它不包含 `.git`、Git ignored 的 `build/install/log` 产物、设备状态或工作区
外文件；创建过程没有删除、移动或覆盖用户文件。

恢复时应新建独立 checkout 并 checkout 上述 HEAD，先运行 `sha256sum -c SHA256SUMS`，
再 `git apply --binary tracked_worktree.patch`，最后把 `untracked-files.tar` 解到新工作区；
不得覆盖当前含有更新工作的目录。逐步命令见 baseline 内 `RESTORE.md`。

## 3. 修改文件

Phase 0/报告：

- `docs/robot_architecture/PHASE0_REFRESH_2026-09-26.md`
- `docs/robot_architecture/PHASE1_OFFLINE_REPORT_2026-09-26.md`
- `docs/robot_architecture/MIGRATION_PLAN.md`
- `docs/robot_architecture/ARCHITECTURE_ISSUES.md`
- `docs/robot_architecture/AUDIT_SUMMARY.txt`

Piper：

- `dual_arm_ws/src/piper/piper/command_authority.py`
- `dual_arm_ws/src/piper/piper/command_writer.py`
- `dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py`
- `dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py`
- `dual_arm_ws/src/piper/launch/start_single_piper.launch.py`
- `dual_arm_ws/src/piper/launch/start_single_piper_rviz.launch.py`
- `dual_arm_ws/src/piper/launch/start_two_piper.launch.py`
- `dual_arm_ws/src/piper/package.xml`
- `dual_arm_ws/src/piper/test/test_phase1_final_writer.py`
- `dual_arm_ws/src/piper/test/test_phase1_legacy_launch.py`
- `dual_arm_ws/src/piperh_control/piperh_control/command_authority.py`
- `dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py`
- `dual_arm_ws/src/piperh_control/package.xml`
- `dual_arm_ws/src/piperh_control/test/test_phase1_command_authority.py`
- `dual_arm_ws/src/piperh_control/test/test_playback_tracking.py`

reBot：

- `dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`
- `dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py`
- `dual_arm_ws/src/rebotarmcontroller/test/test_phase1_lifecycle.py`
- `dual_arm_ws/src/rebotarmcontroller/test/test_safe_home_completion.py`

底盘：

- `src/antbot_h743_bridge/antbot_h743_bridge/command_authority.py`
- `src/antbot_h743_bridge/antbot_h743_bridge/bridge.py`
- `src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py`
- `src/antbot_h743_bridge/package.xml`
- `src/antbot_h743_bridge/test/test_bridge_safety.py`
- `src/antbot_h743_bridge/test/test_phase1_base_authority.py`
- `src/antbot_real_bringup/config/real_base.yaml`
- `src/antbot_real_bringup/launch/real_base.launch.py`

## 4. 风险与改动映射

| 风险 | Phase 1 离线处理 | 实施后状态 |
|---|---|---|
| C01 Action/Servo 并发 | 单 owner、epoch token、monotonic lease；Action/Servo/Gravity 显式 acquire/renew/release；`_publish_driver` 再验 token | 离线关闭 |
| C02 vendor/SDK 旁路 | vendor 最终 `PiperCommandWriter` 再验 lease；raw PosCmd/enable topic 永久拒绝；MIT 每次 `send_joint` 前再验 lease；旧 unguarded node 永久拒绝启动 | 默认运行路径离线关闭 |
| S01 connect 自动 enable | reBot 拆为 DISCONNECTED/CONNECTED/INITIALIZED/ENABLED/FAULT/SAFE_STOP；connect 只连接，initialize 仍不使能，enable 显式调用 | 离线关闭，硬件反馈语义待验 |
| S02 shutdown 自动 home | shutdown 按 normal/fault/estop 分原因；默认不 home；fault/estop 禁止 home；显式 normal request 才可 home | 离线关闭，物理停机待验 |
| S03 legacy 双 Piper 危险默认 | 双臂 CAN 默认空、必须明确且不同；`auto_enable=false`；legacy 默认 fail closed，显式 opt-in 才解析 | 离线关闭 |
| C04 `/cmd_vel` 多 writer | producer 进入 operator arbitration；唯一内部 `TwistStamped` topic；owner UUID + lease；bridge 在 UART 前最终验证；disable/reconnect/reset/shutdown 撤销；禁用状态不重获 claim | Phase 1 范围离线关闭 |

所有 gate 都是 fail closed：无 owner、旧 epoch、过期 lease、release/revoke 后 token、非选中
source 和锁定状态均不能进入运动 transport。ROS 2 topic identity 不是密码学身份；防恶意节点伪造
属于 DDS security/后续全局 ResourceManager 范围，不把它误报为本轮已实现。

## 5. 新增/更新的安全测试

- Piper：Action/Servo 互斥、显式 ownership 切换、stale/released token、非 owner final writer、
  cancel/exception/shutdown release、vendor SDK final writer、raw topic 永久关闭、MIT final writer、
  legacy launch fail closed。
- reBot：connect/initialize 不 enable、不运动；enable 后才允许运动；disable 后拒绝；normal 默认
  不 home；显式 normal home；fault/ESTOP 永不 home；清理仍执行 stop/disable/disconnect。
- 底盘：单 owner、lease expiry、非 owner rejection、disable/ESTOP revoke、锁定时不重获 claim、
  锁定时不写 motion frame、最终 UART writer 检查、真实配置不再把 `/cmd_vel` 直连 bridge。

对应测试文件见第 3 节的 `test_phase1_*` 及更新的既有 safety/playback/safe-home 测试。

## 6. 离线验收结果

- 精确功能/fake transport 测试：`117 passed`。
- 根工作区 `./scripts/test.sh`：ROS `93 tests, 0 errors, 0 failures`；固件协议 CTest
  `10/10 passed`。
- 包级测试：`piperh_control 46/46 passed`；`rebotarmcontroller 24/24 passed`。
- 构建：`antbot_h743_bridge`、`antbot_real_bringup`、`piper`、`piperh_control`、
  `rebotarmcontroller` 全部成功。
- `python3 -m compileall` 成功。
- `real_base.launch.py` 与 `start_two_piper.launch.py` 的 `--show-args` 静态解析成功；没有真正 launch 节点。
- baseline 的 SHA256 全部通过。

`piper` 全包历史 lint 仍失败：flake8 报 355 项且 pep257 失败，主要覆盖原 vendor 风格文件，
也包含本轮新增文件的格式告警；功能测试和构建均通过。未在安全迁移中机械格式化整套 vendor
代码，以免扩大变更面。此前 build 目录还保留两个与本轮无关的 `rebot_teach_mode` 旧测试失败，
本轮未运行或修改该 Phase 2+ 功能。

## 7. 仍未解决的问题

- C03 切臂 cancel 终态/停稳证明、C05 全局底盘与双臂任务互锁仍开放。
- 尚无完整 Supervisor、ResourceManager、统一 health adapter、`robot.launch.py`；它们属于后续阶段。
- 无可信真实 odom，Nav2 必须继续关闭。
- ROS 2 内部 identity/lease 防止偶发多 writer，不构成对恶意同图节点的安全认证。
- Piper 受控 enable service 仍负责硬件使能；是否真的使能成功必须依据现场反馈验证。
- 代码质量门禁需另立任务清理 Piper vendor 包 lint，不能把 lint 通过误当成运动安全证明。

## 8. Phase 1 完成度

本轮规定的 Phase 0 刷新、baseline、Phase 1 最小 authoritative ownership、reBot 生命周期、
legacy fail-closed 和无硬件测试已完成。按本轮软件离线范围计为 **100%**；这不代表物理安全、
整机集成或后续 Phase 已完成。

## 9. 台架必须验证

以下全部标记为 **HARDWARE VERIFICATION REQUIRED**：独立急停及其反馈、机械臂掉电/刹车
行为、左右臂物理映射、CAN interface/bitrate、USB identity、MotorBridge transport、H743/MCU
固件版本、Piper/reBot enable 实际反馈、lease 超时后的物理停止、UART/CAN 断线重连、Action 与
Servo 交接时的实臂静止、normal/fault/ESTOP shutdown 无隐式运动。必须使用限速、架空/隔离、
人工急停在手的分项台架流程；不得在此报告基础上直接运行整机任务。

## 10. 是否可进入 Phase 1 真机验收

离线安全测试全部通过，当前默认可达的软件运动旁路已收口，可以进入受控的 Phase 1 台架验证：

`READY FOR PHASE 1 BENCH VALIDATION`

该结论只授权制定并人工执行台架清单，不授权本轮自动连接或运动真机。

## 11. 是否可进入 Phase 2

**不可以。** 必须先完成第 9 节台架项目、记录证据并处理发现的问题；本轮没有创建 Supervisor、
没有启动 Nav2，也没有实施任何 Phase 2+ 内容。
