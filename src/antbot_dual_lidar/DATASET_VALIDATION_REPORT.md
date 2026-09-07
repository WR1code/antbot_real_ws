# Phase 2A 数据集验证报告

状态：五段前左最小 MCAP 已真实录制，但严格验证未通过。

录制器已实现自动 case/时间命名、MCAP、profile、git 状态、USD/config SHA-256、
开始/结束仿真时钟、`ros2 bag info` 和录后校验。校验器检查每个必需话题消息数、
header 时间回退及前/后雷达与主 IMU 的共同覆盖。辅助 IMU 只在显式环境变量开启
时额外记录。

未生成的数据集不能以空 manifest 声称通过；实际统计将在录制后由每个 bag 旁的
`.validation.yaml` 提供。

实际路径：`/home/w/project/antbot/artifacts/phase2a_bags/minimal_front`。五段
MCAP 总计约 134 MiB，均包含 raw、deskew、主 IMU、ground truth、clock、TF 和
TF static，时间戳回退均为 0，raw 雷达 missing frame 均为 0。

严格 IMU 扫描覆盖未通过：录包订阅边界导致每段开头/末尾 2–3 帧的完整
`[header-period, header]` 区间超出 bag 内 IMU 范围；覆盖分别为
37/39、44/47、45/48、47/49、50/53。因此 manifest 正确标记为
`RECORDED_FAILED_VALIDATION`，没有把“已录制”误写成“已通过”。

此外 Python 真值 deskew 节点仅输出约 3.0–3.3 Hz，而 raw 为 10 Hz，说明当前逐点
实现不能在线覆盖每一帧。该性能问题与时间模型未锁定共同阻塞最小基线验收。

## Native + pre/post-roll 重录结果（取代以上失败数据集）

新路径：`/home/w/project/antbot/artifacts/phase2a_bags_native`。五段全部使用
`pre-roll=1.2 s`、动作 `3.0 s`、`post-roll=1.2 s`，同时包含 native raw、
truth deskew、world reference、truth odom、主 IMU、clock、TF/TF static 和
控制命令。逐包真实内容校验均为 `passed: true`。

| case | raw 总帧 | 边界剔除 | 完整覆盖有效帧 | 有效覆盖率 | clock 重复/回退 |
|---|---:|---:|---:|---:|---:|
| 01_static | 53 | 2 | 51 | 100% | 0/0 |
| 02_left_rotation | 54 | 1 | 53 | 100% | 0/0 |
| 03_right_rotation | 50 | 1 | 49 | 100% | 0/0 |
| 04_wall_motion | 55 | 2 | 53 | 100% | 0/0 |
| 05_column_motion | 54 | 0 | 54 | 100% | 0/0 |

覆盖判定使用每帧实际完整区间
`[Header + min(time_offset_ns), Header + max(time_offset_ns)]`，要求 IMU 在两端
均可插值。边界帧单独统计，不进入有效集合；内部覆盖不足帧全部为 0。SHA-256、
频率、最大 gap、每个 topic 数量和完整路径见
`artifacts/phase2a_minimal_dataset_manifest_native.yaml`。
