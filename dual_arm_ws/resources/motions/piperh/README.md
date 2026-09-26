# Piper-H 演示动作文件

用 RViz 的 `Piper-H 预设动作` 面板把动作点组保存为 `*.motion.json`，把最终执行
顺序保存到本目录的 `*.sequence.json`。建议文件基名与
`lab_demos/.../config/action_group_catalog.yaml` 中的逻辑动作名一致。

这里不放未经验证的示例关节坐标。每个文件必须先经过 Isaac 仿真和现场低速验证，
再把动作目录中的对应登记项从 `pending_capture` 改成 `validated`。
