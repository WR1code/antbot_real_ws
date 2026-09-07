# 源码快照说明

本工作区于 2026-09-05 从开发机 `/home/w/project` 整理，源仓库基线提交为
`70e97e85ab82`。复制采用的是当时工作树内容，因此包含尚未提交但与真机有关的修正，
尤其包括 H743 串口自动识别、主机工具、接线文档和固件健康状态修复，以及
`robotcar_navigation` 的本地修改。

## 来源映射

- `src/antbot_*`：`antbot/ros2_ws/src` 中保留的真机相关包；
- `src/robotcar_navigation`：`robotcar/src/robotcar_navigation` 的实体副本；
- `firmware/rs00_fk743_test`：H743 工程、主机工具、测试和接线文档；
- `firmware/images`：整理时已有的 Debug/Release HEX；
- `maps/real`：为 Orin 实机建图预留；原 `artifacts/maps` 属于仿真数据，未复制。

未复制仿真包、构建产物、缓存和旧底盘控制实现。SDK 内嵌的重复
`vanjee_lidar_msg` 保留在源码树中，但通过 `COLCON_IGNORE` 排除，实际使用顶层消息包。
