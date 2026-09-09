# 小车主机拉取更新提示词

把下面整段提示词复制给小车主机（Jetson Orin）上的 Codex 或终端助手。默认仓库目录
为 `/home/orin/antbot_real_ws`；如果实际用户名或目录不同，让它按实机路径替换。

```text
请在这台小车主机上安全更新 AntBot 真机仓库，并完成构建验证。

目标仓库：https://github.com/WR1code/antbot_real_ws.git
目标分支：main
默认目录：/home/orin/antbot_real_ws

严格要求：
1. 不得执行 git reset --hard、git clean、强制 checkout、强制 push，不能删除或覆盖主机现有改动。
2. 开始前检查目录、当前分支、origin、git status 和正在运行的 ROS/AntBot 进程。
3. 如果仓库有未提交改动、分支发生分叉、origin 不匹配或拉取不能 fast-forward，立即停止并报告具体情况，不要自行丢弃、stash 或合并。
4. 如果 AntBot、RViz2 或 H743 桥正在运行，不要主动结束进程，也不要更新其正在使用的工作区；停止本次更新并报告，等待人工停机后再继续。
5. 原始 STEP 为约 308 MB 的 Git LFS 文件，但小车运行只需要仓库普通 Git 中的轻量 STL。本次设置 GIT_LFS_SKIP_SMUDGE=1，不下载 STEP 实体。
6. 不发送 /cmd_vel，不查询或使能电机，不打开串口，不启动 RViz2；最终只运行 --check-only。

请按以下流程执行：

- 如果 /home/orin/antbot_real_ws 不存在：
  GIT_LFS_SKIP_SMUDGE=1 git clone --branch main --single-branch https://github.com/WR1code/antbot_real_ws.git /home/orin/antbot_real_ws

- 如果目录已经存在：
  cd /home/orin/antbot_real_ws
  git status --short --branch
  git remote -v
  git fetch origin main
  git log --oneline --decorate --max-count=5 HEAD origin/main
  确认工作树干净、当前分支为 main、origin 正确且可以快进后，再执行：
  GIT_LFS_SKIP_SMUDGE=1 git pull --ff-only origin main

- 拉取完成后，在 /home/orin/antbot_real_ws 中执行：
  ./scripts/install_dependencies.sh core
  ./scripts/build.sh
  ./scripts/test.sh
  ./scripts/start_antbot_operator.sh --check-only

- 最后验证：
  git status --short --branch
  git rev-parse HEAD
  source ./scripts/setup_env.sh
  ros2 pkg prefix antbot_description
  ros2 pkg prefix antbot_navigation
  ros2 pkg prefix antbot_h743_bridge
  上述三个包必须都位于 /home/orin/antbot_real_ws/install/ 下。

完成后用中文报告：更新前后提交号、是否快进、构建结果、测试统计、check-only 结果、三个包的实际路径，以及任何未解决问题。不要自动启动小车或使能电机。

如果我另外明确要求在小车主机上下载原始 CAD，确认 git-lfs 已安装后再执行：
git lfs pull --include="src/antbot_description/cad/*.STEP"
```

这段提示词刻意跳过原始 STEP 的下载。RViz2 使用
`src/antbot_description/meshes/gk4xc_001_003_full.stl`，因此不会影响模型显示。
