# 外部工作区

这里集中保存 PUDU 仿真栈依赖的第三方 ROS 2 工作区：

- `nav2_reference_ws`：Nav2 参考实现及本地 Gazebo 依赖
- `linorobot_sim_ws`：Linorobot2 Gazebo 仿真
- `exploration_reference_ws`：自主探索功能
- `scurm_sentry_ws`：SCURM、FAST-LIO 与 ICP 定位参考实现
- `arena4_ws`：独立 Arena 仿真/评测参考环境（不属于当前运行依赖链）

它们在目录上归属于本项目，但仍保持为独立的 colcon overlay，避免同名包和依赖层次互相污染。统一路径定义位于项目根目录的 `stack_paths.bash`。

为兼容已有终端命令和迁移前的 colcon 产物，`/home/robot/*_ws` 保留为指向这里的软链接。新脚本均使用本目录中的规范路径。

`build_all.bash` 只编译当前 PUDU 功能依赖的前四个工作区。Arena4 自带另一套大型 ROS 环境，通过根目录的 `build_arena4.bash`、`start_arena4.sh`、`stop_arena4.sh` 独立管理，避免污染当前 overlay。
