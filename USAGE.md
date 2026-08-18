# PUDU Robot Workspace 使用指南

## 1. 项目功能

| 功能 | 说明 | 启动入口 |
| --- | --- | --- |
| 普通地图导航 | Gazebo 2WD 机器人、已有地图、AMCL、Nav2、RViz | `./start_linorobot.sh` |
| 在线建图 | SLAM Toolbox 在线生成二维地图 | `./start_linorobot.sh --slam` |
| 自动探索建图 | SLAM + Nav2 + `explore_lite` 前沿探索 | `./start_linorobot.sh --explore` |
| SCURM 2D 导航 | Theta*、约束平滑、MPPI、自适应脱困；使用 AMCL | `./start_linorobot.sh --scurm-nav` |
| SCURM 3D 定位导航 | Gazebo 3D 雷达、`/imu/data`、ICP + FAST-LIO；不启动 AMCL | `./start_linorobot.sh --scurm-lio` |
| FAST-LIO 3D 建图 | 使用仿真雷达和 IMU 生成并保存 PCD | `./start_linorobot.sh --scurm-lio-map` |
| 已知地图全覆盖 | 根据 `/map` 自动规划机器人可达自由区的弓形覆盖路线 | 在导航命令后增加 `--coverage` |
| 1.5× 高速仿真 | Gazebo 物理时间目标 1.5×，只影响 SCURM LIO 模式 | 增加 `--fast-sim` |
| 定位误差评估 | 将 FAST-LIO 定位与 Gazebo ground truth 对比并输出误差 | 随 `--scurm-lio` 自动启动 |
| Arena4 导航评测 | Gazebo Harmonic、动态行人、任务生成、多机器人/多规划器评测 | `./start_arena4.sh` |

全覆盖 RViz 默认显示：黄色待执行路线、橙色未覆盖区域、绿色已覆盖区域、蓝色实际轨迹、紫色位姿箭头，以及 `7×7 m` 局部障碍代价地图。位姿箭头不显示数字文字。

## 2. 整合后的目录

所有相关工作区现已集中在 `/home/robot/pudu_robot_ws`：

```text
pudu_robot_ws/
├── src/                              # PUDU 自有功能包
├── external/
│   ├── nav2_reference_ws/            # Nav2、TurtleBot3、本地 Gazebo
│   ├── linorobot_sim_ws/             # Linorobot2 仿真
│   ├── exploration_reference_ws/     # explore_lite
│   ├── scurm_sentry_ws/              # SCURM、FAST-LIO、ICP
│   └── arena4_ws/                    # 独立 Arena 仿真评测环境
├── stack_paths.bash                  # 集中路径定义
├── build_all.bash                    # 全栈编译
├── build_arena4.bash                 # Arena4 独立增量编译
├── start_linorobot.sh                # 仿真统一入口
├── stop_linorobot.sh                 # Linorobot/SCURM 停止入口
├── start_arena4.sh                   # Arena4 可选启动入口
└── stop_arena4.sh                    # Arena4 安全停止入口
```

第三方工作区仍是独立 colcon overlay，避免同名包和依赖互相污染；`external/COLCON_IGNORE` 防止顶层构建重复扫描它们。原来的 `/home/robot/nav2_reference_ws`、`linorobot_sim_ws`、`exploration_reference_ws`、`scurm_sentry_ws`、`arena4_ws` 已保留为兼容软链接，旧命令仍可使用。Arena4 不进入普通 `build_all.bash`，但已经提供独立的检查、构建、启动、停止入口。

## 3. 启动前准备

进入工作区：

```bash
cd /home/robot/pudu_robot_ws
```

项目已构建时可直接启动。首次部署、整体迁移或修改第三方代码后，统一执行：

```bash
# 仅在本机缺少 Gazebo 时执行
./install_gazebo_local.sh

# 按 Nav2 → Linorobot → 探索 → SCURM → PUDU 的顺序增量编译
./build_all.bash
```

普通修改后的 `./build_all.bash` 是增量编译。迁移到新的绝对目录或需要彻底刷新 CMake 缓存时使用：

```bash
PUDU_CLEAN_CMAKE_CACHE=true ./build_all.bash
```

只修改 PUDU 自有包时，也可继续使用 `source setup_underlays.bash && colcon build --symlink-install`；只重编 SCURM 时使用 `./build_scurm_reference.sh`。

每个额外终端在运行 ROS 2 命令前都要加载同一运行环境：

```bash
source /home/robot/pudu_robot_ws/setup_linorobot_runtime.bash
```

默认使用 ROS Domain `42`，避免和其他 ROS 实验互相干扰。

## 4. 常用启动命令

```bash
# 普通已有地图导航（默认）
./start_linorobot.sh

# 二维在线建图
./start_linorobot.sh --slam

# 自动探索未知区域并建图
./start_linorobot.sh --explore

# SCURM 2D Nav2 配置
./start_linorobot.sh --scurm-nav

# 完整 3D ICP + FAST-LIO 定位导航
./start_linorobot.sh --scurm-lio

# 推荐的完整功能：3D 定位 + 全覆盖 + 1.5× 仿真
./start_linorobot.sh --scurm-lio --coverage --fast-sim
```

可传入自定义世界或机器人初始位姿：

```bash
./start_linorobot.sh --scurm-lio \
  world:=/absolute/path/scene.world \
  spawn_x:=1.0 spawn_y:=2.0 spawn_yaw:=0.0
```

限制：

- `--coverage` 需要稳定的已有地图和已启动的 Nav2，不能与 `--slam`、`--explore` 或 `--scurm-lio-map` 同时使用。
- `--fast-sim` 仅用于 `--scurm-lio` 或 `--scurm-lio-map`。
- `--scurm-nav` 是轻量二维模式；只有 `--scurm-lio` 使用 ICP + FAST-LIO 并关闭 AMCL。

## 5. 使用地图全覆盖

启动：

```bash
./start_linorobot.sh --scurm-lio --coverage --fast-sim
```

节点收到 `/map` 后会自动提取机器人可达自由区域、避开障碍和不可通行窄区，并在 RViz 显示整张弓形路线；不会自动移动。当前自带地图约规划 `692.6 m²`，换地图后会自动重新计算。

确认路线后执行：

```bash
source ./setup_linorobot_runtime.bash
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
```

常用控制：

```bash
ros2 service call /coverage/plan_map std_srvs/srv/Trigger '{}'  # 按 /map 重新规划
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'     # 暂停
ros2 service call /coverage/resume std_srvs/srv/Trigger '{}'    # 继续并规划遗漏区域
ros2 service call /coverage/cancel std_srvs/srv/Trigger '{}'    # 取消，保留覆盖历史
ros2 service call /coverage/query std_srvs/srv/Trigger '{}'     # 查询状态和进度
ros2 service call /coverage/clear std_srvs/srv/Trigger '{}'     # 清除区域和历史
```

只覆盖手选区域时，在 RViz 使用 **Publish Point** 按顺序点击至少三个顶点，然后执行：

```bash
ros2 service call /coverage/close_area std_srvs/srv/Trigger '{}'
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
```

再次调用 `/coverage/plan_map` 可恢复整图规划。

## 6. FAST-LIO 建图和 PCD

```bash
./start_linorobot.sh --scurm-lio-map \
  map_output_path:=/tmp/my_scurm_map.pcd

source ./setup_linorobot_runtime.bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

使用指定 PCD 进行仿真定位：

```bash
./start_linorobot.sh --scurm-lio \
  map_path:=/absolute/path/my_scurm_map.pcd
```

PCD 必须与当前 Gazebo 世界匹配，否则 ICP 初始配准和 FAST-LIO 定位会失败。

## 7. 可选真实 Livox/SCURM 入口

```bash
./start_scurm.sh --check
./start_scurm.sh --mapping
./start_scurm.sh --mapping --no-livox
./start_scurm.sh --localization map_path:=/absolute/path/GlobalMap.pcd
```

该入口用于 MID360、已有驱动或 rosbag，不属于 Gazebo 一键启动流程；以前台方式运行，使用 `Ctrl+C` 停止。上真机前必须校准雷达—IMU 外参并检查 Livox 网络地址。

## 8. 停止

停止由 `start_linorobot.sh` 启动的 Gazebo、Nav2/SLAM、RViz、探索和覆盖节点：

```bash
cd /home/robot/pudu_robot_ws
./stop_linorobot.sh
```

脚本只终止本项目记录的进程会话，不会清理无关 ROS 2 进程。切换启动模式前应先执行停止命令。

## 9. Arena4 可选仿真评测

Arena4 已保存在 `external/arena4_ws`，包含 498 个 ROS 源码包和自己的 install/Python 环境。它固定使用 ROS Domain `1`、Fast DDS 和 Gazebo Harmonic，与 PUDU/Linorobot 的 Domain `42` 隔离。

```bash
cd /home/robot/pudu_robot_ws

# 环境与运行状态检查
./start_arena4.sh --check
./start_arena4.sh --status

# 标准 GUI、固定场景、完全无界面
./start_arena4.sh
./start_arena4.sh --scenario
./start_arena4.sh --headless --scenario

# 选择机器人和世界
./start_arena4.sh robot:=turtlebot world:=hospital

# 只停止由 start_arena4.sh 启动的实例
./stop_arena4.sh
```

在额外终端操作 Arena ROS 图：

```bash
source /home/robot/pudu_robot_ws/setup_arena4_runtime.bash
ros2 node list
ros2 launch arena_bringup arena.launch.py --show-args
```

修改 Arena4 源码后独立增量编译：

```bash
./build_arena4.bash
```

构建脚本检测到 Arena 正在运行时会拒绝覆盖 install 空间。Arena 的启动和停止脚本只管理自己记录的进程会话；在旧终端手工启动的 Arena 必须仍在原终端按 `Ctrl+C` 停止。

## 10. 状态与日志

关键话题：

```text
/map
/scan
/scurm/lidar_points
/imu/data
/odom
/ground_truth/odom
/scurm/localization_error/position
/scurm/localization_error/yaw
/coverage/status
/coverage/progress
```

运行日志位于：

```text
${XDG_RUNTIME_DIR:-/tmp}/linorobot2-${UID}/gazebo.log
${XDG_RUNTIME_DIR:-/tmp}/linorobot2-${UID}/navigation.log
${XDG_RUNTIME_DIR:-/tmp}/linorobot2-${UID}/exploration.log
${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}/arena.log
```

快速查看：

```bash
runtime_dir="${XDG_RUNTIME_DIR:-/tmp}/linorobot2-${UID}"
tail -f "${runtime_dir}/navigation.log"
```

查看全部启动参数：

```bash
./start_linorobot.sh --help
```

### 图形驱动提示

当前 Intel/Mesa 环境中，RViz 启动时可能输出一次
`indexed_8bit_image ... active samplers`，Gazebo 也可能输出一次
`Deleting a connection right after creation`。两者都是图形界面/上游插件提示；当 RViz 显示 `Global Status: OK`，且机器人、地图和局部代价地图可见时，不表示 RobotModel 或 TF 故障。

SCURM 启动脚本会等待 FAST-LIO 初始化后再激活 Nav2/RViz，并排除无关的 TurtleBot 模型目录。持续出现 `Invalid frame ID`、`Missing model.config` 或 RobotModel 红色状态才应按真实故障检查对应日志。
