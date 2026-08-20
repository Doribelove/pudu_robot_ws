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

Arena4 已保存在 `external/arena4_ws`，包含 498 个 ROS 源码包和自己的 install/Python 环境。统一入口每次选择独立 ROS Domain，并使用 Fast DDS 和 Gazebo Harmonic，与 PUDU/Linorobot 的 Domain `42` 以及上次异常退出遗留的 DDS 服务隔离；在新终端 `source setup_arena4_runtime.bash` 会自动加入当前 Arena 实例的 Domain。

```bash
cd /home/robot/pudu_robot_ws

# 环境与运行状态检查
./start_arena4.sh --check
./start_arena4.sh --status

# 默认 Jackal + TEB，只显示 RViz；Gazebo 作为后台仿真器运行
./start_arena4.sh
./start_arena4.sh --scenario
./start_arena4.sh --headless --scenario

# hospital + TurtleBot + DWB + NavFn，并自动连续探索
./start_arena4.sh \
  robot:=turtlebot \
  world:=hospital \
  local_planner:=dwb \
  global_planner:=navfn \
  tm_robots:=explore

# 同时指定 Nav2 控制器、全局规划器和行为树配置
./start_arena4.sh robot:=turtlebot world:=hospital \
  local_planner:=mppi \
  global_planner:=smac_2d \
  inter_planner:=navigate_w_replanning_time

# Jackal 差速底盘 + TEB + NavFn（TEB 使用 Nav2 Costmap2D，不依赖旧 converter）
./start_arena4.sh --scenario robot:=jackal world:=hospital \
  local_planner:=teb global_planner:=navfn \
  inter_planner:=navigate_w_replanning_time tm_robots:=guided

# 固定场景、无界面运行；等价的任务参数见下文
./start_arena4.sh --headless --scenario robot:=jackal world:=factory

# 启动完成后的只读基线检查
./verify_arena4_baseline.sh --robot jackal

# guided 空闲状态下做短距离底盘运动检查（会实际移动车辆）
./verify_arena4_baseline.sh --robot jackal --move

# 只停止由 start_arena4.sh 启动的实例
./stop_arena4.sh
```

在额外终端操作 Arena ROS 图：

```bash
source /home/robot/pudu_robot_ws/setup_arena4_runtime.bash
ros2 node list
ros2 launch arena_bringup arena.launch.py --show-args
```

Arena4 参数采用 ROS 2 launch 的 `名称:=值` 语法，可以按需组合：

| 参数 | 含义 | 默认值 | 当前可选值 |
| --- | --- | --- | --- |
| `world` | 地图/仿真环境 | `map_empty` | `factory`、`hospital`、`ignc`、`map_empty`、`house17`、`generated`、`.generated` |
| `robot` | 机器人模型 | `jackal` | `WLP311D`、`WLP311E`、`boxer`、`dingo`、`husky`、`jackal`、`rbkairos`、`rbrobout`、`rbsummit`、`rbtheron`、`rbvogui`、`ridgeback`、`rskomnidirectional`、`turtlebot` |
| `local_planner` | Nav2 控制器/局部规划配置 | `teb` | `crowdnav`、`crowdnav_attngraph`、`drlvo`、`dwb`、`graceful`、`mppi`、`regulated_pure_pursuit`、`rotation_shim`、`sicnav`、`teb` |
| `global_planner` | Nav2 全局规划配置 | `navfn` | `navfn`、`smac_2d`、`smac_hybrid`、`smac_state_lattice`、`theta_star` |
| `inter_planner` | Nav2 行为树配置 | `navigate_w_replanning_time` | 用下面的 `--show-args` 查询完整列表 |
| `tm_robots` | 机器人任务生成模式 | `explore` | `guided`、`explore`、`random`、`scenario` |
| `tm_obstacles` | 障碍物/行人生成模式 | 见下文 | `parametrized`、`random`、`scenario`、`environment` |
| `headless` | 界面模式 | 统一入口默认 `1` | `-1`、`0`、`1`、`2`；`1` 仅 RViz，`2` 为完全无界面，`0` 显示 Gazebo 和 RViz |
| `env_n` / `env_d` | 并行环境数/间距 | `1` / `50` | 正整数 / 距离值 |

其中 `local_planner` 是跟踪路径并输出速度的控制器，`global_planner` 负责生成全局路径，`inter_planner` 选择 Nav2 的行为树/重规划策略。参数名虽然保留了 Arena 的旧称，但这里加载的是 `configs/nav2` 下的配置；`teb` 是当前工作区新增的可选 Nav2 Controller，`dwa` 仍不是有效选项。

TEB 的依赖和运行方式：g2o 安装在 `external/arena4_ws/third_party/g2o-install`，由
`setup_arena4_runtime.bash` 和 `start_arena4.sh` 自动加入运行库路径。控制器直接从
Nav2 的实时 `Costmap2D` 提取致命占据栅格，未加载 ROS1 风格的 `costmap_converter` 插件；
Jackal 按四轮滑移转向（skid-steer）建模，使用两圆 footprint；TEB 与 velocity smoother 统一为前进 1.2 m/s、后退 0.4 m/s、角速度 1.8 rad/s，线/角加速度 2.5 m/s² 和 2.0 rad/s²。全局路径前视和碰撞硬检查均为 2.5 m（最高速度下约 2.1 s）。TEB 用 via-point 遵循 NavFn 路线，同时允许障碍代价局部胜出并在绕障后回线；首段路径向后时可直接生成受控倒车轨迹。

`smac_hybrid` 和 `smac_state_lattice` 带运动学/转弯半径约束，在医院狭窄区域或随机起点紧邻障碍时可能拒绝路径；栅格地图优先从 `navfn` 或 `smac_2d` 开始。仅想启动环境后手动在 RViz 下发目标时，可加 `tm_robots:=guided`；此模式会等待目标，启动后不自动运动。默认 `explore` 会自动采样起点和目标。统一入口会让 Gazebo 相机持续跟随机器人，并且只有在模型已生成且 Nav2 的 controller、planner、bt_navigator 全部进入 `active` 后才报告成功。

`scenario` 不是第二个 Gazebo 世界，而是当前 `world` 对应的机器人起终点、静态障碍和行人任务层。`factory`、`hospital`、`ignc` 已自带完整 SDF 家具，因此未显式指定 `tm_obstacles` 时会自动使用各自的 `scenarios/default.json`，不会再在医院里随机叠加通用货架。需要压力测试时仍可明确写 `tm_obstacles:=random`；需要完全复现固定起点、目标和行人时使用：

```bash
./start_arena4.sh \
  robot:=turtlebot world:=hospital \
  local_planner:=dwb global_planner:=navfn \
  tm_robots:=scenario tm_obstacles:=scenario
```

hospital 没有单独维护 `walls.yaml`，运行时会从占据栅格提取并简化墙体轮廓交给 HuNav，Gazebo 则继续使用原 SDF 的物理墙；这样行人会避开医院墙和障碍，同时不会复制出第二套可视/碰撞墙。每次 launch 还会生成独立的 `GZ_PARTITION`，旧的或其他终端启动的 Gazebo 世界不会混入当前实例。

随源码安装的完整、实时选项由启动文件从配置目录自动发现，并会校验拼写：

```bash
source /home/robot/pudu_robot_ws/setup_arena4_runtime.bash
ros2 launch arena_bringup arena.launch.py --show-args
```

若使用 Arena 原生命令，写法与统一入口一致，但要先加载隔离环境：

```bash
cd /home/robot/arena4_ws
source arena.bash
ros2 launch arena_bringup arena.launch.py \
  sim:=gazebo robot:=turtlebot world:=hospital \
  local_planner:=mppi global_planner:=smac_2d \
  inter_planner:=navigate_w_replanning_time
```

`factory`、`hospital`、`ignc` 带独立 Gazebo 世界模型；`map_empty`、`house17`、`generated` 目前使用 Arena 的空白 Gazebo 世界，再加载各自地图与任务数据。hospital 有数百个碰撞网格，ODE 物理频率已从原来的 1 kHz 调整到 20 Hz，让仿真时间接近真实时间；Nav2、激光和 HuNav 仍按各自更新率工作。统一入口会等待首次 `Task Reset!`、Gazebo 机器人实体和 Nav2 核心节点全部就绪后才报告启动成功，最长约 90 秒。

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
