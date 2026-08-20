# Arena4 TEB Controller 集成说明

## 组件

- 源码：`external/arena4_ws/src/nav2_teb_controller`
- Nav2 配置：`external/arena4_ws/src/arena/simulation-setup/configs/nav2/controllers/teb/controller_config.yaml`
- 本地 g2o：`external/arena4_ws/third_party/g2o-install`
- 运行时入口：`setup_arena4_runtime.bash`、`start_arena4.sh`

TEB 已作为 `nav2_core::Controller` 插件注册，启动参数名为 `local_planner:=teb`。
Jackal 按四轮滑移转向（skid-steer）建模：控制空间仍是非全向的 `(vx, wz)`，不输出横向速度，
但用有限权重表达轮胎侧滑，不再套用 Ackermann 的转向角、转向速率和 G3 转向连续性约束。
前进/后退/角速度上限分别为 `1.2 m/s`、`0.4 m/s`、`1.8 rad/s`，线/角加速度上限分别为
`2.5 m/s²` 和 `2.0 rad/s²`。两圆 footprint 保守覆盖 `0.51 x 0.43 m` 矩形，消除了旧四圆模型
在车宽方向产生的过大虚拟外形。

全局路径截取和轨迹碰撞硬检查统一为 `2.5 m`。最高前进速度下对应约 `2.1 s` 的空间窗口，
既保留医院走廊内的即时绕障余量，也避免过长路径把远处转弯提前拉入当前优化。

`velocity_smoother` 使用完全相同的速度、加速度和减速度边界，Recovery Server 的旋转速度与
角加速度也同步为 `1.8 rad/s`、`2.0 rad/s²`，避免 TEB 输出在下一环节被另一套限制重新塑形。
控制器也实现 Nav2 的动态限速接口：百分比或绝对线速度限值会同比缩放线速度、倒车速度和角速度。
Arena4 原有的 `recoveries_server/nav2_recoveries` 配置属于旧 Nav2 接口，实际运行节点不会读取；
现已迁移为 `behavior_server/nav2_behaviors`，上述恢复限速会真正进入运行态。

## 全局路径跟随

TEB 每个中间位姿都通过 `EdgeViaPoint` 约束到当前 NavFn 折线的最近投影，Arena4 配置使用
`weight_viapoint: 30.0`。因此无障碍直线路径会保持直行；障碍物代价可以在局部胜出，绕障后
via-point 会把轨迹重新拉回全局线。
对于终点不变但路线发生变化的重规划，新旧路径几何偏差超过 `reinit_path_dist: 0.35` 时会清除
旧 band 并从新全局路径初始化，避免旧弯路被 warm start 长期保留。

当全局路径第一段位于车体后方时，TEB 会按首段方向初始化倒车轨迹；即使局部路径终点最终又
回到车体前方，也不会误判成必须先原地掉头。前进偏置保留为较小权重 `5.0`，用于避免无意义
倒车，但不会压制从死角倒出的合理动作。

## 障碍物接口

控制器不再加载 ROS1 风格的 `costmap_converter` 插件，也不要求其动态库存在。每个控制周期
直接锁定并读取 Nav2 `Costmap2D` 的 `LETHAL_OBSTACLE` 栅格，转换为 TEB 点障碍物；同时用同一
张 costmap 更新 ESDF。`costmap_converter_msgs` 仅保留给当前未启用的 HCP 兼容占位接口，正常
Jackal 路径不创建或订阅 converter 节点。

## 构建

无 sudo 权限时使用工作区本地 g2o：

```bash
cd /home/robot/pudu_robot_ws/external/arena4_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$(pwd)/third_party/g2o-install:${CMAKE_PREFIX_PATH:-}"
./colcon_build --packages-select nav2_teb_controller arena_simulation_setup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

`start_arena4.sh` 会在后台 launch 子进程中自动导出 g2o 的 `LD_LIBRARY_PATH`，不需要手工设置。

## 启动与验证

```bash
cd /home/robot/pudu_robot_ws
./start_arena4.sh --scenario --headless \
  world:=hospital robot:=jackal local_planner:=teb global_planner:=navfn \
  inter_planner:=navigate_w_replanning_time tm_robots:=guided
```

另开终端加载同一个 ROS Domain 后发送目标：

```bash
source /home/robot/pudu_robot_ws/setup_arena4_runtime.bash
ros2 action send_goal /task_generator_node/jackal/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 15.45}, orientation: {w: 1.0}}}}'
```

启动脚本成功返回表示 Gazebo 模型、Nav2 controller/planner/BT lifecycle 和 map/base TF 均已就绪。
控制器日志中应出现 `TEBController activated`，不应出现 `Failed to load library` 或
`costmap converter plugin`。

## 已知边界

当前候选仓库的 Homotopy Class Planner 仍是占位实现，配置中保持关闭；TEB 采用直接单带优化。
行为树若以很短周期反复重发等价路径，控制器会保留 warm start；目标或路径几何显著变化时清空轨迹。
全局路径是否可达仍由 NavFn/Smac2D 决定，TEB 只负责差速底盘的局部可执行跟踪。
