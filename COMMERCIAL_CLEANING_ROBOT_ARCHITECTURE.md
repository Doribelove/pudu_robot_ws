# 商业清扫机器人整体架构与学习路线

> 面向普度机器人商业清扫车业务线，重点覆盖导航、感知、定位、控制、覆盖清扫与多机调度。
>
> 整理日期：2026-08-17

## 1. 总体结论

针对商场、写字楼、酒店、医院、工厂等室内商业清扫场景，建议：

**以 ROS 2 Nav2 为导航骨架，围绕覆盖清扫、稳定定位、2.5D 感知和独立安全控制做产品化扩展。**

不建议一开始把重点放在无人车式重型架构或纯端到端学习架构上。商业清扫机器人更看重稳定性、可解释性、可维护性、作业完成率和故障恢复能力。

## 2. 开源项目优先级

| 优先级 | 项目 | 重点学习内容 | 商业清扫车价值 |
|---:|---|---|---|
| 1 | [Navigation2 / Nav2](https://github.com/ros-navigation/navigation2) | Planner/Controller Server、Costmap、行为树、Lifecycle、恢复、禁行区、限速区、回充 | 最接近商业移动机器人导航骨架 |
| 2 | [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) + [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | 2D 激光建图、位姿图、轮速与 IMU 融合、地图保存和定位 | 室内清扫定位的基础组合 |
| 3 | [OpenNav Coverage](https://github.com/open-navigation/opennav_coverage) + [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) | 区域分解、弓字形清扫、覆盖宽度、重叠率、转弯、遗漏区域 | 最贴近清扫业务核心 |
| 4 | Nav2 [Smac Planner](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html) + [MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html) | 非圆形车体规划、运动学约束、倒车、窄通道和动态避障 | 适合矩形、尺寸较大的洗地机 |
| 5 | [ros2_control](https://control.ros.org/master/doc/ros2_control/doc/index.html) | 底盘硬件接口、差速/阿克曼控制、限速、加速度和急停接口 | 理解导航到电机之间的控制链 |
| 6 | [Linorobot2](https://github.com/linorobot/linorobot2) | Nav2、SLAM Toolbox、EKF、URDF、Gazebo 和底盘完整集成 | 最适合快速看懂 ROS 2 移动机器人全栈 |
| 7 | [RTAB-Map ROS 2](https://github.com/introlab/rtabmap_ros) 或 [Isaac ROS Nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) | 深度相机、三维障碍、悬空物、桌腿和低矮障碍 | 补充 2D 激光雷达盲区 |
| 8 | [Open-RMF](https://github.com/open-rmf/rmf) | 多机调度、楼层、电梯、门、充电和任务分配 | 面向大客户与多机器人部署 |

## 3. 第一优先级：Nav2

Nav2 不是单一算法，而是一套适合产品化拆分的导航框架：

- Behavior Tree：组织清扫、回充、补水、排污、恢复和人工接管。
- Planner Server：负责全局路径和区域间转场。
- Controller Server：负责局部避障和轨迹跟踪。
- Costmap：融合激光、深度相机、禁行区与动态障碍物。
- Lifecycle：控制模块启动、停止和故障恢复。
- Docking Server：用于充电、加水或排污站对接。
- Collision Monitor：提供导航软件层的碰撞监控。
- Keepout/Speed Filter：处理玻璃门、扶梯、危险区域和人员密集区的禁行与限速。

建议重点阅读：

1. Costmap 2D 及各种 Layer。
2. Behavior Tree 与恢复流程。
3. Smac Planner。
4. Regulated Pure Pursuit 和 MPPI。
5. Collision Monitor。
6. Docking Server。
7. Keepout Filter 和 Speed Filter。

Nav2 的[架构说明](https://docs.nav2.org/concepts/)将规划器、控制器、行为和环境表达设计成插件，适合商业产品保留框架并替换内部算法。

## 4. 第二优先级：定位架构

商业清扫车不建议只运行一个“大一统 SLAM 节点”。更稳定的设计通常分为三层：

```text
轮速计 + IMU
      ↓
robot_localization EKF
      ↓
连续平滑的 odom → base_link
      ↓
2D 激光定位 / AMCL / Scan Matching
      ↓
稳定的 map → odom
```

推荐的产品思路：

- 建图阶段：使用 SLAM Toolbox 生成地图并进行人工审核。
- 运行阶段：在版本化的静态地图上定位。
- 动态货架、行人和手推车：进入局部 Costmap，不轻易写入永久地图。
- 定位失败：触发重定位、回退、原地观测或远程协助。
- 玻璃、大面积空旷和长走廊：使用视觉、反光板、AprilTag 或场景特征辅助。
- 重点建设地图版本管理、错误闭环检测、地图污染防护和定位置信度监控。

## 5. 第三优先级：覆盖清扫

清扫机器人与普通送餐机器人的最大区别不是点到点导航，而是**完整、可验证的区域覆盖**。

[OpenNav Coverage](https://github.com/open-navigation/opennav_coverage)已经提供：

- 输入多边形区域和内部孔洞。
- 生成弓字形、蛇形等覆盖路线。
- 设置机器宽度和实际清扫宽度。
- 设置路径重叠率。
- 支持 Dubins 和 Reeds-Shepp 转弯连接。
- 通过 Nav2 行为树执行覆盖任务。

商业清扫产品还需要扩展：

- 沿墙清扫及内外边界清扫。
- 考虑清扫盘偏置，而不是只规划车体中心线。
- 处理柱子、货架岛和临时封闭区。
- 动态绕障后的断点续扫。
- 已清扫、未清扫、重复清扫和不可达区域记录。
- 脏污程度对应的速度、水量和刷盘压力控制。
- 区分清扫作业路径和关闭清扫头的转场路径。
- 电量、水量和污水箱容量约束。
- 多区域任务排序和跨楼层任务。

清扫业务建议维护独立的作业状态图：

```text
静态地图：墙、固定设施
导航代价地图：实时障碍、膨胀区、禁行区
覆盖状态图：已扫、漏扫、重复、不可达
脏污语义图：污渍、垃圾、重点区域
```

## 6. 第四优先级：规划与控制组合

对于体积较大的矩形清扫车，建议从下面的组合开始：

- 全局及转场规划：`Smac State Lattice`。
- 覆盖路径跟踪：`Regulated Pure Pursuit`。
- 动态复杂区域：`MPPI`。
- 车体模型：使用实际多边形 Footprint，避免仅使用圆形半径。
- 底盘输出：Velocity Smoother → 安全限速 → 底盘控制器。

控制器学习优先级：

1. **Regulated Pure Pursuit**：行为容易解释，适合低速直线覆盖。
2. **MPPI**：适合动态障碍、窄空间和复杂局部行为。
3. **自研 MPC**：当清扫质量对横向误差、曲率和刷盘轨迹有严格要求时再做。
4. **DWB/TEB**：可以了解，但不作为新产品主要学习方向。

对于非圆形清扫车，应使用真实多边形轮廓进行路径和轨迹碰撞检查。Smac Lattice 支持差速、全向、阿克曼和自定义运动模型，也能处理倒车及紧凑空间。

## 7. 第五优先级：立体感知

室内清扫车不能只依赖一条 2D 激光扫描平面，需要重点处理：

- 桌面、悬空柜体和突出货架。
- 桌椅腿、低矮托盘及地面小物体。
- 玻璃门、镜面和黑色吸光物体。
- 台阶、扶梯、沟槽和落差。
- 行人、宠物和购物车等动态目标。
- 水渍、线缆和布料等清扫风险。

推荐工程路线：

```text
2D 安全激光 → 主定位 + 基础障碍
深度相机/3D 激光 → 立体障碍 + 低矮障碍
语义模型 → 人、玻璃、线缆、垃圾和污渍
                   ↓
         统一投影到 2.5D 局部代价地图
```

使用 NVIDIA Jetson 时可以研究 Nvblox：从深度相机或 3D 激光实时重建三维环境，并向 Nav2 输出二维 Costmap。没有 NVIDIA 平台时，优先研究 RTAB-Map、PCL 和 Nav2 Voxel Layer。

## 8. 多机调度与场所系统

Open-RMF 适合用于学习：

- 多机器人任务分配。
- 交通冲突协调。
- 电梯和自动门对接。
- 充电任务调度。
- 多楼层导航图。
- 异构机器人系统接入。

它更适合作为机器人本体导航上方的场所级调度层，不建议直接替代机器人本体的实时导航和作业管理。

## 9. 推荐的产品级整体架构

```text
云端/运营平台
任务下发、地图版本、数据回传、远程诊断、多机调度
                         ↓
任务与作业层
区域选择 → 沿边 → 覆盖 → 补扫 → 回充/补水 → 故障恢复
                         ↓
导航层 Nav2
BT Navigator → Coverage/Planner → Controller → Velocity Smoother
                         ↑
定位层                 环境表达层
轮速+IMU EKF           静态地图
2D激光定位             动态 Costmap
视觉/标签辅助           3D 障碍和语义层
                         ↓
独立安全监督层
安全激光、碰撞条、悬崖传感器、急停、速度限制、健康监控
                         ↓
实时底盘与清洁控制器 MCU
电机闭环、制动、刷盘、吸水、水泵、阀门、电池和故障保护
```

重要原则：**Nav2 Collision Monitor 不能替代经过验证的硬件安全链。** 急停、防跌落、防撞条和安全激光限速应当能够绕过上层 ROS，在计算机死机或通信异常时仍然停车。

## 10. 推荐学习与实践顺序

建议完成一个可运行的“最小商业清扫车”原型：

1. 用 Linorobot2 看懂 TF、URDF、EKF、SLAM 与 Nav2 如何连接。
2. 在 Gazebo 中建立一台矩形差速清扫车。
3. 配置 SLAM Toolbox 建图和 AMCL 定位。
4. 使用 Nav2 完成点到点导航、禁行区和动态避障。
5. 接入 OpenNav Coverage，实现单个房间的弓字形覆盖。
6. 增加“沿边—覆盖—绕障—断点续扫—补扫”行为树。
7. 配置 Smac Lattice + RPP/MPPI。
8. 加入深度相机，处理悬空和低矮障碍。
9. 加入自动回充、补水和排污流程。
10. 最后研究 Open-RMF 多机调度。

## 11. ROS 2 版本建议

当前开发机为 Ubuntu 22.04，适合直接使用 ROS 2 Humble 学习和原型验证。

- Ubuntu 22.04：ROS 2 Humble。
- Humble 官方支持截止到 2027 年 5 月。
- 新产品设计应同步关注更新的长期支持版本，并避免让新增业务代码与某个 ROS 发行版强耦合。
- 自研模块尽量通过标准消息、Action、Service 和 pluginlib 接口与 Nav2 解耦。

## 12. 商用开发注意事项

- 开源项目适合作为参考和基础设施，不应未经评审直接作为产品安全方案。
- 导入前检查许可证、修改义务、第三方依赖及专利风险。
- 建立录包回放、仿真回归、硬件在环和现场数据闭环。
- 对定位丢失、传感器遮挡、网络中断、算力过载和进程崩溃进行故障注入。
- 指标不应只看导航成功率，还应包括覆盖率、重复率、漏扫率、人工接管率、平均恢复时间和单位面积耗时。

