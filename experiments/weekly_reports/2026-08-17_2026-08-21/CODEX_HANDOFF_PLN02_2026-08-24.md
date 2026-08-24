# PLN-02 Codex 项目交接

更新时间：2026-08-24  
工作区：`/home/robot/pudu_robot_ws`

## 1 交接目的

本文件用于在没有原聊天记录的情况下，让新的 Codex 准确接手 PLN-02 静态 A2B 全局规划课题。新 Codex 必须先读取本文件和项目约束，不得根据文件名或阶段编号推断任务已经全部完成。

## 2 必读文件

按顺序读取：

1. 课题范围约束：`/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/AGENTS.md`
2. 本交接文件：`/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/CODEX_HANDOFF_PLN02_2026-08-24.md`
3. 周报：`/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/weekly_report.md`
4. 课题完成情况：`/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/PLN-02_课题完成情况_2026-08-21.md`
5. 新 Codex 启动提示词：`/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/NEW_CODEX_START_PROMPT_PLN02.md`

## 3 唯一课题范围

只研究：在静态栅格地图上，输入起点、目标位姿、Jackal footprint、运动学参数和横向偏好，输出全局 A2B 参考路径并评价其性能、资源消耗、静态碰撞、运动学可行性和路径质量。

正式约束：

- 只使用静态障碍，`dynamic_obstacles=false`；
- 地图固定 `0.05 m/cell`，当前不实现多分辨率地图；
- Jackal footprint 为 `0.51 x 0.43 m` 矩形；
- 项目实验约束：禁止原地旋转；
- 硬最小转弯半径 `0.40 m`，最大曲率 `2.50 1/m`；
- 允许倒车，`reverse_penalty=2.0`；
- 正式运动学搜索使用 `REEDS_SHEPP`；
- 不研究动态障碍、TEB/MPPI/DWB 控制、`cmd_vel`、建图、定位、多机调度或真机运行。

注意：真实 Jackal 是差速/滑移转向平台，可以近似原地旋转。禁止原地旋转和 `0.40 m` 半径是本课题为了研究连续曲率全局路径施加的实验约束，不得描述为 Jackal 固有物理极限。

## 4 Git 和数据状态

### 开发分支

- 开发基线：`main`
- 当前主线提交：`21dcaee Add static layered planner benchmarks through stage 8`
- GitHub：`https://github.com/Doribelove/pudu_robot_ws`

### 报告分支

- 报告分支：`report/2026-08-17_2026-08-21`
- 已上传周报和课题完成情况；报告分支不应用于继续开发算法。
- 新 Codex 若要修改源码，应先确认工作区状态，再切换到 `main`；不得把报告分支的资料误当作算法新增内容。

### 特殊路径

根仓库 `.gitignore` 会忽略 `experiments/`，部分 `external/` 内容也可能不会在根仓库普通 `git status` 中显示。不得因为 `git status` 为空就认定实验数据或 Arena evaluation 源码不存在。提交报告资料时需要限定目标并使用 `git add -f`，不得强制添加整个 `experiments/`。

## 5 当前系统架构

```text
静态栅格地图 + 起点/目标 + footprint + 运动学参数 + 横向偏好
                            |
                            v
L1 拓扑层：选择房间/走廊/门口，输出拓扑边和粗走廊
                            |
                            v
L2 栅格层：走廊内 A*；失败时扩大走廊或回退全图 A*
                            |
                            v
运动学违规检测：急转弯、航向跳变、曲率和拼接检查
                            |
                            v
L3 运动学层：只对违规窗口执行局部 Hybrid 修复
                            |
                            v
静态 footprint + 硬半径 + 连续性验收
                            |
                            v
路径、有效性、来源标记、回退记录和失败原因码
```

预定回退顺序：

```text
拓扑搜索
  -> 走廊内栅格 A*
  -> 扩大走廊
  -> 全图栅格 A*
  -> 局部运动学修复
  -> 结构化失败原因码
```

## 6 当前完成状态

| 工作项 | 状态 | 当前结论 |
|---|---|---|
| Hospital 0.05 m 基线 | 已完成 | NavFn/Smac 四组基线，每组 50 条 measured |
| 评测框架 | 已完成阶段性实现 | 能统计成功率、耗时、CPU、RSS/PSS、路径质量、碰撞和失败码 |
| L1 拓扑层 | 已完成阶段性实现 | 已构建和持久化拓扑图，并测量预计算开销 |
| L2 栅格层 | 已完成 | 支持全图 A*、走廊 A*、走廊扩张和全图回退 |
| Stage 7 L3 | 仅历史消融 | 允许原地旋转，不得作为正式车辆约束结论 |
| Stage 8A L3 | 已完成阶段性实现 | 禁止原地旋转，局部 Hybrid 修复并执行硬约束验收 |
| Stage 8B 横向偏好 | 已完成阶段性实现 | 完成靠中和靠右权重扫描 |
| 统一端到端规划器 | 未完成 | 当前仍是阶段性模块和 CLI，尚未完全统一 |
| 同进程端到端计时 | 未完成 | 当前组合耗时是估算，不是正式实测 |
| 多地图/多规模曲线 | 未完成 | 当前正式结论主要来自单张 Hospital 地图 |
| RRT*/Kinodynamic-RRT | 未完成 | 尚未接入和评测 |
| 完整降级链验收 | 部分完成 | 已验证走廊扩张和全图回退，尚未覆盖全部分支 |
| 结题报告 | 未完成 | 当前只有阶段周报和数据归档 |

## 7 核心实验数据

### 7.1 Hospital 和 query

- 地图：`1600 x 1600`，约 `80 x 80 m`，`0.05 m/cell`，共 256 万栅格；
- 固定 10 对 query；
- 随机种子：`20260821`；
- 每组：10 cold、30 warmup、50 measured。

输入文件：

- `/home/robot/pudu_robot_ws/experiments/maps/hospital_005/map.yaml`
- `/home/robot/pudu_robot_ws/experiments/planner_benchmark/hospital_005/queries_v2.yaml`
- `/home/robot/pudu_robot_ws/experiments/planner_benchmark/hospital_005/protocol_v2.yaml`

### 7.2 Stage 5 基线

| 基线 | action success | static footprint valid | final valid | Planning P50/P95/P99 |
|---|---:|---:|---:|---:|
| NavFn Product | 50/50 | 50/50 | 50/50 | 10.156 / 12.066 / 13.934 ms |
| NavFn Normalized | 50/50 | 50/50 | 50/50 | 9.130 / 11.511 / 12.331 ms |
| Smac Product | 40/50 | 35/50 | 35/50 | 29.805 / 143.482 / 145.027 ms |
| Smac Normalized | 50/50 | 40/50 | 40/50 | 22.415 / 101.101 / 117.800 ms |

解释：

- NavFn 是二维 A* 基线；
- Smac Hybrid 搜索位置和航向，计算成本更高；
- Product/Normalized 是本项目的参数变体，不是 ROS 标准算法名称；
- Smac Product 使用 DUBIN，Smac Normalized 使用 REEDS_SHEPP；
- Stage 5 是早期基线，协议仍有 `allow_in_place_rotation=true`，不得用于正式禁止原地旋转的硬约束结论；
- `action_success` 不等于 `final_valid_success`。

目录：`/home/robot/pudu_robot_ws/experiments/planner_benchmark/hospital_005/`

### 7.3 Stage 6 L1/L2

- `full_grid`：9/10 query 成功；
- `topology_guided_grid`：6/10；
- `topology_guided_grid_fallback`：9/10；
- 纯拓扑走廊相对全图平均减少约 `42.55%` 展开节点；
- 纯拓扑走廊在线 speedup `1.255x`；
- 带回退 speedup `0.608x`，即整体约慢 `1.64x`；
- 拓扑引导路径长度/full-grid：均值 `1.0042`，P95 `1.0145`；
- 带回退路径长度/full-grid：均值 `1.0175`，P95 `1.1321`。

拓扑预计算：

- wall `7763.684 ms`；
- CPU `8588.354 ms`；
- peak RSS `251772928 bytes`；
- 224 节点、195 边、35 连通分量；
- 文件 `879577 bytes`；
- 估算 break-even 约 90 queries，但仅适用于纯走廊成功模式。

目录：`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/hospital_005/stage6_l1_l2/`

### 7.4 Stage 8A 正式硬约束 L3

- 50 条候选，最终有效 `35/50=70%`；
- 同静态模型可达记录 `35/45=77.78%`；
- query 级 `7/9`；
- 100 次局部 Hybrid action 都返回成功，但只有 75 次通过静态和运动学复核；
- 成功路径原地旋转 0、静态碰撞 0、硬半径违规 0；
- 最小观测半径 `0.4397 m > 0.40 m`；
- 最大曲率 `2.2744 1/m < 2.50 1/m`；
- 最大拼接位置误差 `0.0189 m`；
- 最大拼接 yaw 误差 `0.438°`；
- L3 planning P50/P95/P99：`18.874 / 36.509 / 37.658 ms`。

重要负结果：

- `q00`、`q08` 为 `KINEMATIC_REPAIR_FAILED`；
- `q04` 为 `STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH`，未进入 L3；
- 组合在线时间是 Stage 6 加 L3 的估算，平均约为 full Smac 的 `22.79x`；这是“耗时比”，不是加速倍数；
- 当前不能声称分层方案端到端更快。

目录：`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/hospital_005/stage8a_hard_radius_l3_v2/`

### 7.5 Stage 8B 横向偏好

选定权重：

```yaml
center: 1.0
right_edge: 1.0
```

- Center：偏差 `0.6551 -> 0.0778 m`，改善 `88.1%`；路径均值增加 `3.85%`；展开节点增加 `12.4%`；在线时间增加 `25.2%`；
- Right-edge：侧墙误差 `1.0268 -> 0.2400 m`，改善 `76.6%`；正确侧比例 `80.9%`；路径均值增加 `5.95%`；展开节点增加 `18.1%`；在线时间增加 `24.0%`；
- 两种偏好最终有效均为 `35/50`，成功路径无碰撞和硬半径违规。

结论：横向偏好确实有效，但以路径长度和计算量为代价，没有带来性能收益。

目录：`/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/hospital_005/stage8b_lateral_preference_v2/`

## 8 代码入口

主要实现：

- L1/L2：`external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/topology.py`
- L1/L2 CLI：`external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/topology_cli.py`
- Stage 7 旧运动学层：`kinematic.py`、`kinematic_cli.py`
- Stage 8A：`stage8.py`、`stage8_cli.py`
- Stage 8B：`preference.py`、`preference_cli.py`
- 报告：`stage8_report.py`
- 三层 RViz 可视化：`layered_visualizer.py`
- CLI 注册：`external/arena4_ws/src/arena/evaluation/arena_evaluation/setup.py`

已注册命令包括：

```text
topology_benchmark
kinematic_benchmark
stage8a_hard_radius_l3
stage8b_lateral_preference
stage8_report
```

## 9 已覆盖和未覆盖的任务书指标

| 指标 | 状态 |
|---|---|
| 碰撞路径数 | 已测，成功路径为 0 |
| 运动学不可行段数 | 已测，成功路径为 0 |
| 规划成功率 | 已测并区分 action/static/final |
| planning/wall P50/P95/P99 | 已测，主要是单地图 |
| CPU 时间 | 已测，短请求受约 10 ms tick 粒度限制 |
| RSS/PSS | 已测，尚无地图规模曲线 |
| 拓扑预计算 | 已测 |
| 路径长度/曲率/净空 | 已测 |
| 横向偏好满足度 | 已测 |
| L1/L2 消融 | 已测 |
| L3 正式硬约束 | 已测 |
| 统一端到端分层增益 | 未完成 |
| 完整降级链 | 部分完成 |
| 多地图/多规模曲线 | 未完成 |
| RRT*/Kinodynamic-RRT | 未完成 |
| 跨环境复现实验 | 未完成，仅具备固定协议和数据归档 |

## 10 新 Codex 下一步优先级

必须先向用户确认本次要继续哪个目标，不得自动启动大型实验。若用户要求继续开发，推荐顺序：

1. 统一 L1/L2/L3 的输入输出、路径来源标记、失败码和回退记录；
2. 在同一进程和同一次请求中测量 L1、L2、L3 分段时间、总 wall/CPU 和峰值内存；
3. 分析 `q00`、`q08` 的局部修复失败，不放宽禁止原地旋转和 `R_min=0.40 m`；
4. 明确 `q04` 的保守静态语义与 Nav2 costmap 语义差异，不可静默改成成功；
5. 验证完整降级链和每个结构化失败码；
6. 生成静态合成地图集：长走廊、多房间、多门连接、大厅、迷宫、随机静态障碍、多连通区域，并形成规模曲线；
7. RRT*/Kinodynamic-RRT 是否必须完成，应先让用户与导师确认优先级；
8. 多分辨率地图当前被 `AGENTS.md` 和用户决定排除，不得自行恢复。

## 11 验证命令

在执行前先确认当前分支和工作区：

```bash
cd /home/robot/pudu_robot_ws
git status --short
git branch --show-current
```

开发应基于 `main`。切换前必须确保没有未提交的用户修改：

```bash
git switch main
source ./setup_arena4_runtime.bash
```

只做帮助和测试检查时可使用：

```bash
ros2 run arena_evaluation topology_benchmark --help
ros2 run arena_evaluation stage8a_hard_radius_l3 --help
ros2 run arena_evaluation stage8b_lateral_preference --help

cd /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation
pytest -q
```

历史 Stage 8 验证结果为 `81 passed`，但新 Codex 必须根据当前代码实际重新验证，不能只引用历史结果。

## 12 禁止误报

不得声称：

- 任务书已全部完成；
- 三层组合已经证明端到端加速；
- Stage 7 是正式车辆约束结果；
- Stage 5 Smac action 成功等于路径有效；
- `22.79x` 是加速；
- Hospital 单地图已经证明超大地图规模规律；
- 0.1/0.05 对照是多分辨率算法；
- `0.40 m` 是 Jackal 固有物理最小转弯半径；
- 完整 Arena4/Gazebo/TEB 运行时间是全局规划时间。

## 13 当前报告和云端位置

- GitHub 报告分支：`https://github.com/Doribelove/pudu_robot_ws/tree/report/2026-08-17_2026-08-21`
- 周报：`experiments/weekly_reports/2026-08-17_2026-08-21/weekly_report.md`
- 课题完成情况：`experiments/weekly_reports/2026-08-17_2026-08-21/PLN-02_课题完成情况_2026-08-21.md`
- Word 周报：`experiments/weekly_reports/2026-08-17_2026-08-21/PLN-02_A2B周报_李永祺_2026-08-17至2026-08-21.docx`
- 精选图表：`experiments/weekly_reports/2026-08-17_2026-08-21/report_assets/figures/`
- 完整数据：`experiments/weekly_reports/2026-08-17_2026-08-21/data/`

