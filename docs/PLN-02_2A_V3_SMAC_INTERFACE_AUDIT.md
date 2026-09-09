# PLN-02 2A-V3：pinned Smac 接口审计

日期：2026-09-08  
结论：pinned `SmacPlannerHybrid` 没有显式 SE(2) reference/guide 输入。因此 2A-V3 的 E5 是独立实验性 SE(2) planner adapter，不是给原生 Smac 增加二维 soft cost，也不能称为“原生 Smac 已支持语义 guide”。

## 1. 审计对象

- pinned Nav2 仓库：`external/arena4_ws/src/deps/nav2/navigation2`
- HEAD：`656ae8d4c56978efbdd446fe85582f2bcd06e920`
- 已有 dirty 文件：`nav2_bringup/launch/navigation_launch.py`；本任务未修改。
- `smac_planner_hybrid.hpp` SHA-256：`3ac5b7c2d6b557fbc0ec8dd950213b3156426b77cfb05c1b189f9c3aa6760934`
- `smac_planner_hybrid.cpp` SHA-256：`58026ec1e0d8d435940741d1e0280846eeb7d4f0fe3d5f7dce2d5ffa4f9d4cde`

## 2. 公开入口的真实能力

`nav2_core::GlobalPlanner::createPlan` 只接收 start 与 goal，配置阶段只注入 TF 和 `Costmap2DROS`：

- `nav2_core/include/nav2_core/global_planner.hpp:50-78`
- `nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid.hpp:59-87`

`SmacPlannerHybrid::createPlan` 的数据流为：

1. 从 `Costmap2DROS` 取得二维 master costmap；
2. 把 start/goal yaw 量化到内部 yaw bin；
3. 把 footprint 与二维 costmap 交给 collision checker；
4. 调用内部 `AStarAlgorithm<NodeHybrid>::createPath`；
5. 返回 `nav_msgs/Path`。

证据：`nav2_smac_planner/src/smac_planner_hybrid.cpp:273-385`。

可配置但不构成显式 guide 的参数包括：

- `angle_quantization_bins`：`smac_planner_hybrid.cpp:77-81`；
- `minimum_turning_radius`：`smac_planner_hybrid.cpp:102-104,169-170`；
- `motion_model_for_search`：`smac_planner_hybrid.cpp:139-148`；
- `cost_penalty`、解析扩展和通用搜索预算：`smac_planner_hybrid.cpp:93-137`。

## 3. 缺少的接口

公开 plugin 接口和本版本 `SmacPlannerHybrid` 均没有：

- per-state `(x,y,yaw_bin)` reference corridor；
- 每个 station 的 yaw-bin admissibility；
- reference-deviation term；
- lane instance/parking component phase；
- guide hash 或 route-station monotonicity输入。

所以二维 occupancy/costmap 能表达障碍和标量代价，但不能表达“同一个 `(x,y)` 在不同 yaw 下有不同的 lane-relative 引导代价”。这与 2A-V2 r3 的离线二维 guide 能通过、在线 Hybrid 搜索不能兑现的观察一致。

## 4. 2A-V3 采用的边界

E5-r13 的边界是：

1. Nav2 负责生成并通过 `GetCostmap` 服务返回实际 effective master；
2. adapter 只有在 policy/source/expected/server hash、sequence 与 ROI 都完成 exact ACK 后才读取它；
3. 独立 48-bin、forward-only Dubins state-lattice 显式读取 route phase 和语义 guide；
4. 每条 primitive 用 effective master、完整 padded Jackal footprint、硬语义和路线 phase 检查；
5. 生成的 fresh path 经 canonical PathAudit 后，以真实 `nav_msgs/Path` 发布并作内容回显；
6. 任何 soft fallback 仍是 fresh SE(2) 搜索结果，但 `semantic_success_counted=false`。

该 adapter 没有修改 pinned Nav2 源码，没有提高 Smac 迭代预算，没有放宽倒车、原地旋转、最小转弯半径、最大曲率或碰撞约束。

## 5. 架构含义

2A-V3 是架构变化，不是 2A-V2 的一个 soft-cost 参数版本。它已经证明“显式 SE(2) guide 可以兑现 targeted 三条难例”，但当前只是研究级独立 planner adapter。把它注册成长期运行的 Nav2 GlobalPlanner plugin、做更大 query 集和生命周期/内存工程化，属于后续生产化工作。
