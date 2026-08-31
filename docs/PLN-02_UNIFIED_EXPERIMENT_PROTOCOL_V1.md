# PLN-02 统一实验指标与验收协议

版本：PLN-02-EXP-V1

状态：新实验冻结约束

适用项目：/home/robot/pudu_robot_ws

适用架构：3A-V0、2A-V0、3D-V0、2D-V0 及其 -rN 实现迭代

## 1. 目的与优先级

本文件统一不同架构、不同地图和不同算法迭代的实验条件、数据字段、统计口径和结果判定，防止只比较单个数字而忽略实验条件差异。

本文件只覆盖 A2B 全局路径规划，不覆盖建图、定位、动态控制、速度跟踪、真机行驶和多机调度。

约束优先级如下：

1. 本项目 AGENTS.md 和已冻结的 P0_EVALUATION_DEFINITION.md；
2. 本文件 PLN-02-EXP-V1；
3. 架构专属协议；
4. 实验脚本的默认值。

发现冲突时必须停止正式实验，在 manifest 中记录冲突并升级 protocol_version；不得按个人判断静默选择一个口径。

## 2. 实验身份与版本命名

每次实验必须同时记录：

    experiment_id: <唯一实验目录名>
    protocol_version: PLN-02-EXP-V1
    architecture_id: 3A-V0 | 2A-V0 | 3D-V0 | 2D-V0
    implementation_revision: rN
    experiment_kind: static_formal | static_smoke | dynamic_incremental | ablation | optimization_ab

架构 ID 与实现迭代号分离：

- 3A-V0：L1 Graph A* + L2 走廊 Grid A* + L3 局部 Smac Hybrid A*；
- 2A-V0：L1 Graph A* + L2 关闭 + L3' 全走廊 Smac Hybrid A*；
- 3D-V0：L1 Graph A* + L2 走廊 D* Lite + L3 Smac Hybrid A*；
- 2D-V0：L1 细化拓扑图 D* Lite + L2 关闭 + L3 全走廊 Smac Hybrid A*；
- bug 修复、缓存、参数和性能优化只增加实现迭代号，不得擅自改成新的架构版本。

默认实现解析：`2A-V0` 默认指当前已验证的修复实现（机器可读审计字段 `implementation_revision: r3`）。历史 `r1` 结果只读保留，用于追溯早期固定 2 m 走廊基线，不得作为新实验的默认代码或对照臂；后续修改从 `r4` 递增。

实验目录不得覆盖历史目录。推荐格式：

    <architecture_id>_<map_id>_<experiment_kind>_<date>_<revision>/

## 3. 可比性分组

只有下列字段全部一致的结果，才允许进入同一张严格对比表：

    map_id + map_sha256 + map_yaml_sha256
    query_set_id + query_sha256 + query_order_seed
    evaluation_resolution
    footprint_hash + kinematic_profile
    planner_parameters + timeout_policy
    software_commit + patch_hash + build_profile
    protocol_version + warmup/repetition policy
    cache_mode + session policy
    dynamic_obstacles

不同地图、不同分辨率、不同走廊 profile、不同重试预算或不同缓存状态必须分组报告，不得合并平均。

优化前后必须固定地图、query、起终点顺序、随机种子、机器、软件构建和超时策略；只允许改变被测优化项。

## 4. 输入冻结

### 4.1 地图

每张地图必须记录：

    map_id: mentor_map_20260825_005
    map_yaml: <path>
    map_image: <path>
    map_yaml_sha256: <sha256>
    map_image_sha256: <sha256>
    native_resolution_m: <float>
    evaluation_resolution_m: <float>
    width_cells: <int>
    height_cells: <int>
    area_m2: <float>
    free_ratio: <float>
    unknown_ratio: <float>
    occupied_ratio: <float>
    largest_free_component_ratio: <float>

当前架构比较默认使用 0.05 m/cell。如果按 P0_EVALUATION_DEFINITION.md 的 0.10 m/cell 评测，必须标记为独立 protocol variant；两种分辨率不可直接混入同一性能曲线。

未知区、地图外区域均按不可通行处理。不得由某个架构单独改变占据阈值或 allow_unknown。

### 4.2 起点终点集

每条 query 必须冻结并记录：

    case_id: A2B-01
    start: [x_m, y_m, yaw_rad]
    goal: [x_m, y_m, yaw_rad]
    expected_reachability: reachable_positive | unreachable_negative | ambiguous_excluded
    region_preference: none | center | edge
    case_sha256: <sha256>

运行前校验：起终点在地图内、为已知自由区、完整 footprint 不碰撞、通过统一连通性预筛选。输入无效属于 INVALID_INPUT，整批实验作废，不计入算法成功率。

所有架构必须使用完全相同的 query 文件、顺序和姿态；禁止运行中吸附起点或目标点。

### 4.3 车辆与碰撞语义

必须记录 footprint_hash 和完整 footprint 顶点。当前主线硬约束为：

    Rmin = 0.40 m
    maximum_curvature = 2.50 1/m
    allow_reverse = false
    allow_in_place_rotation = false

路径验收使用含 padding 的完整矩形 footprint，不得用单一圆半径替代。搜索层膨胀只用于代价和安全余量，不能替代最终 footprint 碰撞检查。

静态正式实验必须 dynamic_obstacles=false。3D-V0/2D-V0 的动态快照实验使用独立 experiment_kind=dynamic_incremental，不得与静态主表合并。

## 5. 运行协议

### 5.1 重复、预热与顺序

当前架构正式对比默认：每个 query 3 warmup + 5 measured；预热样本不进入统计。所有架构共享同一 query 顺序和 query_order_seed。

确定性算法按相同输入应可复现。RRT*/AO-RRT* 若获批准接入，必须固定种子 0..29，报告 30 次分布，不得只报告最优一次。

同轮 A/B 实验应交替执行架构臂或使用等价的固定顺序，并在 manifest 中记录，以便识别温度、后台负载和缓存预热影响。

### 5.2 Session 与缓存

地图级拓扑构建、索引、走廊缓存和 Smac/ROS session 的启动关闭必须单独记录：

    topology_build_count
    topology_load_count
    topology_build_wall_ms
    topology_build_cpu_ms
    topology_load_wall_ms
    topology_cache_bytes
    session_start_count
    session_close_count
    session_restart_count

在线规划耗时从请求发出到最终终态/路径返回计时；拓扑构建、缓存首次构建、进程启动和 session 关闭不计入单次 online wall time，但必须计入实验总成本报告。

同一实验必须明确 cache_mode=cold | warm | optimized。冷缓存和热缓存不得混合；若优化仅在热缓存生效，必须同时报告 cold/warm 结果。

### 5.3 超时与取消

每个请求只能使用一份总在线预算。当前主线默认：

    planner_budget_ms = 2000
    evaluator_deadline_ms = 2500

层间降级、重试、走廊扩大和 fallback 共享同一总预算；不得每次重试重新获得完整 2 s。任何修改预算的实验必须升级 protocol 或独立分组。

## 6. 结果分类

每个 measured 样本只能有一个主 result_code，按以下优先级判定：

| result_code | 判定 |
|---|---|
| COLLISION | 返回路径，但完整 footprint 与占据、未知或地图外相交 |
| KINEMATIC_INFEASIBLE | 路径无碰撞，但存在曲率、航向、位置连续性或运动模型违规 |
| SUCCESS | 在预算内返回非空、有限、无碰撞、运动学可行且端点误差合格的路径 |
| NO_PATH | 在预算内正常结束并明确无路，未输出路径 |
| TIMEOUT | 命中规划预算或评测截止时间 |
| EXCEPTION | 崩溃、协议错误、空成功响应、NaN/Inf、非法结果或未分类错误 |
| BACKEND_UNAVAILABLE | 真实后端未启动或不可用；不得伪造路径，单独报告环境阻塞 |

INVALID_INPUT 只用于数据集校验失败，不得作为算法失败率分母。每个主结果还必须带细分 reason_code 和 last_layer，例如：

    L1_NO_ROUTE
    L2_CORRIDOR_EXHAUSTED
    L2_DSTAR_TIMEOUT
    L3_KINEMATIC_SEARCH_TIMEOUT
    L3_LOCAL_SPLICE_VALIDATION_FAILED
    START_IN_LETHAL_SPACE
    NO_PATH_IN_CORRIDOR
    ACTION_ABORTED

调用级 action_success 不等于 query 级 final_valid_success。只有通过最终静态和运动学验收的路径才可计为 SUCCESS。

## 7. 统一指标

### 7.1 成功率与硬安全指标

主表必须同时给出 measured-sample 和 query-level 两种统计，不得混称“成功率”：

    sample_final_valid_rate = final_valid_samples / measured_samples
    query_all_repeat_valid_rate = queries_with_all_measured_samples_valid / measured_queries
    query_any_valid_rate = queries_with_at_least_one_valid_sample / measured_queries

历史项目以 sample_final_valid_rate 为兼容主指标；导师要求按起终点对比较时，必须同时展示 query_all_repeat_valid_rate。

硬安全统计：

    collision_case_count = 有至少一个碰撞插值位姿的 case 数
    kinematic_invalid_case_count = 有至少一个运动学违规段的 case 数
    reverse_case_count = 使用倒车的 case 数
    in_place_rotation_case_count = 使用原地旋转的 case 数

正式可行路径的硬线：碰撞、运动学违规、倒车和原地旋转均为 0。失败原因不能通过修改路径字段或删除违规点来消除。

### 7.2 端点与路径有效性

对输出路径以平移步长不大于 0.025 m、yaw 步长不大于 1 deg 插值，逐位姿执行完整 footprint 栅格相交检查。

必须记录：

    start_position_error_m
    goal_position_error_m
    goal_yaw_error_deg
    path_point_count
    static_footprint_valid
    kinematic_valid
    final_valid_success

默认端点验收阈值：位置误差不大于 0.125 m，目标 yaw 误差不大于 5 deg。不同阈值必须作为实验参数显式记录并单独比较。

### 7.3 路径长度、净空与平滑性

对每条有效路径单独计算，再对结果求分位数；禁止用总长度之比代替逐 case 比值。

    path_length_m = sum(hypot(dx, dy))
    euclidean_ratio = path_length_m / euclidean_start_goal_distance_m
    reference_ratio = path_length_m / reference_path_length_m

参考路径必须来自冻结的 reference_path_id。没有可信参考时写空值，并在 manifest 中说明 reference_unavailable_reason，禁止猜测全图最优路径。

必须记录：

    path_length_m: P50/P95/P99
    euclidean_ratio: P50/P95/P99
    reference_ratio: P50/P95/P99
    minimum_clearance_m: min/P50/P95
    mean_clearance_m: mean/P50/P95
    maximum_curvature_1pm: max/P95
    heading_change_rate_1pm: P95
    large_turn_count: >30 deg 的相邻平移段数量
    heading_jump_count
    position_discontinuity_count
    reverse_length_m
    in_place_rotation_count

原地旋转不增加路径长度；倒车长度取正值，但当前正式约束下 reverse_length_m 必须为 0。

净空必须基于完整 footprint 到最近障碍的距离，不能用骨架距离或单点距离冒充。

### 7.4 横向偏好

每条 query 必须记录 region_preference=none|center|edge。无偏好样本不参与偏好满足度统计。

靠中至少报告：

    center_clearance_m: P50/P95/P99
    center_preference_satisfaction_rate

靠边至少报告：

    target_wall_lateral_error_m: P50/P95/P99
    edge_preference_satisfaction_rate

偏好项权重、目标侧壁、允许误差和窄通道放宽策略必须写入 manifest。不得通过修改 inflation radius 隐式实现靠边或靠中。

### 7.5 在线耗时、CPU 与内存

官方 online wall time 为请求发出到收到完整终态和路径的单调时钟差值，单位 ms。必须报告：

    online_wall_ms: P50/P95/P99
    planner_wall_ms: P50/P95/P99
    timeout_rate
    censored_wall_ms_p50/p95/p99
    cpu_ms: P50/P95/P99
    avg_cpu_percent: P50/P95/P99
    ready_memory_mib: P50/P95/P99
    peak_memory_mib: P50/P95/P99
    incremental_memory_mib: P50/P95/P99

超时按 evaluator_deadline_ms 右删失；超时样本不得从尾部指标中静默删除，至少同时给出 observed 和右删失保守统计。

CPU 时间必须包含规划器创建的线程/子进程，不包含评测器、Gazebo、RViz、AMCL。平均核占用定义为 100 * cpu_ms / online_wall_ms，多线程可超过 100%。

峰值内存必须能区分 planner、pipeline 和 Nav2 stack；共享库场景同时报告 RSS 与 PSS，不能直接相加宣称整机总内存。

分位数统一使用 NumPy quantile(method="linear")。

### 7.6 分层与搜索诊断

每个架构只记录实际调用，不允许用估算值替代：

    l1_call_count
    l1_route_success_count
    l1_route_search_nodes
    l1_time_ms
    l2_call_count
    l2_time_ms
    l2_search_nodes_expanded
    l2_search_nodes_generated
    l2_search_space_ratio
    l3_call_count
    l3_time_ms
    l3_repair_window_count
    l3_retry_count
    fallback_count
    fallback_trace

架构专属字段：

| 架构 | 必报字段 |
|---|---|
| 3A-V0 | L2 corridor mode、Grid A* 展开/生成节点、L3 窗口来源和拼接失败数 |
| 2A-V0 | L2 调用必须为 0、走廊 profile、Smac 全程调用、端点候选和 corridor fallback |
| 3D-V0 | D* Lite g/rhs/OPEN/km 状态、初次展开、增量展开、changed cells、L1 reroute、L2 reset |
| 2D-V0 | 细化拓扑节点/边、边状态变化、候选接入点、L1 D* Lite 增量展开、L3 corridor 结果 |

没有真实后端提供的指标必须保留对应字段；字段值允许为空，并在 manifest 中记录 metric_unavailable_reason，不能填估算值。

### 7.7 拓扑、缓存与资源开销

拓扑预计算属于实验总成本，但不计入每次 online query：

    topology_build_wall_ms
    topology_build_cpu_ms
    topology_peak_memory_mib
    topology_load_wall_ms
    topology_cache_bytes
    topology_node_count
    topology_edge_count
    topology_connected_component_count
    topology_route_length_m
    topology_vs_reference_ratio

缓存 key 至少绑定地图 hash、footprint hash、分辨率、骨架/拓扑算法版本、架构 ID 和关键参数。命中、失效、重建和加载必须可审计。

## 8. 动态增量实验的独立口径

静态正式实验固定 dynamic_obstacles=false。3D-V0 和 2D-V0 的动态实验另建目录和报告，至少记录：

    snapshot_id
    snapshot_timestamp
    occupied_cells_count
    obstacle_confidence
    ttl_s
    map_version
    changed_cells_count
    changed_edges_count
    path_intersection_before_update
    vehicle_ahead_intersection
    dstar_incremental_expanded
    l1_reroute_count
    l2_reset_count
    path_changed_ratio
    repair_success
    dynamic_collision_count

动态障碍只写入 M_dynamic/边代价覆盖层，不得修改静态地图或静态拓扑缓存。障碍出现、移动、消失和整条走廊阻塞至少各有一个可复现实验场景。

动态更新耗时必须与初次规划耗时分开：

    initial_plan_wall_ms
    incremental_update_wall_ms
    full_replan_wall_ms

没有动态更新样本时，不得宣称 D* Lite 的增量收益。

## 9. 统一原始记录格式

每个 measured case 至少保存一行 CSV 和一份可追溯 JSON。CSV 必须包含以下字段；不适用或暂时无法测量的指标仍必须保留该列，值可以为空，并在 manifest/report 说明原因；不得删列、改列名或用估算值填充。

    run_id,experiment_id,protocol_version,architecture_id,implementation_revision,
    map_id,map_sha256,map_yaml_sha256,evaluation_resolution_m,case_id,query_sha256,
    seed,repeat,cache_mode,session_id,start_x,start_y,start_yaw,goal_x,goal_y,goal_yaw,
    preference,expected_reachability,result_code,reason_code,last_layer,
    action_success,static_footprint_valid,kinematic_valid,final_valid_success,
    wall_ms,online_wall_ms,planner_wall_ms,cpu_ms,avg_cpu_percent,
    ready_memory_mib,peak_memory_mib,incremental_memory_mib,
    start_position_error_m,goal_position_error_m,goal_yaw_error_deg,
    path_point_count,path_length_m,euclidean_ratio,reference_ratio,
    minimum_clearance_m,mean_clearance_m,maximum_curvature_1pm,
    heading_change_rate_p95,large_turn_count,heading_jump_count,
    position_discontinuity_count,reverse_length_m,in_place_rotation_count,
    l1_call_count,l1_time_ms,l1_route_search_nodes,
    l2_call_count,l2_time_ms,l2_search_nodes_expanded,l2_search_nodes_generated,
    l2_search_space_ratio,l3_call_count,l3_time_ms,l3_repair_window_count,
    l3_retry_count,fallback_count,fallback_trace,
    topology_cache_hit,topology_load_wall_ms,dynamic_snapshot_id

每个实验还必须提供指标可用性说明，例如：

    metric_availability:
      l2_search_nodes_expanded: {status: unavailable, reason: backend_did_not_expose_counter}
      reference_ratio: {status: unavailable, reason: no_frozen_reference_path}

status 只能为 measured、unavailable 或 not_applicable。无调用的计数型指标只有在日志确认未调用时才能填 0；“无法测量”不能填 0。指标列永远不能删除。

## 10. 汇总报告格式

每份 final_report.md 必须按以下顺序组织：

1. 实验身份：架构、实现、协议、地图、query、机器、软件和 hash；
2. 实验条件：分辨率、footprint、运动学、超时、缓存、session、重复方式；
3. 结果总览：sample/query 成功率、主安全指标、online P50/P95/P99；
4. 资源与分层耗时：CPU、内存、拓扑成本、L1/L2/L3 调用和节点；
5. 路径质量：长度、绕行比、净空、曲率、连续性、横向偏好；
6. 失败分析：主结果码、细分原因码、最后层、降级和回退链；
7. 对比与消融：只比较可比组，明确 cache/cold/warm 和不同 profile；
8. 限制与结论：不能从当前数据推出的结论必须写明；
9. 原始产物路径和复现命令。

结果表至少包含：

| 类别 | 指标 |
|---|---|
| 有效性 | sample_final_valid_rate、query_all_repeat_valid_rate、query_any_valid_rate |
| 安全 | collision_case_count、kinematic_invalid_case_count、reverse_case_count、in_place_rotation_case_count |
| 性能 | online wall P50/P95/P99、timeout rate、CPU、RSS/PSS |
| 路径质量 | length、reference ratio、minimum clearance、curvature、heading rate、large turns |
| 分层 | L1/L2/L3 time、calls、expanded/generated、repair/fallback |
| 预计算 | topology build/load、cache bytes、nodes、edges |
| 可复现 | commit、patch、map/query hash、seed、命令、环境快照 |

即使某架构不能提供某项指标，汇总表仍保留该指标列，并显示空值或 unavailable；同时给出不可用原因。

## 11. 判定规则

### 11.1 单路径

只有 result_code=SUCCESS 且同时满足静态 footprint、运动学、端点误差、禁止倒车和禁止原地旋转约束，才计为最终有效路径。

### 11.2 架构优化

优化只有在以下条件同时满足时才能宣称“有效优化”：

1. 关键功能 gate 不回退：最终有效率、碰撞和运动学硬线不恶化；
2. 被优化指标在同轮可比组中有实际改善；
3. 改善不是由提前失败、减少重试预算、不同缓存状态或不同超时造成；
4. 原始数据、调用计数、路径 hash 和失败分布可复核。

若未达到上述条件，报告使用 no_material_gain、functional_pass_efficiency_fail 或 inconclusive，并保留 baseline 回退路径。

### 11.3 性能合格线

规划耗时、内存、路径长度和偏好满足度的具体合格线，必须先给出同条件 baseline，再由导师确认。未确认前不得自行宣布“达标”或“替代主线”。

## 12. Codex 执行前检查清单

开始实验前必须逐项确认：

- [ ] 已读取 AGENTS.md、P0_EVALUATION_DEFINITION.md 和本文件；
- [ ] 已检查 worktree、当前 commit、未提交修改和历史实验目录；
- [ ] 已冻结地图/query/footprint/运动学/参数/随机种子；
- [ ] 已计算地图、query、源码和配置 hash；
- [ ] 已确认 cache mode、session policy 和 topology build/load 统计方式；
- [ ] 已确认 timeout、重试、降级和 fallback 共用总预算；
- [ ] 已确认静态/动态实验没有混表；
- [ ] 已确认输出目录为空或使用新的版本化目录；
- [ ] 已先运行 targeted smoke gate，再启动正式批量实验；
- [ ] 已确认真实后端不可用时停止并记录 BACKEND_UNAVAILABLE，不伪造成功；
- [ ] 已生成逐 case 原始 CSV/JSONL、manifest、汇总报告和复现命令；
- [ ] 已为每个空指标写入 metric_availability 和原因。

## 13. 变更规则

以下任一项改变都必须增加 protocol_version 或建立独立 variant：

- 地图解析、分辨率、占据阈值、未知区语义；
- footprint、padding、运动学约束、端点误差阈值；
- 超时、重试、降级、fallback 和失败码优先级；
- 预热次数、重复次数、query 顺序和随机种子；
- online wall time、CPU、内存、路径质量和成功率的统计公式；
- cache/session 是否计入在线耗时；
- 动态障碍开关或快照语义。

只改变算法内部实现而不改变上述口径时，保持架构 ID 不变，增加 implementation_revision，并在报告中说明 baseline 与 fallback。
