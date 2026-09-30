# 3YD-V7-r1 模式与层间接口

r1沿用r0全部层间字段及任务结构，不改变语义路线、靠右几何、候选数量或搜索范围。为兼容原缓存和输出，层间schema保持原值；实验及版本元数据标记V7-r1。

运行配置新增`THREEYD_V7_GOAL_STOP=1`与`THREEYD_V7_COMPACT_LEGACY=1`，分别控制无语义目标停止及语义备选状态复用。新原生内核通过`tools/build_goal_search.py`独立构建。

诊断字段`search`准确区分`goal_stop_dijkstra`、`full_component_dijkstra`和`bounded_weighted_astar`。`expanded`为已确定距离的结点数；r0完整分量Dijkstra会计算全部可达结点，r1仅计算目标确定前的结点。粗图节点/边数量不变。

| 层 | 语义模式 | 无语义模式 |
|---|---|---|
| 输入 | PGM/YAML＋人工语义区域/拓扑＋起终点 | 0.05 m PGM/YAML＋起终点 |
| L1 | 道路/路口有序候选、交通方向、靠右参考 | 粗自由空间路由、单条中性引导线、沿线搜索范围；不产生交通规则 |
| L2 | 0.15 m 二维 D*；道路方向和 0.30 m 中心侧靠右软目标 | 同一 D* 与障碍处理；方向场为零，引导线不偏移，仅作软代价目标 |
| L3 | 0.05 m 原图、Smac、Jackal 模型、连续车体和语义交通终验 | 同一地图、Smac、车辆和几何终验；跳过不存在的语义交通审核 |

## 无语义 L1 输出

`load_map_only_world(map.yaml, cache_root)` 按原占据图阈值读取 PGM；未知格和占据格均不可进入。地图分辨率必须为 0.05 m。`FreeSpaceTopology.routes(query)` 返回至多一条 `TopologyRoute`，`channel_plan(route)` 返回：

- `mode="free_space"`、`right_side_rule=null`、`rules_hash=null`；
- `right_reference_polyline`：为兼容旧控制器保留的**字段名**，实际含义是无左右侧偏好的粗引导折线；
- `scope_half_width_m=2.0`：L2/L3 允许搜索的粗路线邻域；L2 的参考线吸引仍是软代价，障碍硬约束优先；
- `region_sequence=[1]`：仅为适配既有路由对象的中性占位符，**不是自动识别的道路或路口**；
- 地图哈希、起终点和路线签名，用于既有版本化缓存。

L1 用保守聚合的 1.2/0.6/0.3/0.15 m 网格找粗连通路，编译较小网格的静态连通图供查询复用。网格对角连接禁止穿过障碍角；最终折线在原 0.05 m 占据图上复核。L1 不判断车体姿态或保证整条路径可执行，车辆可行性由 L2/L3 完成。

## 下游最小模式分支

`V5Controller` 检查拓扑对象的 `mode`。无语义模式使用 `scope_mask(route)` 形成搜索范围；L2 的方向代价为零，不调用道路截面、靠右比例或 0.30 m 偏移。`FreeSpaceTraffic.audit` 只表示“无交通语义可审”；地图占据、终点、连续矩形车体、曲率和转向仍由原有规划器及独立 `PathAuditor` 审核。L3 不启用全图回退，地图内容 ACK 流程保持原有实现。

2.0 m 是当前粗引导的有限搜索范围，不是通过性证明。若粗线附近被车辆约束封住，可能需要改变 L1 候选或增宽受控搜索范围；本轮没有通过放宽车辆终验换取 L1 有效输出。
