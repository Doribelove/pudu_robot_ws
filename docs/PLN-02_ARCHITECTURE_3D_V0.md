# PLN-02 `3D-V0` 动态增量分层架构

`3D-V0` 不是三维空间规划，而是三层职责中的动态增量版本：

```text
L1  二维骨架拓扑图 + Graph A*       静态通道选择
 ↓
L2  走廊二维栅格 D* Lite             动态栅格增量修复
 ↓
L3  局部 Smac Hybrid A*              航向、曲率和车体可行性
```

`3A-V0`（V7）和 `2A-V0` 保持不变，`3D-V0` 使用独立入口、协议和实验目录。

## 数据分层

```text
M_static = occupancy + footprint free mask + skeleton + topology
M_dynamic = timestamped occupied cells + confidence + TTL
M_plan = M_static + M_dynamic
```

动态障碍永远不写回静态地图。L2 和 L3 使用同一 `snapshot_id`；快照过期后清除动态代价。

## 组合逻辑

```text
初次请求：L1 Graph A* → 初始化 L2 D* Lite → L3 Smac

动态快照：
  不影响旧路径                 → 不重规划
  影响前方局部路径             → L2 更新受影响节点并做 A-B 修复
  当前走廊确认整体阻塞         → 标记 L1 边并重新选通道
  L3 失败                      → 先换 B/扩大窗口，不直接判定通道阻塞
```

L2 持久化 `g/rhs/priority_queue/start/goal/km/snapshot_id`，只更新动态障碍及其邻域。新路径采用“旧前缀 + A-B 修复段 + 旧后缀”拼接，并重新执行完整验收。

## 固定约束

- L1/L2 只使用 `(x, y)`；L3 使用 `(x, y, yaw)`。
- `Rmin=0.40 m`、最大曲率 `2.50 1/m`。
- 禁止倒车和原地旋转。
- 静态碰撞、动态碰撞和运动学违规均为硬验收项。
- RRTstar/SST 不调用。
- 动态实验不混入 `3A-V0` 静态正式结论。

## 代码入口

```text
arena_evaluation/dstar_lite.py
arena_evaluation/dynamic_snapshot.py
arena_evaluation/l2_dstar.py
arena_evaluation/layered_dynamic_pipeline.py
arena_evaluation/layered_dynamic_pipeline_cli.py
```

CLI 的 `--demo` 只验证离线 D* Lite 状态更新；真实 Smac 通过 `SmacHybridAdapter` 注入已有地图级 session，不可用时返回 `L3_BACKEND_UNAVAILABLE`，不伪造成功。

## 实现版本

```text
architecture_id: 3D-V0
implementation_revision: r1
```
