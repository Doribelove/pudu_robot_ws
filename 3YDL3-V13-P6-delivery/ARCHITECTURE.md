# P6三层规划架构

`地图、实际起终位姿、车体与区域策略 → L1通道选择 → L2二维参考与允许域 → L3连续曲线 → 终验与成本图ACK → 全局参考点列`

| 层次 | 做什么 | 代码入口 |
|---|---|---|
| L1 | 语义图按道路邻接、方向、停车接口及禁转选择通道；非语义图使用粗自由空间图 | `runtime/3YDL3-V13-L2/base/runtime/base/src/three_yd_v5/topology_astar.py` |
| L2 | 在允许域内搜索XY参考，应用靠中、靠边或无偏好；同域事件采用D*增量更新 | `runtime/3YDL3-V13-L2/base/src/three_yd_v13/incremental.py`、`state.py` |
| L3 | 构建有序凸矩形走廊，以三次B样条控制点表示全路线，通过信赖域SQP和Clarabel求解 | `runtime/3YDL3-V13-L2/base/runtime/base/src/three_yd_v5/curve_l3.py` |

运行链为`IncrementalService → IncrementalController → EventDStar → NativeIncrementalState`。P6在短参考线受阻时，先尝试障碍附近半径5m的中性区域扩域，再进入原走廊和恢复链。相同域内可保留D*状态；换域后重新构建。

多尺度搜索、原生内核、辅助字段分块、位压缩和共享有界缓存共同减少计算与副本。L3先认证简单候选，困难路线再优化、细化或恢复。

车体0.53×0.45m，前进运动，最小转弯半径0.40m。精确求解失败后允许双端0.20m/30°容差。所有恢复共享30s请求预算和15s累计L3预算，返回路径通过车体、曲率、交通和实际Nav2成本图ACK检查。

运行资产补齐和资源路径绑定由独立适配器提供，原449份发布文件保持固定字节。`runtime_supplement`保存隔离复现需要的ROS插件和OMPL库；使用条件见[运行说明](RUNNING.md)。
