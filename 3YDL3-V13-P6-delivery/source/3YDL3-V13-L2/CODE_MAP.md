# 代码阅读入口

| 顺序 | 文件 | 阅读重点 |
|---|---|---|
| 1 | `src/three_yd_v13/service.py` | P7-r1位姿、障碍事务和路径状态 |
| 2 | `src/three_yd_v13/runtime_l1.py` | 运行时车道分段、节点与语义映射 |
| 3 | `src/three_yd_v13/schedule.py` | 新域调度、共享预算和请求缓存 |
| 4 | `profiles/p6/src/three_yd_v5/p6_hint.py` | 局部扩域、请求上下文和阶段键 |
| 5 | `profiles/p6/src/three_yd_v5/route_quality.py`、`recovery.py` | 短参考线与恢复链衔接 |
| 6 | `base/src/three_yd_v13/incremental.py`、`astar.py`、`state.py` | A*/D*、事件更新与状态管理 |
| 7 | `base/native` | P2d原生路径提取、依赖更新和索引堆 |
| 8 | `base/runtime/base/src/three_yd_v5` | 连续L3、轨迹证书、同步和终验 |
| 9 | `tests/p6`、`tests/p7` | 扩域、调度和运行时图合同检查 |

`env.bash`按P6或P7-r1配置组合模块路径。共享代码位于`base`，P6覆盖扩域相关模块，P7-r1进一步覆盖事件接口、运行时L1图与调度。

`tools/original_p6`与`tools/original_p7`保留实验入口，依赖原实验环境配置。源码来源及SHA256见[metadata/SOURCE_PROVENANCE.json](metadata/SOURCE_PROVENANCE.json)。
