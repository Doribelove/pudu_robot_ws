# 3YDL3 · L2优化架构

采用L1选路、L2几何搜索、L3连续轨迹优化的三层架构。输入为地图、当前位姿、目标和障碍变化，输出为`x/y/yaw`全局参考点列，以及路径有效性、失败原因和来源信息。

当前采用 **P6局部扩域**：短参考线受阻时，在障碍附近扩展L2提示域，再生成满足车辆约束的完整轨迹。最新功能候选 **P7-r1** 将障碍反馈到L1，把被切断的长车道分为运行时片段，支持换通道后回到原车道的另一段。

| 内容 | 入口 |
|---|---|
| 三层职责与恢复流程 | [架构说明](ARCHITECTURE.md) |
| P6采用版本 | [profiles/p6/src](profiles/p6/src) |
| P7-r1最新候选 | [src/three_yd_v13](src/three_yd_v13) |
| 基础三层代码、原生内核与数值依赖 | [base](base) |
| 配置、接口与检查 | [运行说明](RUNNING.md)、[代码入口](CODE_MAP.md) |
| 本版实测与待解决问题 | [实验结果](report/RESULTS.md)、[P6请求](report/p6_local_requests.csv)、[P7-r1请求](report/p7_r1_formal_requests.csv) |
| 源码来源与完整性 | [来源清单](metadata/SOURCE_PROVENANCE.json)、[发布清单](MANIFEST.json) |
| 在线展示 | [P6地图](https://gquhtm3ghb2.feishuapp.com/app/app_17gmhxhed6d/?dataset=p6&map=arena_short&obstacle=xlarge&algorithm=H-I&version=local&section=map)、[P7-r1换通道](https://gquhtm3ghb2.feishuapp.com/app/app_17gmhxhed6d/?dataset=p7&map=twin_long&obstacle=thunder&algorithm=H-A&section=map) |

P6的Arena首次5m障碍主事件20/20成功，完整响应中位约1.51–1.78s，路径约136.356/255.157m。P7-r1三项重点换通道任务18/18成功，精确到达请求端点。
