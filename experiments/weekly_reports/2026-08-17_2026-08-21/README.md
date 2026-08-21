# PLN-02 周报归档

- 周期：2026-08-17 至 2026-08-21
- 姓名：李永祺
- 范围：静态 Hospital A2B 全局规划
- 地图：0.05 m/cell，1600 x 1600 栅格

## 目录

- [周报](./weekly_report.md)
- `data/planner_benchmark/hospital_005/`：Stage 5 NavFn/Smac 基线、CSV、日志、路径和绘图
- `data/layered_planner_benchmark/hospital_005/`：Stage 6、Stage 7 消融、Stage 8A/8B 数据、CSV、日志、路径和绘图
- `data/maps/hospital_005/`：本周使用的静态地图、地图元数据和 query/protocol 文件
- `report_assets/`：从 `experiments/deliverables` 筛选出的汇报摘要表、协议输入和代表性图表

## 归档规则

本归档只复制正式 Stage 5–8 输出和本周使用的 Hospital 地图，不包含 `archive_*`、`smoke`、旧版 summary 或 `experiments/teb_hospital`。原始实验目录保持不变。

`report_assets/` 只保留导师汇报需要的材料：Stage 5 基线摘要、Stage 6 L1/L2 消融摘要、Stage 7 历史旋转消融摘要、Stage 8A/8B 验收摘要、固定输入协议以及代表性图表。每个 planner 的重复分布图、原始路径和中间调试产物未重复放入该目录，但完整数据仍在 `data/` 下。

筛选时排除 Stage 3/Stage 4 历史结果、重复的 0.1 m 主图、每个 planner 的重复分布图、smoke/debug/中间尝试和原始路径压缩文件；这些内容不参与本周结论。`report_assets/source_bundle/` 保留 deliverables 包的版本和校验信息，便于追溯来源。

Stage 7 目录保留为历史消融数据；它允许原地旋转，不能作为正式车辆约束结论。Stage 8A 才是禁止原地旋转的正式硬约束结果。Stage 8A 的组合耗时是基于分段结果的估算，不是同进程端到端实测。
