# Prompt for the Next Codex

在 `/home/robot/pudu_robot_ws` 继续 Arena4 静态全局 A2B 评测。先完整阅读：

`/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/AGENTS.md`

再阅读：

`/home/robot/pudu_robot_ws/docs/CODEX_HANDOFF_STAGE8.md`

## 当前状态

- Stage 2–8 已完成代码、测试和 Hospital 实验；不要重做。
- 根仓库当前报告分支为 `report/2026-08-17_2026-08-21`，HEAD `27d83af`；`main` 的阶段 2–8 主线提交是 `21dcaee`。
- Arena evaluation 嵌套仓库有意保持 dirty，不能 reset、checkout 或清理用户改动。
- 所有实验输出在 `experiments/`，根仓库忽略该目录。
- 最终打包目录：
  `/home/robot/pudu_robot_ws/experiments/deliverables/arena4_static_a2b_experiment_bundle_stage8_v1_20260821`

## 本任务目标

只完成“固定地图集、固定 query 集、现有 A2B 规划器耗时-内存-规模基线曲线”。不要实现新算法。

候选统一 0.05 m 地图规模档：

```text
ignc_005       25x25 m，约 250,000 cells
house17_005    31.3x24.05 m，约 301,106 cells
factory_005    60x60 m，约 1,440,000 cells
hospital_005   80x80 m，约 2,560,000 cells
```

这些候选必须先派生、验 hash、验 origin/范围和静态 occupancy 语义；不得直接假设可用。

## 固定约束

- `dynamic_obstacles: false`；禁止 actor、HuNav、Pedsim、random/scenario obstacles。
- 不启动 Gazebo、完整 Arena4、TEB、MPPI、DWB 或控制实验。
- 不修改原始地图。
- 不重跑或覆盖 Stage 3–8 目录。
- 继续使用现有 `planner_benchmark`，只新增独立协议/输出目录和只读跨地图报告。
- 主统计只用 `run_mode == measured`。
- 分开报告 `action_success`、`static_footprint_valid`、`final_valid_success`。
- 失败 query 保留结构化 `result_code`。

## 交付要求

1. 每张地图一个版本化协议和固定 query YAML。
2. 查询验证 CSV，明确 INVALID_START/INVALID_GOAL、footprint collision、连通性。
3. NavFn Product/Normalized 和 Smac Product/Normalized 基线；每图每组合使用相同 query 数、warmup、repetition、timeout。
4. 只读跨地图汇总 CSV，至少包含：

```text
map_id
map_sha256
resolution
width_cells
height_cells
grid_cells
free_grid_cells
physical_area_m2
planner_id
config_variant
run_mode
count
success_count
action_success_rate
static_footprint_valid_rate
final_valid_success_rate
planning_time_ms_P50/P95/P99
wall_time_ms_P50/P95/P99
cpu_total_ms_P50/P95/P99
planner_rss_peak_bytes_P50/P95/P99
planner_pss_peak_bytes_P50/P95/P99
stack_rss_peak_bytes_P50/P95/P99
stack_pss_peak_bytes_P50/P95/P99
```

5. 生成耗时、CPU、planner/stack 内存、成功率随 `grid_cells` 和物理面积的曲线。
6. 在报告中明确：四个地图之间的场景拓扑和 query 难度可能不同，曲线是观测基线，不是纯复杂度因果证明。
7. 运行测试、增量构建、CLI help 检查，清理本轮进程。

开始前只做检查和计划；在确认地图/query 验证通过后再运行正式实验。最终汇报必须列出新文件、每图每组合行数、成功/有效率、P50/P95/P99、内存、CPU、图表路径和未解决问题。
