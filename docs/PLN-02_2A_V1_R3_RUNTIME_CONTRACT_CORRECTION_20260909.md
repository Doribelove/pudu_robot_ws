# 2A-V1 r3：运行时合同更正与 next3 结果入口

上一轮 next2 五图正式 Smac YAML 实际使用72 heading bins，报告和protocol声明48。因此旧“冻结合同全部通过”结论撤回；旧路径审计事实仍保留，不能把72与48的差值作为配置一致的优化消融。旧实验文件与归档不改写。

本轮在启动时回读实际 planner/costmap 参数，并要求地图绑定的合同receipt后才能发布和搜索。冻结候选candidate_04使用实际48 bins、完整Jackal footprint、DUBIN、Rmin0.40 m、原迭代上限和unchanged canonical PathAudit。

五图固定20查询、每查询3 warm-up/5 measured：495/500 final-valid；medium-160和medium-200均100/100，small100/100、mentor95/100、mentor-4×100/100。五图正式安全、有效率和共同成功P50门槛通过。剩余mentor/A2B-16保留失败，另有冻结canonical几何下静态不连通证书，不计规划成功。

Cold为每图一个独立进程和空tile cache，共5/5；不外推到25样本、全查询cold成功率或P95/P99。本轮未重跑四张large/xlarge。

- [本轮最终报告](/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2a_v1_r3_next3_hard_query_feasibility_20260909T092923Z/final_report.md)
- [正式验收](/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2a_v1_r3_next3_hard_query_feasibility_20260909T092923Z/analysis/final04_attempt01/acceptance_gates.json)
- [参数问题证据与更正](/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2a_v1_r3_next3_hard_query_feasibility_20260909T092923Z/investigation/protocol_discrepancy.json)
- [复现命令](/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2a_v1_r3_next3_hard_query_feasibility_20260909T092923Z/reproduce.md)
- [归档校验](/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2a_v1_r3_next3_hard_query_feasibility_20260909T092923Z/archive/verification.json)

本索引是新增更正入口，不修改历史报告。源码/测试改动九个文件，未stage、commit或push；完整仓库状态随本轮environment.json归档。
