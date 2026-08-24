# 给新 Codex 的完整启动指令

将以下内容完整发送给新的 Codex：

```text
请接手 `/home/robot/pudu_robot_ws` 中的 PLN-02 静态 A2B 全局规划课题。

本对话没有之前的聊天上下文。开始任何分析、修改或实验前，必须完整读取：

1. `/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/AGENTS.md`
2. `/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/CODEX_HANDOFF_PLN02_2026-08-24.md`
3. `/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/weekly_report.md`
4. `/home/robot/pudu_robot_ws/experiments/weekly_reports/2026-08-17_2026-08-21/PLN-02_课题完成情况_2026-08-21.md`

然后执行只读检查：

cd /home/robot/pudu_robot_ws
git status --short
git branch --show-current
git log -5 --oneline --decorate

注意当前可能位于报告分支 `report/2026-08-17_2026-08-21`。报告分支只用于周报归档；如果后续任务涉及源码开发，先检查是否存在用户修改，再基于 `main@21dcaee` 工作。不要擅自删除、覆盖或移动已有实验数据。

课题范围严格限定为静态 A2B 全局规划：

- `dynamic_obstacles=false`；
- 地图固定 `0.05 m/cell`，不实现多分辨率；
- 禁止原地旋转，硬最小转弯半径 `0.40 m`；
- 允许倒车，`reverse_penalty=2.0`；
- 正式运动学模型使用 `REEDS_SHEPP`；
- 不做动态障碍、TEB/MPPI/DWB 控制、cmd_vel、建图、定位、多机调度或真机部署。

必须保留以下事实：

- Stage 5 是 NavFn/Smac 早期基线；
- Stage 7 允许原地旋转，只是历史消融；
- Stage 8A 才是正式禁止原地旋转的硬约束结果；
- `action_success` 不等于 `final_valid_success`；
- q00/q08 仍存在 `KINEMATIC_REPAIR_FAILED`；
- q04 是 `STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH`；
- 当前组合耗时是估算，平均为 full Smac 的 22.79 倍，不能称为加速；
- 目前只完成 Hospital 单地图，不能声称任务书或规模曲线已经完成。

当前已完成 L1 拓扑、L2 全图/走廊 A*、局部 L3 Hybrid 硬约束修复和靠中/靠右偏好阶段性评测；未完成统一端到端规划接口、同进程端到端计时、多地图规模曲线、完整降级链、RRT*/Kinodynamic-RRT 和最终结题报告。

完成读取和检查后，请先向我汇报：

1. 当前 Git 分支和工作区是否干净；
2. 你理解的课题范围；
3. 已完成、部分完成和未完成事项；
4. 你认为下一步最高优先级；
5. 本次任务是否需要修改代码或运行实验。

在我确认下一步前，不修改源码、不启动正式实验、不运行完整 Arena4/Gazebo，也不生成新的性能结论。
```
