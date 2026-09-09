# PLN-02 语义端点过渡区复验 R1

本轮已执行冻结 targeted3 的控制重放和完整路径复验，固定每端 6 m 契约下为 **2/3，C1，未达到 B**。negative 的安全路径只有 10.553 m，裁剪后无有效统计区间；它不是安全失败，也不是连续空间不可行证明。在线规划、48-bin 三臂、selected8、服务端 ACK 和在线性能均未运行。

日期：2026-09-07。架构仍为 `2A-V2`，契约为 `semantic-endpoint-transition-6m-r1`，未定义 V3。

## 复验结果

![三条冻结路径的语义统计窗口](../private_data/pudu_wanda_3f/results/transition_preflight3_r1_20260907T050200Z/transition_windows.png)

想做什么：确认同一个 6 m 契约能否同时用于三条冻结路径。图由本轮实际弧长生成；灰色为不统计软语义的端点区，绿色为有效窗口，红色为没有有效窗口。各行横轴尺度不同。

采用：输入路径保持不变，语义指标使用全局 0.025 m 等弧长站位；完整路径另用不超过 0.025 m / 1° 的密集采样审核安全。通过控制证书或原航点重建 Dubins 控制链，并逐点精确匹配原路径。

结果：三条控制重放、完整 padded footprint、精确端点、同 lane instance、R0、禁止倒车/原地旋转、曲率和禁停终点检查通过。语义门槛仅两条通过。

| 查询 | 路径长度 | correct-side | target-band | 横向误差 P50 | 本契约结果 |
|---|---:|---:|---:|---:|---|
| positive | 32.003801 m | 0.853933 | 0.544320 | 0.387184 m | 通过，801 样本 |
| negative | 10.553371 m | 不适用 | 不适用 | 不适用 | INVALID，0 样本 |
| south | 59.464220 m | 1.000000 | 0.737757 | 0.150000 m | 通过，1899 样本 |

门槛保持为 correct-side ≥0.80、target-band >0.50、横向误差 P50 ≤0.50 m。negative 未计成通过，也未从分母中删除。

## 更正上一轮结论

上一轮敏感性报告把 positive 在 `d=6 m` 的通过，与 negative/south 在原契约 `d=0` 的通过并列，误推为统一契约下三条都通过。事实上 negative 原 sensitivity CSV 的 `d=6 m` 行已经是零样本、失败。这是比较口径错误，本轮没有改动端点或路径来消除该失败。

本轮使用统一等弧长站位，所以 positive 的结果为 `0.853933 / 0.544320 / 0.387184 m`，不直接沿用旧的分段采样数字。negative 的全路径指标仍合格：`1.000000 / 0.731132 / 0.250000 m`；这只能作为另一个契约的参考，不能替代当前 INVALID。

历史 negative 严格 witness 长度为约 9.14、9.28、10.55 m，均不满足 `L>12 m`。另有 13.816 m 安全 corridor 候选，但全路径 target 样本为零，裁剪不能使其达标。以上是现有候选的审计，不证明任何更长合格路径均不存在；本轮未人为绕圈或拉长路径。

## 实现边界

```mermaid
flowchart LR
    A[地图和冻结查询身份校验] --> B[控制重放及完整路径安全审计]
    B --> C[统一 6 m 窗口语义审计]
    C --> D[2/3：negative 无有效区间]
    D --> E[停止在线与 selected8]
```

想做什么：避免离线重放或不完整汇总被误认成在线 B。采用：独立 preflight CLI 汇总三条结果；空集、缺查询、重复查询、乱序、任一失败均拒绝进入在线。离线全通过也不能单独晋升。

结果：原新建的 transition benchmark 不再委托 r2 启动 ROS，仅返回明确的禁用状态。此前入口存在契约配置误作规划器配置、未绑定 48 bins、重复次数被当查询数、任一次成功掩盖其他重复失败等问题，不能作为合格在线实现；后续真实在线接口需要另行实现和审查。`semantic_transition_online_adapter.py` 文件名仅兼容历史入口，其现有功能明确为保存路径的离线复核，不是在线规划器；plain JSON 不再称为实际发布的 `nav_msgs/Path`。

## 下一步契约建议

建议单独批准短路径补充规则：`L≤12 m` 时使用完整路径语义统计，`L>12 m` 时继续使用每端 6 m 窗口。所有长度都必须通过完整安全和运动学审核，不允许通过人为绕圈切换评分口径；需要同时报告全路径和适用区间指标，并审查 12 m 阈值附近的行为。

该规则**尚未实施、尚未获批**，不能据此判本轮通过。若批准，另起 contract revision，先统一复验三条，再设计并实现真正按请求在线求解的接口；不得把缓存 witness 的重放当成在线规划。parking-only 的中心带验收阈值也必须在进入 selected8 前明确冻结，现有 canonical semantic auditor 仅提供偏差统计，不存在可直接引用的完整中心带验收门槛。

## 记录

- 权威结果目录：`private_data/pudu_wanda_3f/results/transition_preflight3_r1_20260907T050200Z/`。
- [最终门禁](../private_data/pudu_wanda_3f/results/transition_preflight3_r1_20260907T050200Z/gate.json)、[逐查询 CSV](../private_data/pudu_wanda_3f/results/transition_preflight3_r1_20260907T050200Z/per_query.csv)、[输入与源码绑定](../private_data/pudu_wanda_3f/results/transition_preflight3_r1_20260907T050200Z/manifest.json)、[产物哈希](../private_data/pudu_wanda_3f/results/transition_preflight3_r1_20260907T050200Z/artifact_hashes.json)。
- 三个 query 子目录保存独立完整安全审计；上层 `*_controls.json` 为控制重放结果，不能只用子目录中未核验控制的 pose-only 标志替代。
- 本轮重新校验地图、语义地图、targeted3 内容 hash 和默认 selected8 的文件/内容 hash、顺序、净空、拓扑长度约束；selected8 仅校验配置，**没有运行规划**。
- 旧 `next_stage_online_adapter_positive_20260907T160200Z`、`...south_20260907T160300Z`、`...negative_20260907T160400Z` 目录只读保留，**排除出在线证据**。旧敏感性及探索目录只作背景；不回写其结果。
- 未修改飞书、pinned Nav2、旧实验或默认 selected8；未 stage、commit、push。本契约实验未启动 ROS/Nav2/Smac；全包测试另含 domain 194 的独立 Smac 后端探针，不计入契约在线实验。2026-08-26 的旧可视化进程和其他任务的 ROS 进程均未终止。

复现（退出码 `2` 表示门禁失败，结果完整写出）：

```bash
cd /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation
/usr/bin/python3 -m arena_evaluation.semantic_transition_preflight \
  --workspace /home/robot/pudu_robot_ws \
  --output /home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/<new-write-once-directory>
```

验证结果：

- 新增专项：15 passed。
- 全包：加载项目 `setup_arena4_runtime.bash` 后，**402 passed，6 warnings，85.32 s**；包含真实 Smac 后端探针，不是拼接多个部分测试的计数。
- 首次默认 `python3` 命中缺少 pytest/YAML 的 pyenv；改为 `/usr/bin/python3`。只加载 Humble 的尝试为 401 passed、1 failed，根因是 launch 搜索路径缺少项目 `arena_evaluation` 包；保留该失败记录。补齐项目 overlay 后单测和全包均通过，未修改测试或规划器来规避失败。
- `compileall`、根仓库 `git diff --check`：通过。
- 隔离 `colcon build arena_evaluation`：通过，6.21 s；仅既有 OMPL/C++ warnings。安装后 `semantic_transition_preflight --help`：通过，使用系统 Python。没有覆盖主安装目录。
- [最终全包验证记录](../private_data/pudu_wanda_3f/results/transition_r1_full_verification_20260907TCd6yIX/verification_runtime_final.json)、[最终全包日志](../private_data/pudu_wanda_3f/results/transition_r1_full_verification_20260907TCd6yIX/pytest_full_runtime_env.log)、[隔离构建记录](../private_data/pudu_wanda_3f/results/transition_isolated_build_20260907T050227Z/verification.json)。确证由本轮测试启动的进程已全部退出，残留为 0；未手工终止其他任务进程。
- 独立复核：权威目录 36 个产物哈希、6 个源码快照哈希、3 条 witness 的路径/控制/元数据/NPZ 哈希全部匹配。

测试通过只说明当前实现和复验工具通过回归，不会把 `2/3` 离线门禁变成 B。
