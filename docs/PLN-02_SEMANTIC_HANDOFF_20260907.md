# PLN-02 语义规划续接记录

保存时间：2026-09-07 06:57 UTC。用户要求保存当前进度、切换到新 Codex 对话；本轮停止新增搜索，不代表任务完成。

## 1. 最重要的结论

**尚未达到真实 B，不可宣称在线系统完成或生产可用。** 正式架构仍为 `2A-V2`，新候选尚未正式命名。用户允许探索和修改架构，但不允许靠修改冻结地图、端点、安全门槛或统计作弊过关。

已经实现并运行三个使用当前地图/请求的独立候选：48-bin reference Hybrid、稀疏 directed SE(2) roadmap、地图派生 portal + 连续 Dubins 航点优化。前两种未找到 positive 严格路径；第三种在约 297 秒后生成了一条**数值、安全、精确重放及非自交筛查通过**的路径。

**但最新图像检查发现：这条新候选越过目标约 9 米，到目标带掉头，再沿相邻轨迹返回。非自交并不等于“没有不必要的往返”。这项必要性审计仍未解决，不能直接把其 `gate_passed=true` 当作完整严格验收。** 旧 positive witness 更有两处自交，已被新的失败关闭审计挡住。

当前下一步首先是解决上述反作弊/必要性判定与 positive 有效见证，而不是立刻运行在线或 selected8，也不是继续扫 2A-V2 soft cost 权重。

## 2. 用户已经批准的契约

本对话用户明确回复“批准补充规则并继续”，不要重新请求相同批准。

- `L <= 12 m`：整条路径统计软语义。
- `L > 12 m`：统计弧长 `[6 m, L - 6 m]`。
- lane：correct-side `>=0.80`；correct-side 且 lateral error `<=0.50 m` 的 target-band ratio `>0.50`；lateral error P50 `<=0.50 m`。
- parking：现有 normalized deviation P50 `<=0.25`；deviation `<=0.25` 的中心带比例 `>0.50`。
- parking 的 deviation 是既有无量纲净空场：`1 - min(region_clearance,map_clearance)/component_max_combined_clearance`，不是米。
- 使用固定全局 0.025 m 弧长采样；lane 与 parking 各自分母，不得跨类抵消。
- 全路径硬安全、精确端点、yaw、地图和 footprint 不变。不得以增加点密度、重复点、无意义绕圈、往返访问同一区域提高指标。

权威契约：

- `docs/PLN-02_SEMANTIC_TRANSITION_CONTRACT_R2.md`
- `external/arena4_ws/src/arena/evaluation/arena_evaluation/config/pudu_wanda_3f_semantic_endpoint_transition_r2.yaml`
- `protocol_id: PLN-02-SEMANTIC-ENDPOINT-TRANSITION-R2-V1`
- `contract_revision: semantic-endpoint-transition-short-full-6m-r2`

旧 original/R1 契约与 C1 历史结果保留，不得回写成新契约成功。

## 3. 工作区与约束

项目 `/home/robot/pudu_robot_ws`；包目录下文记为 `PKG`：

```text
/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation
```

先完整读取 `external/arena4_ws/src/arena/evaluation/AGENTS.md` 及其他适用 AGENTS。

- 静态 A2B 全局规划，原始地图 0.05 m；不得新增多分辨率地图、动态障碍或局部跟踪控制。
- forward-only、禁止原地旋转；硬 `Rmin=0.40 m`、最大曲率 `2.50 1/m`。
- padded Jackal footprint：`(+/-0.265, +/-0.225)`。
- 不改 pinned Nav2，不终止其他任务进程，不覆盖旧实验目录。
- 工作树和暂存区很脏，包括用户的 3D 项目修改；不得 reset/clean/restore/stage/commit/push。
- 主仓 HEAD：`ed25b1767976ecb48086bfa429b2b5a3a49d7226`。
- evaluation 嵌套仓 HEAD：`94762429bea19b84cab50a3d0910a736184738a0`。
- 使用 `/usr/bin/python3`。普通 `python3` 为 pyenv 环境，依赖不同。
- ROS 测试需 `source /home/robot/pudu_robot_ws/setup_arena4_runtime.bash`。
- 不修改飞书，不新增自动化或启动另一个对话，除非用户明确要求。
- 不恢复/重新指派旧代理。旧 next_stage_audit、online_adapter、protocol_v3 均因 429 停止；新对话默认本地继续，遵守自身委派限制。

## 4. 冻结输入与默认 selected8

常规实验必须默认使用 `PKG/config/pudu_wanda_3f_selected8_gt50m_r2_v2.yaml`，通过 `semantic_query_defaults.load_query_set(..., require_default_contract=True)` 验证。

| 身份 | 值 |
| --- | --- |
| query-set | `pudu_wanda_3f_semantic_compare_selected8_r2_v2` |
| selected8 query hash | `7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e` |
| map hash | `05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a` |
| semantic hash | `2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553` |

保持原 8 条顺序、拓扑长度 >50 m、端点净空 >=1.5 m。该集经过筛选，不能推断总体成功率。

targeted 三条是用户授权的难例预检，不替代 selected8：

- `r3-mirror-1-positive`
- `r3-mirror-2-negative`
- `cmp2-02-lane-south`

targeted config：`PKG/config/pudu_wanda_3f_r3_targeted_preflight3_v1.yaml`；query hash `66212b05ef6c4d16eaedafc1c27866387c3d74aa85bf5ac69bca92487254667d`。

输入 NPZ/JSON：`private_data/pudu_wanda_3f/results/constraint_inputs_20260907T021418Z/`。

地图：`private_data/pudu_wanda_3f/extracted/optemap.pgm`。

语义：`private_data/pudu_wanda_3f/results/conversion_v1/semantic_map_v1.json`。

拓扑：`private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8/topology_cache`。

## 5. 实验结果与证据优先级

下面目录均相对 `private_data/pudu_wanda_3f/results/`。目录名中的时间部分曾使用预定标签，并不可靠表示实际开始时刻；真实耗时以日志/JSON 为准，不按文件名推断。

### 5.1 旧见证重新审计

`transition_r2_preflight3_20260907T063000Z/`：曾报告 3/3，但未包含回绕审计，**已被下一个审计替代，不能用于在线入口放行**。

`transition_r2_preflight3_revisit_audit_20260907T071500Z/`：目前旧见证权威判定，2/3、C1、online_implementation_eligible=false。

| 查询 | correct-side | target-band | P50 | 状态 |
| --- | ---: | ---: | ---: | --- |
| positive 旧见证 | 0.853932584 | 0.544319600 | 0.387184411 m | 两处自交，拒绝放行 |
| negative | 1.0 | 0.731132075 | 0.249999970 m | 严格重放通过，长 10.553371 m |
| south | 1.0 | 0.737756714 | 0.149999946 m | 严格重放通过，长 59.464220 m |

positive 旧见证两处自交约围出 9.3 m、5.7 m 路径。正式密集交点以 `*_audit.json` 中 revisit_audit 为准。

### 5.2 新请求驱动候选

| 目录 | 方法与结果 |
| --- | --- |
| `transition_hybrid_positive_r2_20260907T064000Z/` | 60 秒超时；搜索半径 .40 与重建 .401 不一致，排除为正式结果 |
| `transition_hybrid_positive_r2_radiusfix_20260907T070500Z/` | 修正为 .401 / 2.493765586 后仍 60 秒未找到路径 |
| `transition_roadmap_positive_r2_20260907T070000Z/` | 原采样，约 2.7 万 edge attempts；诊断不完整，非权威 |
| `transition_roadmap_positive_reachable_r2_20260907T074000Z/` | 68 sites / 670 poses / 18,150 attempts / 10,221 edges；234 完整候选中 228 自交拒绝；保留安全路径长 8.497 m，但 side/band=0，P50=3.90 m；约 10.12 秒 |
| `transition_map_optimize_positive_r2_20260907T073000Z/` | 90 秒，4,326 候选，791 collision-free，61 无回绕，语义通过 0 |
| `transition_map_optimize_positive_r2_360s_20260907T075000Z/` | 预算 360 秒，实际 297.1125 秒；13,863 候选，3,277 collision-free，553 无回绕；找到数值通过路径，但返访必要性待核验 |
| `transition_candidate_independent_replay_20260907T064200Z/` | 新候选独立证书重放、安全与非自交通过；不含“相邻轨迹折返是否必要”的充分审计 |

新候选数值：长度 31.152675 m；side 0.835723598；band 0.547588005；P50 0.425988376 m；精确端点；最大控制曲率 2.493765586；全 padded footprint 通过；显式方向错误/禁停端点/倒车/原地旋转为 0；证书确定性重放通过。

**必须查看这张真实图，不能仅看 JSON 的 `gate_passed`：**

![新候选实际折返与语义窗口](../private_data/pudu_wanda_3f/results/transition_candidate_independent_replay_20260907T064200Z/path_and_semantics.png)

这不是碰撞绕障所必需的路线：原请求有约 8.5 m 的安全直达路径，但不满足语义。当前新候选为采集足够目标带长度而到目标外折返，是否构成用户禁止的无意义往返尚未被证据化解决。不能自行放松这一条，也不能因此声称连续空间无解。

## 6. 当前源码入口

均在 `PKG/arena_evaluation/`：

- `semantic_transition_contract.py`：纯 R2 统计。各类分母分离；短路径全程；固定采样；空类和非法字段失败关闭。
- `semantic_transition_r2_preflight.py`：旧见证重放、hash 和 exact endpoint 绑定、hard features、回绕筛查；当前重新运行会给 2/3。不要改全局 WITNESSES 使旧证据冒充新证据。
- `semantic_path_revisit_audit.py`：全点 piecewise-linear 非局部交点、共线重叠、零位移检查。**仅为筛查，不检测所有相邻往返或证明绕行必要性。**
- `semantic_transition_hybrid_probe.py`：复用 48-bin ExplicitSE2GuideOracle；半径已修正；不是在线接口。
- `semantic_transition_roadmap.py`：地图派生稀疏 SE(2) 路网，权重 Dijkstra + portal 候选；已加起点可达 component 过滤；proper_crossings 使用稀疏点，仅用于候选淘汰，最终必须用严格审计。
- `semantic_transition_optimize.py`：0.05 m 可行栅格图 + 目标连通区 + 地图派生 portal 控制点 + SciPy differential_evolution。无历史 witness 输入或硬编码端点坐标。其 `gate_passed` 只覆盖已实现审计，尚不能证明往返必要性。
- `semantic_transition_candidate_verify.py`：新候选独立证书重放、hard/features/semantics/revisits 审计及真实图生成。没有安装 entry point，使用 `python -m`。
- `semantic_constraint_core.py`：ConstraintWorld、Dubins edges、完整 padded rectangle 检查、canonical PathAudit、控制重放。

`ConstraintWorld` 是 targeted 的单 lane 诊断适配器，**不适用于 mixed lane/parking selected8**。

setup.py 新增了四个 entry points：`semantic_transition_r2_preflight`、`semantic_transition_hybrid_probe`、`semantic_transition_roadmap`、`semantic_transition_optimize`。保持其他用户修改。

### 运行示例

所有 `--output` 必须换成未存在的新目录。

```bash
cd /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation
/usr/bin/python3 -m arena_evaluation.semantic_transition_optimize \
  --inputs /home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/constraint_inputs_20260907T021418Z \
  --query r3-mirror-1-positive \
  --output /home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/NEW_UNIQUE_DIR \
  --timeout 360 --seed 20260907
```

## 7. 下一步的工程顺序

1. **先解决真实验收缺口**：审计新 positive 轨迹的非局部近距返访、朝向、可移除回路/安全短接及其语义贡献。不要把“无几何自交”偷换为“无不必要往返”；也不要任意加新阈值当作已批准协议。记录审计证据和仍缺少的契约定义。
2. 若能找到原批准规则下没有不必要往返的路径，才将该新见证和 negative/south 放进新的三条权威聚合；严格绑定输入/输出 hash，旧聚合只读保留。
3. 当前 297 秒是离线可行性探索，不能作为在线合格。可探索目标带几何骨架/走廊分解、地图派生 deterministic seeds、减少连续优化维数；不能把这条见证缓存成在线请求的答案。
4. 只有三条真实离线门槛成立后，才命名新架构并实现真正 request-generated 在线 adapter；消费 SE(2)/trajectory corridor/路径级预算，输出 nav_msgs/Path，接 canonical PathAudit。
5. 使用现有真实地图准备管线适配 selected8。可参考 `two_layer_v2_semantic_r1_benchmark._prepare`、route orientation、R3 RegionalPreferenceBuilder、R2 composer 和 ExactSemanticSmacSessionR2。不得把 lane-only NPZ 当作完整语义场。
6. 做同轮 48-bin E0 / E4-r3-compatible / 新候选公平在线对比、非空 exact effective-content ACK、独立进程/domain 冷时延、selected8 每臂每条 1 warmup + 3 measured。
7. 新候选 selected8 必须 24/24 measured final-valid、无 E0 逐次成功回归、安全违规 0；记录所有 R0–R4 层级与原因，不能把放宽后有效等同 R0 语义达标。
8. 冷请求 candidate/E0 P50 <=2.0x，目标 <=1.5x；预计算/启动/内存单报。全部 B 门槛同时满足才算 B；未做 30–50 query 且 >=100 有效样本的正式扩样不宣称 A 或生产可用。

## 8. 验证现状

- 最新专项：**26 passed in 0.28s**，覆盖 R2 统计、回绕筛查、入口失败关闭、地图派生 seeds 和 roadmap。
- 全包最近一次：**429 passed / 1 failed / 6 warnings，86.25 秒**。唯一失败是 roadmap 测试 SimpleNamespace 未提供新使用的 `world_to_cell`，测试替身已修复，26 项专项已确认。**修复后尚未重跑全包，不得写全包已全通过。**
- 该次 JUnit：`results/transition_r2_tests_20260907T064000Z.xml`。
- 隔离 colcon build：通过，1 package，约 5.23 秒；只有既有 OMPL C++ warnings。
- 隔离目录：`/tmp/pln02_transition_r2_build_20260907.9ZJFgM/`，没有覆盖主 install。
- 安装后的上述四个 CLI `--help`：通过。
- candidate_verify.py 是构建后新增，最后的测试替身修复也在构建后；最终全包/compileall/最终 build 仍需补跑。
- 本轮根仓与嵌套仓 `git diff --check` 已通过，最后交接文件写入后需再查。
- 交接收尾再次完成两仓 `git diff --check`，以及全部七个新模块的 `compileall`，均通过。
- 2026-09-07 06:57 UTC 检查无本轮 semantic_transition/semantic_constraint/pytest/colcon 运行进程；所有工具 session 已收取结束结果。
- 本轮没有启动 ROS/Nav2/Smac；未清理或终止其他任务/旧可视化进程。

## 9. 保存位置

本轮关键源码、测试、setup/config 副本与主仓/嵌套仓的 staged/unstaged patch 均保存在：

`private_data/pudu_wanda_3f/results/semantic_handoff_20260907T065717Z/`

patch 是只读快照，**不要在当前已经包含这些修改的工作树上重复 apply**。它们包含原有用户修改，不代表全部由当前语义工作产生。没有 commit/push。

本文件是最新入口。先前“3/3 已满足在线入口”及新候选 JSON 数值通过声明，都必须结合本文件所列尚未完成的往返必要性审计理解。
