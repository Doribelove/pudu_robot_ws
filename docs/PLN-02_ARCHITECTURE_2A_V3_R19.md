# PLN-02 Architecture 2A-V3 r19

## 结论

本轮实现了 `r19-goal-coreachable-corridor`：目标可达性优先的有向 SE(2) 图搜索、精确终点 yaw 的局部连接、保持原目标函数的剪枝，以及自然性审计分歧的失败关闭。

结果是**有实际进展，但 sentinel 未通过，不能替代 r13 的研究型 B 基线，更不能晋升 A/生产**。x32-29 首次在本轮限制内现场生成完整停车路径；新 aisle 口径的停车中心带占比 85.41%、P50 0.07936，通过研究门槛。但途经 lane 的靠右比例只有 67.13%，且原始自然性检查提示绕行需要解释。本轮不启动三查询复验、在线 ROS、selected8 或 expanded32。

旧 R2 parking component 指标没有改变：本路径中心带占比 6.69%、P50 0.66463，仍失败。新 aisle 指标单独命名 `parking-route-local-aisle-normalized-r3-research`，不是已批准 R2 的替代验收。不得把新停车单项通过写成正式 B 通过。

## 冻结与范围

- architecture：`2A-V3`
- revision：`r19-goal-coreachable-corridor`
- protocol：`PLN-02-2A-V3-R19-GOAL-COREACHABILITY-V1`
- 主仓 branch/HEAD：`codex/pln-02-2a-v2-3d-v1-r1` / `ed25b1767976ecb48086bfa429b2b5a3a49d7226`
- evaluation branch/HEAD：`humble` / `94762429bea19b84cab50a3d0910a736184738a0`
- pinned Nav2 HEAD：`656ae8d4c56978efbdd446fe85582f2bcd06e920`
- map：`05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a`
- semantic map：`2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553`
- expanded32 内容 hash：`42464f9e99d3a017e63dde3c9559944698deccf732b2d03a0f56110d814dfd02`
- sentinel：`x32-29-cmp2-08-parking-internal`
- start：`[-19.750998999999993, 73.06689600000001, -0.4636476090008061]`
- goal：`[-20.900999, 34.066896, -0.7853981633974483]`

完整读取并遵守 evaluation/AGENTS.md，保留主仓、evaluation 和 Nav2 已有 dirty/staged 内容。本轮只新增独立文件，不改父源码、pinned Nav2、setup.py、查询、端点或语义地图；不进行 3D-V1、动态规划或飞书工作。没有 reset、clean、checkout、stage、commit、push 或终止其他任务进程。

## 对父轮诊断的更正

旧报告和结果不改写；以下是本轮新增证据。

1. `maximum_station_skip=2` 的原代码实际允许连接后面第 1 和第 2 层。上一轮把它解释为“只能下一层”的说法不正确。本轮新增专门回归，r18 的桥接诊断继续保持排除，不把它作为已证明的根因。
2. 原可达性诊断只记录 start-reachable 子图内的入边，其 `goal_coreachable_state_count=1` 不能证明全候选图没有到 goal 的边。r19 的反向分析独立遍历全部候选层。
3. 本轮发现真正的终点构图缺口：固定精确 goal yaw 下，倒数第 1/2 个采样层的最短 Dubins 连接分别约 2.712/2.995 m，超过原 1.75 m 局部边限；更早一层存在约 0.947 m 的局部连接，却因相差 3 个 station 层未被枚举。r19 仅对剩余 route station ≤1.75 m 的状态显式连接精确 goal；边长、比例、footprint、master 和全部运动学检查保持原值。

这证明旧离散图存在连接遗漏，不证明连续空间不可行，也不是事后将查询改成语义不适用。

## 实现与正确性边界

搜索状态仍是 48-bin、forward-only DUBIN 的 SE(2) 状态；沿 route station 正向运动，反向仅指计算 goal-coreachability 的分析顺序，不是车辆倒车。原 station 采样、每层候选位置、yaw offsets、Rmin ≥0.40 m、最大曲率 2.50 1/m、完整 padded footprint、安全层、精确端点和局部边长/比例都保留。

在有限 DAG 上，为每个状态保留一个最低加性代价的到 goal 后缀。与父搜索先截断少量局部后继不同，所有通向 goal-coreachable 状态的合法局部边均参与比较。最多 12,000 个展开状态，图搜索墙钟上限 30 s；超时不返回 partial candidate。这个状态预算不等同于父版本的 Pareto-label 预算，不能据此宣称两种算法计算工作量相同。

优化只使用非负加性目标的下界：精确 Dubins 长度加已知 suffix 代价；进一步提前计算同一个 semantic edge statistic，淘汰代价严格劣于已知可行边的候选。保留的边仍完整执行原 `_edge` 检查。小图随机 DAG 和真实 Dubins 边统计的剪枝开/关 parity 通过；真实地图不剪枝版本超时，因此未宣称大图穷举 parity 已测得。

单后缀只对固定加性目标最优，不保证 lane 比例、parking P50 等整条路径的非加性验收最优。最终依赖未降低阈值的整条路径审计，语义失败不能被装饰成 fallback 成功。

### 自然性必须单独解释

最终路径无几何自交/重复占用回访，精确控制重放和有序 route-station 检查通过，且没有越过 goal plane。但是原始自然性检查报告约 1.967 m 累计 route-progress 回退，其中一个 6.20 m 弧段对应约 1.747 m 回退；状态为 `DETOUR_JUSTIFICATION_REQUIRED`。

父版把 terminal/nearest-route 几何检查保留为诊断，以避免弯曲路线的投影误报。本轮没有证据证明此处究竟是投影歧义还是不必要绕行，因此不能把“有序投影通过”和“无自交”写成“没有不必要往返”。r19 新增 `R19_NATURALNESS_REVIEW_REQUIRED` 失败关闭：未提供独立解释前不准验收。该提示也不被误报为碰撞、安全违规或连续空间不可行。

新增 guard 后重新现场运行，路径文件 SHA 与 guard 前完全相同：

`315c19423e81c8f58add55522209e46c87a43bbb936e900a32fb361c8d68575c`

这次变化是审计结论的完整性，不是重新调路径做过线。

## 单查询逐阶段结果

下表均为离线、单次、同一冻结查询的开发诊断，不是在线 E0/E5 配对性能数据。

| run | 假设 | 图搜索 | 请求墙钟 | 结果 |
|---|---|---:|---:|---|
| v1 | 全图反向可达性，原终点层连接 | 很短 | 6.942 s | goal 无合法入边；无路径 |
| v2 | 增加局部精确 goal attachment | 30 s 上限 | 35.858 s | 超时；无 partial |
| v3 | 加长度下界剪枝 | 30 s 上限 | 35.315 s | 超时；无 partial |
| v4 | 加同目标函数 semantic prefilter | 16.363 s | 22.020 s | 现场完整路径；lane gate 失败；自然性提示未解决 |
| v5 | 同搜索，加入自然性失败关闭 | 16.662 s | 22.234 s | 路径 SHA 不变；lane + naturalness review 失败 |

最终 v5：2,952 个状态展开；2,314 个 goal-coreachable 状态；保留 2,313 个后缀 control；11,831 次完整 primitive 检查；长度下界剪枝 27,133 次、semantic dominated 剪枝 61,939 次。静态准备 0.593 s，最终审计 0.231 s，peak RSS 1,422,446,592 B。crop 数组 17,454,240 B；全进程 RSS 仍包含其他静态图/审计缓存，不能用 crop 大小代替 resident memory。

本轮没有 E0 冷进程配对、warm/cold 分布或 RSS soak，不能宣称延迟比或内存生命周期门槛已达标。所有上述分位数均不适用，未输出正式 P95/P99。

## 最终路径的安全与语义

| 项目 | 最终 v5 | 结论 |
|---|---:|---|
| canonical final-valid / footprint / kinematic | true / true / true | 通过物理检查，不等于语义总验收 |
| 最大曲率 | 2.4937655864 1/m | 通过 |
| reverse / 原地旋转 | 0 m / 0 | 通过 |
| start/goal XY/yaw 误差 | 全部 0 | 通过 |
| effective master、硬语义、no-stopping endpoint | 通过 / 通过 / 0 违规 | 通过离线检查 |
| 弧长 / 上限 | 57.21465 / 73.81492 m | 通过 |
| exact primitive replay、ordered progress、revisit screen | 通过 | 不替代绕行必要性审计 |
| 新 aisle parking 中心带 | 562/658 = 85.4103% | 研究门槛通过 |
| 新 aisle parking deviation P50 | 0.0793563（无量纲） | 研究门槛通过 |
| 旧 R2 component 中心带 / P50 | 6.6869% / 0.6646324 | 失败，口径未改 |
| 全 lane 靠右 / error P50 | 774/1153 = 67.1292% / 0.0341641 m | 靠右失败，不用小 P50 掩盖 |
| 原始自然性与 constrained projection 分歧 | 未独立解释 | 失败关闭 |

lane-instance 复核定位如下。每端 6 m 的冻结排除窗口保持；短路径规则未改变。另用路径本身的 yaw 发出左右法向射线，直到同一 semantic lane instance 的真实边界，作为独立方向交叉检查（不是障碍距离或安全间隙）。

| lane instance | active samples | 原 field 靠右 | error P50 | 路径 yaw 射线 d_right/d_left P50 | 射线靠右 | 含目标 state 的层 |
|---|---:|---:|---:|---:|---:|---:|
| 1 | 776 | 99.7423% | 约 0 m | 0.45 / 6.10 m | 100% | 39/39 |
| 4 | 377 | 0% | 5.4000 m | 5.80 / 0.35 m | 0.2674%（374 个有效 probe） | 11/24 |

两种方向判定的 agreement 都约 99.7%；lane 4 不是通过四舍五入、换定义或加大误差容差能合理解决的问题。证据显示实际路径走在错误侧，局部 state 层的目标覆盖也不足；尚未证明是采样支持、phase 转接或全局路径资源选择中的哪一项使合格路线不可达。不能据本次 planner miss 把查询重新分类为不适用。

## 停止条件与下一步

- 本轮 strict sentinel：`FAILED`，仅验证 x32-29，不能把 r17 的另两条通过拼成 r19 的 3/3。
- 三查询复验、在线接口、同轮 E0/E5、exact server ACK、selected8、expanded32：`NOT_RUN_SENTINEL_GATE_FAILED`。
- 本轮 master 是绑定的 **offline expected effective master**，不是观测到的 server master；ACK 必须明确为 NOT_RUN，不能借用历史零 mismatch。
- 最终状态：`R19_SENTINEL_GATE_FAILED; R13_B_RESEARCH_ACCEPTANCE_PRESERVED; NOT_A; NOT_PRODUCTION_PROMOTED`。

下一步仍在 2A-V3 内聚焦多 phase 方向连接，不回到 2A-V2 调 soft cost：先独立标注 lane 4 的真实可达目标带和进入/离开 parking 的转接区，验证连续 forward-Dubins 连接；同时对自然性告警弧段做无约束原始路线投影与障碍必要性检查，不能用限定 station 区间的投影自证无回退。若存在合格连接，再让搜索显式保留 lane/parking 资源可行后缀，而不是只保留一个最低加性代价后缀。仍先做这一个 sentinel；不能证明时按同一 fast gate 停止，不扩大预算或回到小时级扩样。

## 测试与构建

- 最终 `/usr/bin/python3 -m pytest -q .../arena_evaluation/test`：**781 passed，6 个已有 warning，89.30 s**；r19 专项 14 passed。
- 测试覆盖全图 goal-coreachability、dead-end 分支、两层连接范围、精确 goal pose 与局部连接窗口、timeout 无 partial、稳定 tie-break、随机 DAG 剪枝 parity、Dubins semantic prefilter parity、父契约绑定、语义失败不装饰、自然性缺失/分歧失败关闭。
- 相关 r0–r18 安全/ACK/kinematic/footprint/自然性回归包含在完整测试中；未进行新的动态实验。
- compileall、root/evaluation `git diff --check`、两个 r19 module CLI `--help`：通过。
- isolated colcon：`arena_evaluation_msgs` 和 `arena_evaluation` 通过，7.07 s；构建目录 `/tmp/2a_v3_r19_colcon_Ywye2N`。已有 OMPL 编译 warning 保留。使用 symlink install，最终 Python 修改经 installed-module help/compileall/full pytest 验证。
- 权威解释器 `/usr/bin/python3` 3.10.12；ROS Humble。完整测试用工作区已有 Python 3.10 dependency path 提供 scikit-image，没有安装软件或修改环境；真实地图实验不引入该测试 dependency override。
- r19 未启动 ROS/Nav2，离线 benchmark 均已退出；不 kill 其他任务进程。

## 复现、文件与哈希

主结果：`private_data/pudu_wanda_3f/results/2a_v3_r19_failclosed_sentinel_v5_20260909T034600Z`。

独立边界与图形证据：`private_data/pudu_wanda_3f/results/2a_v3_r19_final_geometry_v3_20260909T034700Z`，其中 `path_lane_and_goal.png`、`lane_instance_audit.json` 仅做已生成路径的复核，不是规划输入。

汇总与验证：`private_data/pudu_wanda_3f/results/2a_v3_r19_closeout_v1_20260909T034900Z`，含 runs.csv、gate_results、performance_summary、artifact/source 校验、日志、最终测试、process/git audit 及 source snapshot。名称内时间后缀仅作唯一目录标识；测量耗时以 result.json 为准。

所有先行目录保留并排除晋升：

- `2a_v3_r19_goal_coreachability_sentinel_v1_20260909T033500Z`
- `2a_v3_r19_terminal_attachment_sentinel_v2_20260909T034500Z`
- `2a_v3_r19_terminal_attachment_bnb_sentinel_v3_20260909T035500Z`
- `2a_v3_r19_terminal_attachment_prefilter_sentinel_v4_20260909T040500Z`
- `2a_v3_r19_postrun_lane_geometry_v1_20260909T041500Z`
- `2a_v3_r19_postrun_lane_boundary_v2_20260909T042000Z`

只新增以下源码/测试/配置及本报告：

1. `arena_evaluation/arena_evaluation/semantic_goal_reachability_r19.py`
2. `arena_evaluation/arena_evaluation/two_layer_v3_semantic_r19.py`
3. `arena_evaluation/arena_evaluation/two_layer_v3_semantic_r19_evidence.py`
4. `arena_evaluation/config/two_layer_v3_semantic_r19_goal_reachability.yaml`
5. `arena_evaluation/config/two_layer_v3_semantic_r19_terminal_attachment.yaml`
6. `arena_evaluation/test/test_semantic_goal_reachability_r19.py`
7. `docs/PLN-02_ARCHITECTURE_2A_V3_R19.md`

evaluation 相对路径以 `external/arena4_ws/src/arena/evaluation/` 为根；结果目录另有可复验汇总脚本 `closeout.py`。父 r17 的 25 项 artifact manifest 逐项复核，父源码 hash 与最终 snapshot 绑定，结果源漂移为空。关键未改动父哈希：

- r17 报告：`c6ac7adb4cac7e1caecefa6aa05d53316606c3454af5d7ca41cceaeedc0bd365`
- r17 config：`dc99fe3a87e25871b3b151dab1dea757b3f5f9c27b8200353e9405742c7fc64c`
- r17 runner：`f7fb9cd0d966ca1addbadba3056aaa42f33c25f4b73d686164c4a7d4211d0039`
- r17 aisle builder：`6612dd56409ba13b3f5eef71b07af151d3ee982b4799fb636afdbbfd29bc4bb2`
- r17 final_result：`21acaed5bffd9054f8747ab3149306ac32b5eb849001b15f161762d14cd25033`

```bash
source /opt/ros/humble/setup.bash
source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash
export PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation:$PYTHONPATH
/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r19 \
  --config /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/config/two_layer_v3_semantic_r19_terminal_attachment.yaml \
  --output <全新且不存在的目录>
# 预期 exit 2：真实失败关闭，不是执行异常。
```

最终保护：未 stage/commit/push，未改 pinned Nav2，未覆盖任何父实验，本轮不生产晋升。
