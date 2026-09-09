# PLN-02 语义过渡区契约 R2

日期：2026-09-07。协议 `PLN-02-SEMANTIC-ENDPOINT-TRANSITION-R2-V1`；契约修订 `semantic-endpoint-transition-short-full-6m-r2`。用户已批准短路径全程统计及停车阈值补充；本文件和同名配置独立冻结，旧契约、旧实验及其 C1 结论只读保留。

## 1. 身份与输入

当前正式架构仍为 `2A-V2`，规划候选标为 `UNNAMED_SEMANTIC_PLANNER_CANDIDATE`。三条 targeted 离线严格通过后，才允许根据实际实现命名新架构；命名不代表获得 B。每个实验同时记录 architecture、candidate、implementation、contract、protocol ID 和源码/配置 hash。

地图保持原导师 `pudu_wanda_3f` 静态 0.05 m 地图、完整 padded Jackal footprint、端点、yaw、路线和 ROI 绑定。地图 hash `05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a`，语义地图 hash `2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553`。

## 2. 统计窗口与阈值

`L` 是未经删点或截断的完整输出路径弧长。`L <= 12 m` 使用 `[0,L]`；`L > 12 m` 使用 `[6,L-6]`。比较使用原始数值，不四舍五入或按 query ID 特判。以全局固定 0.025 m 弧长采样计算软指标，与输入点密度无关；完整路径安全检查另按不大于 0.025 m 和 1° 密集插值。

| 适用类别 | 冻结门槛 |
| --- | --- |
| lane | correct-side `>=0.80`；correct-side 且目标横向误差 `<=0.50 m` 的比例 `>0.50`；横向误差 P50 `<=0.50 m` |
| parking_area | normalized deviation P50 `<=0.25`；normalized deviation `<=0.25` 的中心带比例 `>0.50` |
| junction、unlabelled | 无靠右/靠中统计要求，单独报告 `NOT_APPLICABLE`，不能记为语义成功 |

停车 normalized deviation 复用原字段：`1 - min(region_clearance,map_clearance) / component_max_combined_clearance`，在冻结停车连通分量内归一化，数值越小越靠近其最大净空区域。`0.25` 是无量纲阈值，不能解读为离几何中心线 0.25 m。停车中心带比例是本修订新增验收统计。

lane 和 parking 各自用活动窗口内本类别的样本为分母，报告各类及 lane instance 指标，禁止跨类平均抵消失败。适用类别内缺失、非有限字段不能删除后计为通过；没有适用样本时标记不适用，targeted 必须存在可验收 lane 样本。

禁止为跨过 12 m 分界或提高窗口指标而重复点、改采样密度、插入无必要弧段、绕圈或往返访问同一区域。报告必须同时给出全路径/窗口指标、总长度、窗口长度、样本数及重复位姿/重复弧段审计。自然安全绕障产生的长度变化需有地图证据；不能仅因加长后指标达标判作合格 witness。

## 3. 全路径硬约束

包括被排除窗口在内的整条路径均需通过 canonical PathAudit 和 final expected-effective master 上的完整 padded footprint 密集检查：碰撞、硬语义、倒车、原地旋转及禁停端点违规为 0，精确 start/goal x/y/yaw，`Rmin=0.40 m`，最大曲率 `<=2.50 1/m`。未知区、地图外、禁行区仍不可通行；无路径时 `hard_constraints_held=null / NOT_APPLICABLE`。

必须加载与 hash 匹配的完整语义 feature，检查完整路径穿过的每项显式方向规则；不能由 lane 标签、route 朝向或 NPZ 的 hard/no_stopping 栅格推定“没有 explicit direction”。显式方向错误距离必须为 0。禁停检查覆盖冻结任务端点，包含 start 和 goal；中间普通轨迹采样点不当作停车点，也不宣称控制器运行中绝不暂停。语义 feature 证据或硬审计缺失时拒绝放行。

## 4. Targeted 与 Selected8

targeted 固定为 `r3-mirror-1-positive`、`r3-mirror-2-negative`、`cmp2-02-lane-south`。沿用冻结精确位姿、内容 hash、同 lane instance、R0 和确定性路径/控制重放，三条离线全部通过才进入在线；三条在线必须再独立满足相同门槛，不能用历史 witness 代替在线输出。

selected8 默认集仍为 `pudu_wanda_3f_semantic_compare_selected8_r2_v2`，query hash `7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e`，保持八组原顺序及拓扑长度 `>50 m`、端点净空 `>=1.5 m`。它允许跨 lane、junction、parking，不套用 targeted 的单 lane 限制。各类语义指标、R0 严格通过率与所有 R0–R4 尝试及放宽原因分别报告；保留原协议允许的 R1–R4 安全最终成功，不能把它称为 R0 严格偏好成功。

每个实验臂每条 query 执行 1 warmup + 3 measured，主候选必须 8/8 查询的全部 24 个 measured 样本 final-valid。三臂应有 72 measured 样本，按 `(arm,query_id,repetition)` 核查完整性；缺失、重复、未知行拒绝验收。E0 成功保持按 `(query_id,repetition)` 比较，不能用某条 query 的任意一次成功掩盖其他重复失败。该集合经过筛选，不支持总体成功率推断。

## 5. 在线与真实 B

同轮三臂为 E0、E4-r3-compatible、主候选，全部使用 48 yaw bins，并核对生成参数及服务端实际参数；历史 72-bin 结果不能补齐。冻结相同地图、query、footprint、预算、顺序及配置差异，所有重试共用请求总预算。主候选须有直接消费 SE(2) reference、trajectory corridor 或路径级语义约束的在线接口，输出真实 `nav_msgs/Path`；不修改 pinned Nav2 核心。

exact effective-content ACK 必须覆盖所有在线发布尝试，记录非空完整的 attempt/ACK 关联。绑定 sequence、policy/source-grid/expected-master/server-content hash 和 ROI bbox，hard/soft mismatch、stale、hash/sequence mismatch 全部为 0；no-op 只复用完整 key 的已成功 ACK。空日志、缺记录和历史 ACK 均不能通过。

冷请求主候选/E0 P50 `<=2.0x` 为硬门槛，`<=1.5x` 为目标；冷对照使用独立进程与 ROS domain，预计算、启动成本、内存另报。真实 B 必须同时完成三条离线和在线、同轮 48-bin 三臂、selected8 全重复通过、无 E0 回归、全部安全/ACK/冷时延硬门槛，以及专项和全包测试、compileall、隔离 colcon build、安装后 CLI、根/嵌套 git diff 检查和本轮进程清理。任何缺失门槛都不能判 B。

未完成 30–50 queries、每条重复且至少 100 个有效样本的正式扩样时最高 B，不宣称 A、生产可用或直接晋升；扩样达到该规模也不是自动 A。

机器可读权威配置：[pudu_wanda_3f_semantic_endpoint_transition_r2.yaml](/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/config/pudu_wanda_3f_semantic_endpoint_transition_r2.yaml)。所有实际输入、路径、控制和审计以新的 write-once 结果目录保存，不覆盖历史结果。
