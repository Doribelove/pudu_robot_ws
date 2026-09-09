# PLN-02 路径资源约束可行性研究 r0

日期：2026-09-07。协议：`PLN-02-CONSTRAINED-FEASIBILITY-R0-V1`。

结论：**C1，Stage 1 仅 2/3 targeted 查询通过，未达到 B。** 本轮没有正式定义或实现 2A-V3，没有启动在线 adapter、E0/E4/V3 公平对照、selected8 或扩样实验。正例的连续空间可行性仍未确定。

## 1. 冻结身份与输入

本研究只处理静态 A2B 全局参考路径。已完整读取用户指定六文件、适用 `evaluation/AGENTS.md`、P0 和统一实验协议。旧 r0–r4 源码和结果用于输入审计；新候选均重新计算。

Stage 0：`private_data/pudu_wanda_3f/results/constraint_stage0_20260907T021418Z`。

| 绑定 | 值 |
|---|---|
| Stage 0 protocol SHA-256 | `dfe0955c24873f6d5fd926c2954b3f93e08779ca44102787f9f57ef12e286d2c` |
| 地图 | `05cf18d0df40235f69ba5f0168bb490f9175541431c0c516a962e7ce1965529a` |
| 语义地图 | `2560a4f4c86a86aeaf9993262648aaeb26998948e79fe3b92ecf47b6e69d0553` |
| targeted 查询内容 | `66212b05ef6c4d16eaedafc1c27866387c3d74aa85bf5ac69bca92487254667d` |
| selected8 查询内容 | `7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e` |
| root HEAD | `ed25b1767976ecb48086bfa429b2b5a3a49d7226` |
| evaluation HEAD | `94762429bea19b84cab50a3d0910a736184738a0` |
| pinned Nav2 HEAD | `656ae8d4c56978efbdd446fe85582f2bcd06e920` |

新输入目录 `constraint_inputs_20260907T021418Z` 保存三条完整栅格与绑定。重新构建后，三条 route、R0 ROI、expected effective master 哈希均与冻结 r4 一致；master 分别为：

- positive：`c58ec26bdeb38ce8f98707688355bc0087d3157ff5facf096c22a9e559041f78`；
- negative：`8c361b30e8f3a978d5a1017b1f254a010abc9cf0f5ca2595947e92956047e25c`；
- south：`7074260f5fdb9dc6323c7cbba33a6c765e99c13c3811c9b6e8e1e068b987127f`。

硬契约仍为 0.05 m/cell、48-bin yaw 域、forward-only DUBIN、Rmin=0.40 m、最大曲率 2.50 1/m、禁止倒车和原地旋转、同 lane instance、原始起终点 x/y/yaw、R0、完整 padded Jackal footprint（±0.265 m × ±0.225 m）。靠侧比例 ≥0.80、横向 P50 ≤0.50 m、target-band 比例严格 >0.50。所有判断使用原始数值。

每个查询/变体冻结 120 秒、1,000,000 labels/candidates、2 GiB RSS。附加研究路径长度上界为 `max(40 m, 4×端点欧氏距离)`；它是有限求解范围，不是产品不可达判据。每次运行采用新目录，运行前保存配置和源码快照。目录后缀只是唯一标识；计时使用实际单调时钟。

## 2. 实现与证明范围

新增离线有限图保存 `(x,y,yaw_bin,route station)` 顶点。图顶点保留确定的连续位姿，精确起终点通过解析 Dubins 连接。内部边采用半径 0.401 m（曲率约 2.49377），为数值计算留余量；硬 Rmin 仍为 0.40 m。

每条父链保留长度以及两个整数资源：

`S = 5C − 4N`，`T = 2B − N`。

其中 N 为语义采样数，C 为 correct-side 采样数，B 为同时正确侧且横向误差 ≤0.50 m 的采样数。终点要求 S≥0、T>0；T>0 保证多数采样误差合格，最终仍独立计算 P50。

只有在**同一精确顶点与进度**，且一个前缀长度不大于另一个、两项资源均不小于另一个时，才删除被支配标签。任意共同后缀增加相同长度和资源，因此删除保留可行性。父记录不可变；不存在 r4 的连续落点混淆或同 SE(2) 历史完全塌缩。标签主体采用紧凑数组，百万标签约 25 MB，不存逐标签密集路径。

图按 station 前进，为 DAG。先构建可达边，再反向分别计算 S、T 的最大后缀资源及最短后缀长度。两项最大资源可以来自不同路径，故它们是乐观上界；仅用于必要条件剪枝，不证明可行。新增 queue 的 resource-debt 项只影响展开顺序，不用于证明或删标签，不主张最优性。

这些图是受限路径图：48-bin 域内采用显式列出的有限 heading 子集、station/lateral 候选与有界边族。0.4/0.8 m station 间隔、0.1/0.2 m lateral 候选间隔不改变底层 0.05 m 地图。没有把软语义写成 lethal，也没有宣称穷尽全部 48 个航向的连续空间。

独立路线使用新的分段 Dubins 控制和有界直接参数搜索，覆盖原范围、局部南北扩展和北方绕行。拒绝候选保存原始控制、路径、失败指标；重采样到共同表示后重新审核。没有通过重复点或无意义闭环累计分数。

## 3. Stage 1 结果

| 查询 | 本轮 witness | correct-side | lateral P50 / m | target-band | 说明 |
|---|---:|---:|---:|---:|---|
| r3-mirror-1-positive | 否 | — | — | — | 两个有限图资源上界失败；三个独立控制族未找到合格路径 |
| r3-mirror-2-negative | 是 | 1.0 | 0.2499999702 | 0.7306791569 | 新独立控制链，10.5533705679 m |
| cmp2-02-lane-south | 是 | 1.0 | 0.2500000894 | 0.6259729619 | 新独立控制链，59.4642197097 m |

两条 witness 均通过 exact start/goal x/y/yaw、R0、同 lane、完整 padded footprint、运动学、no-stopping 和控制重放。倒车、原地旋转、碰撞及硬语义违规为 0。几何最大曲率分别低于 2.50，且无大于 π 的单段转弯或 proper 自交。south 保留原目标 yaw `-1.4876550949064484`；历史 r4 oracle 曾把它量化。

多标签 lattice 还独立生成 negative witness：长度 9.2772740371 m，correct-side=1.0，P50=`0.4999999701976776`，target-band=`199/381=0.5223097112860893`。这是未经四舍五入的真实通过。反向图预处理使该变体总耗时约 49.0 s；标签展开 19 个、生成 2,358 个。该耗时不是在线性能。

south 的通用多标签图仍达到百万标签上限。其独立控制 witness 表明查询可行，同时表明通用图搜索的效率仍不足。

## 4. positive 的新证据

地图中的细长障碍带把冻结端点所在西侧与多数目标带所在东侧隔开。原 r4 二维 guide 位于该带东侧；仅有二维 guide 不能证明完整 Jackal 能从端点侧横穿。

在同 lane 且 master<253 的中心点图中，4/8 邻域可通过细缝，最短 start→任意 target→goal 分别约 15.950/14.134 m。对五个细缝的 305 条实际 padded 矩形横穿（yaw=0、0.005 m 采样、横向位置 0.001 m 扫描）全部碰撞。它们是明确的有限横穿试验，不覆盖任意 yaw、任意曲线。

| 有限图 | 可达顶点 | 边 | 构建/判定 | 含起点的 T 上界 | 重建验证 |
|---|---:|---:|---:|---:|---|
| 原范围，0.4/0.1 m station/lateral | 4,553 | 228,911 | 41.47 s | −346 | 顶点、全部边、goal 边和反向界逐项完全相同 |
| 前后扩展 8 m，0.8/0.2 m | 2,741 | 111,841 | 16.32 s | −344 | 同上 |

T 必须严格大于 0；所以这两个**明确有限图**内不存在合格路径。其结论不是超时推断，也不是连续空间不可达证明。有限图受方向子集、顶点布置、Dubins 边族和范围限制。

三个独立正例控制族各执行 120 秒，共生成 204,410 个新候选：

| 控制族 | 候选数 | 最佳拒绝路径 side / P50 / band | 统一终审 |
|---|---:|---|---|
| 原范围，seed 1 | 81,855 | 0.7886178862 / 0.9354520440 m / 0.3723577236 | 碰撞与语义失败 |
| 局部外扩，seed 0 | 69,303 | 0.8321596244 / 0.6999998093 m / 0.3274647887 | 碰撞与语义失败 |
| 北方绕行，seed 2 | 53,252 | 0.8418972332 / 1.9999997616 m / 0.2916996047 | 31.4047078282 m，安全通过、语义失败 |

北方候选无 proper 自交，但含一段超过 π 的 Dubins 转弯，已作为拒绝诊断披露。这些搜索均因预算停止；没有给出连续优化的全局最优或不可行证书。

## 5. 审计、资源与门禁

canonical PathAudit 保持原样。新增独立审计检查解析控制逐段重放、精确端点、原始有效 master、完整 padded footprint，并以 ≤0.025 m、≤1° 密集采样验证。语义沿用未修改的 SemanticPathAuditor 采样规则，额外安全采样不改变语义计分。

注意 canonical 的历史默认 footprint、yaw 采样和端点阈值较松；本轮更严格的契约由额外审计补齐。canonical 的 `minimum_clearance_m` 为中心净空代理，本轮不把它表述为真实 footprint 净空。source/patch/input 哈希在 manifest 中绑定；canonical 兼容 provenance 标签不是独立版本证据。

图证书阶段峰值约 203 MiB；百万标签搜索进程约 300–353 MiB。完整地图最终审计会增加内存，独立 witness 进程峰值约 734–736 MiB。CPU 与 RSS 均记录在逐变体结果，PSS/在线内存未测量。实验与用户既有任务并行运行，因此不能用于生产延迟结论。

| B 必要项 | 本轮状态 |
|---|---|
| 三条离线 witness | **失败：2/3** |
| 三条在线 V3 | 未运行，Stage 1 门禁 |
| 同轮 48-bin E0/E4/V3 公平对照 | 未运行 |
| 服务端 exact effective-content ACK | 离线不适用；未发布 costmap |
| V3/E0 cold-request P50 ≤2.0× | 未测量 |
| selected8，1 warmup +3 measured/臂、8/8、无 E0 回归 | 未运行；该集合仍为选择偏置回归集 |
| 30–50 查询及 ≥100 有效样本 | 未运行 |

没有在线 action success、ACK 成功或延迟达标声明。离线 2/3 不能表述为 V3 在线成功率。

## 6. 验证与工作树保护

- r0–r4 加新增失败/回归测试：67 passed；原源码环境全包 385 passed，最终隔离源码副本全包亦为 385 passed。
- 新增 8 项测试覆盖资源支配、不同进度不合并、off-bin 精确 yaw、证书篡改、语义采样一致、密集安全采样、padded 角部碰撞和 write-once 输出。
- 两个 positive 图证书已完整重建并逐项比较通过；negative lattice 路径证书已重放通过；独立控制候选在交付中再次重放。
- compileall、隔离 colcon build、安装后四个 CLI `--help`、root/evaluation `git diff --check` 均通过；最终构建约 5.69 s，全包测试命令约 86.39 s。完整命令、XML 与夹具复制哈希位于 verification 目录。
- 初次 symlink build 使 setuptools 写回两个源码目录扩展库。已从哈希匹配的既有构建缓存恢复 Stage 0 原始字节；后续在完整源码副本中 symlink build，避免写回用户工作树。
- 额外 non-symlink copy-install 检查为 65 passed、2 failed，原因是旧查询模块用源码相对路径定位配置。失败保留，不修改冻结查询或用户已有模块去掩盖它；仓库原有 symlink 安装是本轮正式验证方式。
- 首个源码副本因漏拷贝仓库外层 Stage5 地图/拓扑夹具而出现 383 passed、2 failed；保留原失败目录。补全且逐文件核对夹具哈希后，在全新副本中完整重建和重跑，全包 385 passed；未修改原测试。
- `setup.py` 仅追加三个新 CLI，原有用户改动保留；索引与三个仓库 HEAD 均保持原状。旧 r3/r4 结果逐文件复核，未覆盖、清理、stage、commit 或 push。
- 本轮未启动 ROS/Nav2/规划服务；离线求解器均已结束。用户已有可视化和其他任务进程保持运行。没有修改飞书文档。

排除目录：三个 `constraint_lattice_*_v1` 在一次展开中超出百万标签上限 4–22 个，已修复精确边界检查并用新目录重跑；`constraint_independent_two_attachment_20260907T024650Z` 在记录无效候选 NaN 时序列化失败，部分结果保留，不能作 witness 证据。所有其余失败、超时与预算耗尽结果也保留。

## 7. 产物与复现

权威汇总：`private_data/pudu_wanda_3f/results/constraint_stage1_delivery_final_20260907T030500Z`，包括逐查询 CSV/JSON、逐候选族 CSV、图搜索变体 CSV、path/control certificate、轨迹与障碍叠加图、protocol/hash、目录角色、保留审计、完整验证副本、新增源码快照和门禁结论。先前 `constraint_stage1_delivery_20260907T025500Z` 是最终验证完成前的快照，保留但不作为最终交付索引。

独立搜索汇总：`constraint_independent_summary_20260907T024000Z`。

正式构建/测试：`constraint_verification_mirror_final_20260907T030000Z`；首个不完整夹具副本：`constraint_verification_mirror_20260907T025000Z`；先前验证及 copy-install 失败 XML：`constraint_verification_20260907T024500Z`。最终临时源码副本和构建位于 `/tmp/pln02_constraint_r0_build.DVW7jt/final`；原始命令及日志保留。

主要图件：[positive 障碍带细节](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/constraint_stage1_delivery_final_20260907T030500Z/overlays/positive_barrier_detail.png)、[negative 合格路径](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/constraint_stage1_delivery_final_20260907T030500Z/overlays/r3-mirror-2-negative.png)、[south 合格路径（三段连续视窗）](/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/constraint_stage1_delivery_final_20260907T030500Z/overlays/cmp2-02-lane-south.png)。

在仓库根目录使用 `/usr/bin/python3`，先设置：

```bash
export PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation
```

正例有限图复现示例（输出必须换成不存在的新目录）：

```bash
python3 -m arena_evaluation.semantic_constraint_lattice \
  --inputs private_data/pudu_wanda_3f/results/constraint_inputs_20260907T021418Z \
  --query r3-mirror-1-positive --output /tmp/pln02_positive_new_run
python3 -m arena_evaluation.semantic_constraint_verify graph \
  --inputs private_data/pudu_wanda_3f/results/constraint_inputs_20260907T021418Z \
  --query r3-mirror-1-positive \
  --run private_data/pudu_wanda_3f/results/constraint_lattice_positive_v2 \
  --output /tmp/pln02_positive_new_replay
```

独立控制搜索命令、seed、输入哈希及各版本源码位于各自 protocol/source snapshot。所有位姿、资源整数和评分可重新计算；表格显示精度不参与门禁。

## 8. 下一步

**保留 C1，不晋升 B。** 本轮已建立更强的多标签可行性工具，并把正例风险收敛到带完整 footprint 的跨障碍连接及其路径资源，而不是继续做 costmap 软权重扫描。

如继续研究，应先对该障碍带及端部连接做严格的连续配置空间/全局约束可行性判定，再决定是否值得增加运动原语和图范围。它需要新的有界研究协议；现有证据不足以宣布整个冻结契约数学不可达。只有获得正例真实 witness 并让 Stage 1 全过，才有资格定义 2A-V3 和开展在线 B 级验证。
