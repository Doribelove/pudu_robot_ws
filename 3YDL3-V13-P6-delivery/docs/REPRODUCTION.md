# P6工程交付与复现

交付版本为 **3YDL3-V13-P6 / H-I / local**。本轮使用固定公司提交`d8c01c5a71a67c2e85c250e8a8148bda9724dc68`中的P6代码，在原十图100个配套Q上进行三轮首次导航回归。原B的结果保留历史标签，本轮P6结果独立统计。

## 文件与运行入口

| 目录/文件 | 用途 |
|---|---|
| `source/3YDL3-V13-L2/` | 固定公司提交副本；449个发布文件保持原字节 |
| `runtime/3YDL3-V13-L2/` | 实际执行包；在副本中补齐原P6参数与启动资产 |
| `docs/RUNTIME_ASSETS.json` | 16个补充资产的原路径、SHA与用途 |
| `docs/DEPENDENCIES.json` | ROS/P5环境链、Python版本、十图原Q、语义/交通输入及外部路径 |
| `tools/check_environment.py` | 只读环境检查；不启动ROS或导航 |
| `tools/run_static_regression.py` | 固定源码/输入核验、导入检查、试跑、正式三轮的统一入口 |
| `tools/p6_static_harness.py` | 继承原评测口径，实际调用P6 `IncrementalService`及`IncrementalController` |
| `PROTOCOL.json` | 本轮请求数、计时、硬约束与验收条件 |

公司发布包原来缺少连续L3配置、Nav2启动文件和部分资源。`runtime`补入原P6的`l3.json`、端点容差及区域配置、`curve_nav2_stack`、两个launch脚本、语义扫掠库、DDS配置及8份评测YAML，共16项。全部新增资产来源登记在清单中；求解器与参数数值沿用原P6。

新运行适配器把`semantic_world.WORK`和`semantic_pipeline.WORK`绑定到运行包的`resources/semantic`，确保语义扫掠库和通信配置从交付目录取得。启动脚本先加载基础环境，再恢复本轮明确登记的`THREEYD_BENCH_SOURCE`、索引/分块目录和实验开关，避免旧环境覆盖新输出位置。

本轮使用全局规划服务与Nav2成本图同步。障碍新增/清除实验和车辆闭环实验分别沿用已封存证据，本轮静态回归不重复执行它们。

## 本机执行

在Bash中使用系统Python3.10。下面`P6_CLOSEOUT_ROOT`指向本轮交付目录，`P6_REPLAY_ROOT`每次取一个全新目录；脚本遇到已存在的输出会停止。

```bash
export P6_CLOSEOUT_ROOT=/home/robot/workspaces/3ydl3_v13_p6_closeout_20261109
cd "$P6_CLOSEOUT_ROOT"
export V13_PROFILE=p6
source "$P6_CLOSEOUT_ROOT/runtime/3YDL3-V13-L2/env.bash"
/usr/bin/python3 "$P6_CLOSEOUT_ROOT/tools/check_environment.py"
```

只读检查覆盖本机Linux x86_64/Python3.10、已登记数值依赖版本、ROS消息包、P5环境链、地图配套输入、P6模块路径优先级、L3配置SHA、16项补充运行资产SHA和原生库位置。2026-11-09补齐运行资源后，本机只读检查104/104项通过。它不导入求解器、不启动节点、不写输出。

验证固定源码及全部地图输入，再检查实际导入类、配置：

```bash
export P6_REPLAY_ROOT=/tmp/p6_replay_fresh_20261109_01
/usr/bin/python3 tools/run_static_regression.py --phase verify
/usr/bin/python3 tools/run_static_regression.py \
  --phase check --output "$P6_REPLAY_ROOT/import_check"
```

先运行独立试跑，核对实际P6服务、成本图ACK、路径终验和退出清理：

```bash
/usr/bin/python3 tools/run_static_regression.py \
  --phase pilot --output "$P6_REPLAY_ROOT/pilot" --domain 226
```

试跑通过后进行原100Q三轮正式回归：

```bash
/usr/bin/python3 tools/run_static_regression.py \
  --phase formal --output "$P6_REPLAY_ROOT/formal" \
  --pilot-result "$P6_REPLAY_ROOT/pilot" --domain 226
```

每批一个地图的10个原Q，每批新进程；共30批、300次完整请求，CPU14串行执行。三个轮次轮换地图顺序。`ROS_DOMAIN_ID`应选同机空闲域，避免与其他运行中的ROS实验混用。继续阅读结果时使用输出目录，复跑时创建新的`P6_REPLAY_ROOT`，不重启历史队列或复用旧输出。

每批`P6_RUNTIME_BEFORE.json`和`P6_RUNTIME_AFTER.json`记录实际加载模块与SHA；每条请求记录服务/控制器/L2状态类。`runs.jsonl`保存原始指标，`summary.json`和监督日志保存完成及清理情况。全批的`COMPLETE.json`表示运行完成，功能验收仍逐条检查300次成功、碰撞/运动学/交通/ACK及原Q绑定。

## 运行环境与数据交接

本机已验证组合为Linux x86_64、glibc2.35、ROS2 Humble、Python3.10.12、NumPy1.24.4、SciPy1.8.0、OSQP0.6.7.post3、PyYAML5.4.1、psutil5.9.0、Pillow9.0.1。`runtime/base/deps`保留随包数值依赖，`runtime/base/native`及`runtime/base/runtime/base/native`保留匹配的原生库和源码。底层ROS/Nav2使用本机已构建环境及snapshot成本图库。

公司包`env.bash`支持`V13_RUNTIME_ENV`指定基础环境，默认路径为：

```text
/home/robot/workspaces/3ydl3_v13_p5_recovery_schedule_20261027/env.bash
```

该脚本顺次加载P4、P2d、P1、V13原环境、旧三层运行环境，最后接入`/opt/ros/humble/setup.bash`与`/home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash`。完整链写在`DEPENDENCIES.json`的`runtime_env_chain`。这里历史目录主要提供ROS/启动环境及评测工具；运行时算法模块必须优先来自本次固定包，实际加载审计会核对。

代码仓库未上传原地图。交接时请项目负责人提供以下内容：

1. `DEPENDENCIES.json`列出的全部地图图像/YAML、原Q JSON、语义与拓扑JSON，以及万达2w有效地图审计和绑定文件。
2. 原基线`MAPS.json`及B的评测工具。本轮使用它们固定数据与统计口径，求解器由P6运行包提供。
3. 现有ROS/Nav2编译环境、P5基础环境脚本及其所引用的本地库。异构平台需重新构建原生库，另做一次环境与结果复核。

原输入引用分布在以下根目录：

| 数据根目录 | 内容 |
|---|---|
| `/home/robot/workspaces/3ydl3_v8_20261011/inputs` | 万达3w、万达2w有效地图、Arena中/大图、导师原图/四倍面积图及原Q |
| `/home/robot/workspaces/3ydl3_v8_crossmap10_20261011/inputs` | Arena小图及原Q |
| `/home/robot/文档/ChatGPT/dddd-任务/output/3yd_fixed10_20260922` | 万达2w原配套配置、合成A/B/C及原Q、fixed10绑定器 |
| `/home/robot/workspaces/3ydl3_v12_r0_completion_b_20261020` | 十图权威`MAPS.json`、本轮继承的评测/监督工具 |

十图内仍保留`Q01–Q10`或原`A2B-01–A2B-10`标识。万达3w内部ID为`mengjia`，万达2w为`pudu_wanda_3f`。万达2w使用已审计有效地图，替换成原PGM会改变问题输入。

跨机最直接的复现方式是在隔离环境中保持这些绝对路径，或将授权数据目录挂载至相同位置。只修改`V13_RUNTIME_ENV`能够更换基础环境脚本，无法同时重写旧地图加载器和评测工具中的绝对路径。需要重定位时，使用单独的适配配置/新运行目录并重新登记输入SHA与模块来源，保留固定`source`作为参照。

拓扑索引和分块场是派生产物，通过`THREEYD_V12_INDEX_ROOT`与`THREEYD_V12_PLANE_ROOT`配置。新回归入口将它们放在本轮`storage`目录；原地图/Q/交通输入是交接的权威数据。生成或加载这些准备数据单列成本，在线首次规划计时从请求开始。

## 原始P6事件证据的复现入口

公司包中的`tools/original_p6/reproduce.py`保留了历史脚本。它按原目录布局定位`docs/PROTOCOL.json`、`registered_events`和原P5工具，并检查原绝对路径源码锁；发布包中的归档布局不是该脚本原执行布局。

在本机完整历史目录中，可以用以下方式新建一条Arena事件复现：

```bash
source /home/robot/workspaces/3ydl3_v13_p6_hint_recovery_20261028/env.bash
/usr/bin/python3 \
  /home/robot/workspaces/3ydl3_v13_p6_hint_recovery_20261028/tools/reproduce.py \
  --output /tmp/p6_event_fresh_20261109_01 \
  --case arena_short --arm H-I --version local --xlarge
```

这条命令读取原5m障碍注册位置并保存新的运行结果。原始P6报告、三轮四尺寸对照、节点诊断继续使用封存记录。`H-A`是同框架重跑A*对照，`H-I`是增量D*；`local`为采用的局部扩域，`tube`为整条管带扩域消融，`baseline`为P5。

## 结果口径与交付完成条件

导航耗时只统计地图和服务准备之后的每次首次完整规划，包含失败尝试、恢复、L1/L2/L3、同步ACK和终验。地图加载、通用初始化、请求外导出分别记录。CPU为20 ms采样的工作进程与ROS后代累计差，另附worker精确process_time；RSS为同一规划进程树采样峰值，按地图面积排序报告。精确成功与失败后0.20m/30°容差成功分别列出。

交付时检查：固定版本/配置正确、300条原Q完整、成功输出零碰撞和零运动学违规、交通与ACK通过、失败原因可读、所有资源进程已退出、代码/输入/结果索引齐全。原B/G研究证据及P8/P9扩展结果的角色见`VERSION_LINEAGE.md`；本轮最终状态以总报告和最终审计为准。
