# 3YD-V9-r9 核心算法源码

本分支交付已封存 r9 的核心实现，配置为 **ASCHNT+OGR+PLMX**，B/P与U关闭。
包含L1拓扑搜索、L2有界D*与软引导、L3连接及严格审计、完整路线复用、任务进度续接、地图更新传输，以及r9的数据表示优化。

`src/three_yd_v3`、`src/three_yd_v4`、`src/three_yd_v5`保留算法和继承实现。
`src/arena_3d_v1`与`src/arena_evaluation`仅保留这些算法引用的通用定义；继承评测入口和默认地图登记已移除。
`native/`为搜索、车体检查、姿态批处理与变化块编码源码；`integrations/`保存所需Nav2发布器的源码变更。

本分支不携带地图文件、场地名称、地图登记、语义标注、真实查询/停车接入坐标、轨迹、实验记录、凭据或预编译库。运行输入由调用方在本地明确提供；算法仍会生成本地缓存与日志，不能将它们加入本分支。

## 本地构建与检查

Linux、g++、Python 3.10及以上、Python开发头文件，以及`requirements.txt`列出的Python依赖。ROS部分另需已配置的ROS 2 Humble与Nav2环境。

```bash
bash build_native.bash
source env.bash
python3 -m pytest -q -p no:cacheprovider tests
```

这些检查使用程序生成的小数组和测试数据，验证搜索距离与路径一致性、数据表示、布尔聚合、标签成员判断和几何行为；不读取现场数据，也不启动完整规划实验。编译库只在本机生成并由Git忽略。

## 接入方式

`env.bash`设置相对源码路径和r9保存配置。核心接口包括：

- `three_yd_v5.topology_default`：选择语义或无语义L1实现。
- `three_yd_v5.controller.V5Controller`：L2参考与L3范围准备。
- `three_yd_v5.fullroute_service.FullRouteService`：完整路线规划与重复请求。
- `three_yd_v5.task_progress.TaskProgressService`：任务进度与变化起点续接。
- `three_yd_v5.v9_r5_session`：任务范围内的局部存储与地图流转。

调用方负责提供授权的地图、语义、拓扑、查询和停车接入绑定，创建运行输出目录，并配置ROS/Smac会话。Smac适配器要求显式传入`map_yaml`或`ctx.map_yaml`，没有默认现场地图回退。通用参数文件不是地图输入；会话按调用参数设置车辆和约束。

完整ROS接入需要匹配的Nav2变化块发布器，见`integrations/nav2_costmap_2d/README.md`。可选OMPL扩展源码保留，但r9选定方案不调用OMPL，默认构建不编译该扩展。

## 源码导出边界

原封存工作区与生产入口保持原样。本副本仅清理场地默认帧名称与案例注释，并调整依赖的相对路径、去除默认地图回退；算法数学实现和硬审计保持。必要依赖按引用定义提取，省略评测命令和报告生成代码。

这里交付算法源码及无现场数据的验证，不重新声明更多地图、连续控制或实车验收。原完整优化目标已按用户指示停止。

源码清单见`SOURCE_MANIFEST.json`，本次构建、检查及隐私审查见`DELIVERY_CHECKS.md`。SciPy堆实现的许可见`native/THIRD_PARTY.md`与`native/SCIPY_LICENSE.txt`；Nav2原版权及许可证随集成源码保留。
