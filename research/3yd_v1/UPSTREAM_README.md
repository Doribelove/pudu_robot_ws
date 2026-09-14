# 3YD-V1：轻量语义拓扑与多分辨率静态规划

正式版本：**3YD-V1**，对应本轮 `topology_015` 分组，即轻量 L1 语义拓扑、L2 0.15 m 参考规划（必要时回退到 0.05 m）、L3 原 0.05 m Smac 的静态链路。基线为 **3YD-V0-r0**；动态障碍和实机控制不在本轮范围内。

版本与实验分组映射见 [VERSION.json](VERSION.json)，下一批建议见 [NEXT_STAGE_PLAN.md](NEXT_STAGE_PLAN.md)。2026-09-14 的命名确认只更新版本说明与解读文档；原始结果、实验分组标识、冻结实现及其历史报告生成脚本保留原记录。

- 接口及验收约定：[INTERFACES.md](INTERFACES.md)。
- 性能与功能结果：`report/performance.md`，机器可读统计为 `report/performance.json`。
- 冻结实现：`report/IMPLEMENTATION_FREEZE.json`；缓存读取修正后的交付哈希见 `report/FINAL_IMPLEMENTATION_FREEZE.json`；原版来源记录：`BASELINE_SOURCE_MANIFEST.json`。
- 单元与回归检查：`report/regression_cache_fix.xml`。

实现文件位于 `external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/`：

| 文件 | 职责 |
| --- | --- |
| `multires_topology.py` | 语义区域、有向接口、按需拓扑搜索；无成对 Dubins 轨迹预计算 |
| `multires_grid.py` | 按世界原点对齐的粗细图转换、保守聚合、独立方向规则、可指定分辨率的 ROI |
| `multires_pipeline.py` | 粗图引导路、靠右偏好、加权 D*、精细回退、ReferencePlan 与原 L3 适配 |

原始地图、车辆模型、Smac 实现和参数不变。L2 参考路经世界坐标投影到 0.05 m，并进入现有有上限的软代价层；硬障碍不能被软参考覆盖。原 PathAudit、实际代价内容 ACK 和独立连续车体检查仍是接收轨迹的条件。

## 复现

在本目录执行；需要现有 ROS 2 Humble 和 `/home/robot/pudu_robot_ws` 的已安装 Nav2 环境。实验仅调用隔离 ROS 域中的规划服务，不下发车辆速度。

```bash
source env.bash
/usr/bin/python3 tools/static_benchmark.py \
  --output results/my_new_run \
  --arm topology_015 --indices 0:20 --repeats 3 --domain 223
```

`--arm baseline` 为 3YD-V0-r0；`topology_005` 为轻量 L1 与新接口、L2 保持 0.05 m 的中间对照组；`topology_015` 为 **3YD-V1**。输出目录必须不存在，避免覆盖历史运行。

受控场景用 `--scene calibration` 或 `--scene rotated`，每次重复执行右、左、右偏好切换；窄通道场景用 `--scene narrow`。构图性能使用 `tools/static_preparation.py` 的 `build` 和 `restore`，分别在新进程中运行。

完整正式运行命令见 `tools/run_static_formal.bash`，其中输出目录固定，不能覆盖已经完成的实验；复现时另选输出标签。各组固定 CPU 15，顺序运行，不同时竞争测试 CPU。

## 当前边界

L1 提供候选通行关系。它不再提前为所有入口出口验证车辆轨迹，候选路线的几何与车辆可行性必须由 L2/L3 检查。本轮测试覆盖固定真实查询和受控场景，不能证明任意起终点都有可用路线。

0.15 m 无路时，当前实现回退到该拓扑路线区域内的完整 0.05 m L2，原因与耗时均计入结果。后续可改成仅在断连位置使用局部精细连接，以降低回退成本。当前不把搜索失败写为 L1 动态通道封闭，也未实现 L3 失败后自动枚举其他拓扑路线。

缓存键绑定地图、语义、规则、车辆、分辨率、路径与偏好；损坏的参考缓存不会通过切到 0.05 m 被掩盖。动态障碍版本字段目前固定为 `static`，不表示动态同步已经实现。
