# 3YD 支线：3YD-V7-r1

2026-09-30 最新 V7 源码交付，远端分支为 **`3YD`**。原工作区为
`/home/robot/workspaces/3yd_v7_r1_20260930`。V7-r1 支持语义及无语义双模式，
在 V7-r0 上优化 L1 查询：无语义粗图搜索在目标确定后停止，语义备选搜索复用路径前缀和静态坐标。

九张固定地图、每图十组查询、三个独立进程的历史正式结果为 **L1 有效输出 270/270**，
完整候选及顺序与 V7-r0 一致。五例完整三层接入回归通过，仅为非正式样例；
尚未完成九图全量三层导航验收，也未替换 V6-r0 冻结生产基线。
结果及资源口径见 [汇总报告](report/RESULTS.md)，模式合同见 [INTERFACES.md](INTERFACES.md)。

## 交付内容

`src/`、`tools/`、`tests/` 和目标停止搜索 C++ 源码按原 V7-r1 文件复制，
包含所继承的 L2/L3 实现。`external/` 包含原运行环境使用的冻结 Python 依赖及
评测扩展 C++ 源码。`native/swept_rect.cpp` 沿用仓库已有 3YD-V1 交付源码。
原版本声明保存在 `RELEASE.json`；交付文件哈希见 `DELIVERY.json`。
新增相对路径环境及构建脚本，原说明和环境分别保存在 `UPSTREAM_README.md` 与 `UPSTREAM_env.bash`。
`COLCON_IGNORE` 将研究副本与主 ROS 工作区构建隔离。

沿用仓库既有数据边界：私有现场地图、语义标注、真实查询坐标、轨迹、现场图片、
完整实验日志和二进制库不随 Git 分发。全部原始记录仍保留在本机原 V7 工作区。
从正式记录逐项提取的 [L1_MEASUREMENTS.csv](report/L1_MEASUREMENTS.csv) 保留
810 次调用的原始耗时、CPU 时间、RSS、有效输出及哈希，不包含坐标和轨迹；
组合版 AB 为其中 270 次。消融组分别统计，不能将 810 次写成组合版样本数。
汇总数值见 [L1_STATISTICS.json](report/L1_STATISTICS.json)。
报告中的原始记录路径是本机证据位置，不代表仓库包含这些数据。

## 不依赖现场数据的检查

Linux、g++、Python 3 开发头文件，以及 NumPy、SciPy、OpenCV、Pillow、PyYAML、psutil、pytest；
部分继承模块导入 ROS 2 Humble Python 接口，需要已有 ROS 环境。

```bash
cd research/3yd_v7
bash build_native.bash
source env.bash
/usr/bin/python3 -m pytest -q -p no:cacheprovider \
  tests/test_v7_r1_search.py tests/test_v7_map_only.py tests/test_v5_geometry.py
```

上述检查使用临时合成地图，覆盖搜索距离/相等代价路径一致性、断连和障碍角处理、
原 PGM 阈值与 0.05 m 分辨率、几何约束；不启动 ROS/Smac 进程。
SciPy 堆代码许可保留在 [native/THIRD_PARTY.md](native/THIRD_PARTY.md)。

## 正式地图实验依赖

历史脚本保留原样；完整复现需要授权本机输入、原配置及固定查询绑定，
不能只克隆仓库就复现九图实验。`tools/l1_only_worker.py` 仍引用原 r13 对照 harness，
语义完整入口仍通过 `fixed10_binding.resolve(map_id)` 验证地图、查询和停车接入绑定。
无语义入口需要原 `inputs/registry.json` 及匹配 SHA-256 的 PGM/YAML/固定查询。
完整三层另需原冻结 Nav2/Smac 原生库及运行配置；这里的构建脚本不重建 Nav2/Smac。
适配路径或进行后续开发应另建版本和输出目录，保留冻结文件及历史输入指纹。

`THREEYD_V7_GOAL_STOP=1` 与 `THREEYD_V7_COMPACT_LEGACY=1` 为默认优化开关。
候选至多三条，首条 A* 预算 50,000、后续候选 10,000；车辆、地图占据和最终验收不变。
