# 3YD-V7-r1：L1双模式查询优化

从V7-r0复制到独立工作区。语义区域、交通规则、候选顺序和下游任务保持兼容；L2、L3源代码未修改。V7-r0及历史冻结基线保留。

正式测量仅限L1：九张固定地图，每张10组查询，在3个独立进程中各调用一次。组合版本有效输出270/270，与同机关闭优化对照及历史V7-r0输出全部一致。五个非正式三层接入样例通过，不能据此声明九图完整导航验收。

- A：无语义粗图搜索找到目标后停止，保持原来的相等代价路径选择，省掉完整距离输出和权重整体转换。
- B：语义备选搜索共享已有路径前缀和静态坐标，只在选中候选时生成几何数组，减少临时对象和复制。

第一轮仅共享前缀的收益不稳定，保留在results/l1_ablation_9maps_10x3和report/iteration1。最终版增加静态坐标复用，正式记录为results/final_l1_ablation_9maps_10x3。两轮没有合并统计。

详见[结果](report/RESULTS.md)、[接口](INTERFACES.md)、[实验协议](EXPERIMENT_PROTOCOL.md)。独立进程峰值RSS基本持平；主要收益是查询耗时下降。

## 运行

```bash
cd /home/robot/workspaces/3yd_v7_r1_20260930
source env.bash
python3 tools/build_goal_search.py

# 无语义L1：固定10组，不启动L2/L3/ROS。
./run.sh --mode free-space --stage l1 --map-id arena_medium_200x200_2d_005 --output results/reproduce_medium

# 语义L1：固定10组，至多3条候选，50,000/10,000预算。
./run.sh --mode semantic --stage l1 --map-id mengjia --output results/reproduce_semantic

# 独立对照及组合版，每张地图3个新进程；输出目录必须不存在。
python3 tools/run_l1_ablation.py --output results/reproduce_ablation
python3 tools/summarize_l1_ablation.py --input results/reproduce_ablation --output report/reproduce

# 完整接入检查独立于正式L1性能实验。
./run.sh --mode free-space --stage full --map-id arena_medium_200x200_2d_005 --query-id A2B-09 --output results/reproduce_medium_integration
```

编译要求g++及现有Python/SciPy运行环境。内核单独构建，不重编Smac。新内核未构建时明确报错，不静默切换算法。第三方许可保留在native目录。

THREEYD_V7_GOAL_STOP和THREEYD_V7_COMPACT_LEGACY默认为1；设为0可分别关闭A/B。候选数量、搜索预算、地图阈值和车辆终验不变。语义输入沿用登记固定集；无语义输入仅支持0.05 m地图，未知格仍禁止进入。

## 检查

```bash
python3 -m pytest -q tests/test_v7_r1_search.py tests/test_v7_map_only.py tests/test_v6_r2_l1.py tests/test_v6_r3_l1.py tests/test_v6_r4_l1.py tests/test_v6_r6_l1.py tests/test_v5_geometry.py tests/test_v5_contract.py
```

54项检查通过；随机图上的288次搜索与原距离及目标路径一致，覆盖相等代价、断连、斜角和目标停止。正式输出比较含所有候选及其顺序，不只比首条路线。
