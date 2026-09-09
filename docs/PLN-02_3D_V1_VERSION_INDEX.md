# 3D-V1 版本索引

当前唯一默认生产全局规划架构是 **3D-V1-r2-stable**。

| 版本 | 状态 | 默认可选 | 用途 |
|---|---|---:|---|
| 3D-V1-r0 | 历史 | 否 | 初始设计、最小验证与显式 legacy 复现 |
| 3D-V1-r1 | 历史，未晋升 | 否 | L2 状态生命周期研究与显式 legacy 复现 |
| r2 production acceptance | 已冻结晋升依据 | 否 | 显式验收/复现入口 |
| 3D-V1-r2-stable | 稳定生产基线 | 是 | 无 revision import、factory、配置、CLI 和缓存流程 |

稳定标签没有创建 3D-V2 或算法 r3；其实现来源仍是
`r2-production-acceptance`。生产入口拒绝 pure D*，也拒绝显式请求 r0、
r1 或研究版 r2。历史版本仅通过明确标记的 benchmark/reproduction 入口使用。

权威材料：

- 稳定版报告：`docs/PLN-02_3D_V1_STABLE_PRODUCTION_BASELINE.md`
- 生产运行说明：`external/arena4_ws/src/arena/three_d_v1/docs/THREE_D_V1_STABLE_RUNTIME.md`
- 版本策略：`external/arena4_ws/src/arena/three_d_v1/docs/THREE_D_V1_VERSION_POLICY.md`
- 迁移与回滚：`external/arena4_ws/src/arena/three_d_v1/docs/THREE_D_V1_MIGRATION_AND_ROLLBACK.md`
- r2 晋升依据：`docs/PLN-02_3D_V1_R2_PRODUCTION_ACCEPTANCE_FINAL_REPORT.md`

当前证据使用真实 4× 地图上的 realistic synthetic workload。工作区内没有找到
可验证的真实 4× 清扫动态日志，因此不得把这些动态变化描述为真实分布。Stage B
证明稳定入口能够集成 Nav2/Smac，但不代表完整 Nav2/TEB 整车系统已经验收。
