# PLN-02 架构稳定版 v7

版本标签：`architecture-stable-v7`

## 固定架构

```text
L1  二维骨架拓扑图 + 图 A*
 ↓
L2  拓扑走廊二维 Grid A*
 ↓
L3  局部 Nav2 Smac Hybrid A*（DUBIN）
```

正式默认入口不调用 OMPL RRTstar 或 SST。窗口修复保持串行执行，并在每次局部替换后进行全路径静态和运动学验收。

## 固定约束

- 地图分辨率：`0.05 m/cell`
- `Rmin=0.40 m`
- 最大曲率：`2.50 1/m`
- 只允许前进
- 禁止倒车和原地旋转
- 使用完整 Jackal footprint 进行线段级碰撞检查
- `dynamic_obstacles=false`

## v7 smoke 状态

在 `hospital_005` 的 `q02/q06/q07/q09` 上执行 `1 warmup + 3 measured`：

- 最终有效：`12/12`
- 静态和运动学违规：`0`
- 地图级 Smac session：`1/1/0`（启动/关闭/重启）
- RRTstar/SST 调用：`0/0`
- Online P50/P95：约 `576/714 ms`
- 严格 `P50 <= 500 ms` 延迟门限尚未通过

因此本版本可作为固定三层架构的稳定开发基线和后续多地图实验起点，但不将当前 smoke 宣称为正式多地图性能结论。

## 可追溯范围

本版本纳入 Arena4 评测包源码、配置、测试和构建文件。编译产物、Python 缓存、地图运行结果及实验目录继续按仓库规则排除；完整实验产物保留在本地实验目录，并由其 manifest 记录源码和配置 hash。

