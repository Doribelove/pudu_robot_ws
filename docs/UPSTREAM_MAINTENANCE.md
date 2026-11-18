# 持续维护的三个上游项目

最近核查于 2026-11-18。

| 项目 | 已合并基础 | 当前重点与下一步 |
|---|---|---|
| [pydata/sparse](https://github.com/pydata/sparse) | [GCXS 切片内存 #957](https://github.com/pydata/sparse/pull/957)、[COO 和 GCXS 稀疏布尔掩码 #962](https://github.com/pydata/sparse/pull/962)、[零维转换 #963](https://github.com/pydata/sparse/pull/963)、[Fortran flatten #964](https://github.com/pydata/sparse/pull/964) | [#1029](https://github.com/pydata/sparse/pull/1029) 修复大尺寸 COO 数组中有效的无符号坐标被误判为越界的问题。[#1034](https://github.com/pydata/sparse/pull/1034) 让 `broadcast_to` 在目标维数小于输入时抛出 `ValueError`。两项均待上游合并。 |
| [Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python) | [数值逆运动学的关节类型和限位 #669](https://github.com/petercorke/robotics-toolbox-python/pull/669)、[mstraj 速度和加速度 #706](https://github.com/petercorke/robotics-toolbox-python/pull/706) | 继续跟进逆运动学和轨迹接口的使用反馈。轨迹回归重点检查段间切换时的位置、速度和加速度。 |
| [SpatialMath](https://github.com/rai-opensource/spatialmath-python) | [平行或非单位直线距离 #224](https://github.com/rai-opensource/spatialmath-python/pull/224)、[Plücker 直线缩放 #232](https://github.com/rai-opensource/spatialmath-python/pull/232)、[Python 3.10 Self 依赖 #233](https://github.com/rai-opensource/spatialmath-python/pull/233) | [线段求交 #282](https://github.com/rai-opensource/spatialmath-python/pull/282) 仍开放。跟进同一端点采用不同尺度的齐次坐标时，线段与直线求交结果是否一致。 |

Sparse 的配套文档包含 [COO 坐标类型的容量限制](https://github.com/pydata/sparse/pull/967)、
[GCXS 入门](https://github.com/pydata/sparse/pull/968) 和
[`where` 的填充值传播](https://github.com/pydata/sparse/pull/987)。

跟进审阅时，在原 PR 记录触发问题的输入、预期结果和验证结果。遇到 CI 失败，
对照基线和当前提交的日志，再决定是否需要修订。最新进展见
[GitHub 上仍开放的 PR](https://github.com/pulls?q=is%3Apr+is%3Aopen+author%3ADoribelove)。
