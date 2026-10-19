# 持续维护的三个上游项目

核查日期：2026-10-19。优先延续已经形成审查记录的功能方向；每次先找具体维护需求，复现并验证，再提交完整的修订。

| 项目 | 已合并基础 | 当前重点与下一步 |
|---|---|---|
| [pydata/sparse](https://github.com/pydata/sparse) | [GCXS 切片内存 #957](https://github.com/pydata/sparse/pull/957)、[零维转换 #963](https://github.com/pydata/sparse/pull/963)、[Fortran flatten #964](https://github.com/pydata/sparse/pull/964)，以及 #967/#968/#987 文档 | 优先完成 [稀疏布尔掩码 #962](https://github.com/pydata/sparse/pull/962)。维护 COO/GCXS/DOK 掩码边界、Array API 和非密集化行为；NumPy 标量生成案例已按维护者要求提供完整复现与栈，等待反馈。 |
| [Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python) | [关节类型/限位 IK #669](https://github.com/petercorke/robotics-toolbox-python/pull/669)、[mstraj 速度/加速度 #706](https://github.com/petercorke/robotics-toolbox-python/pull/706) | 延续轨迹输出与运动学接口的正确性维护。检查合并后调用约定、数值边界与文档实例；存在真实缺陷时补回归，避免把主项目的驾驶约束直接强加给通用机械臂库。 |
| [SpatialMath](https://github.com/rai-opensource/spatialmath-python) | [平行/非单位直线距离 #224](https://github.com/rai-opensource/spatialmath-python/pull/224)、[Plücker 缩放不变关系 #232](https://github.com/rai-opensource/spatialmath-python/pull/232)、[Python 3.10 Self 依赖 #233](https://github.com/rai-opensource/spatialmath-python/pull/233) | 延续空间几何的尺度不变性、退化输入与 Python 版本兼容。先追踪已合并功能的实际反馈，再围绕完整几何行为提交修复与参数化测试。 |

既有 PR 监测继续检查新审阅、评论、CI 与合并状态。收到明确要求后在原 PR
完成代码/测试/文档修订并准确回复。仓库自身失败、待批准 CI 与本次代码引入
的失败分开记录；不重复催促刚联系过的维护者。

本次核查：#962 仍开放，最近维护者请求为 2026-10-07 的失败栈要求，已在
2026-10-08 回复；尚无新增具体要求。表中其余链接均已合并，不能把这些历史
合并计为本次新增成果。后续成果以维护者审查、上游合并和可复现证据记录，
不以无关 PR 数量或徽章为目标。
