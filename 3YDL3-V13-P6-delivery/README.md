# 3YDL3-V13-P6（交付版）

当前稳定工程交付，配置为 **p6 / H-I**，算法固定于 `d8c01c5a`。本目录提供三层规划代码、原生库、运行资产、检查与复现工具，以及本版验证记录。

L1选择通道，L2通过增量D*生成几何参考，L3进行全路线连续曲线优化。输入地图、实际起终位姿和通行策略，输出`x/y/yaw`全局参考点列、路径有效性及失败原因。

| 内容 | 入口 |
|---|---|
| 三层职责与实现 | [架构说明](ARCHITECTURE.md) |
| 运行代码与配置 | [runtime/3YDL3-V13-L2](runtime/3YDL3-V13-L2)、[原始源码](source/3YDL3-V13-L2) |
| 启动、检查及外部依赖 | [运行说明](RUNNING.md)、[依赖清单](docs/DEPENDENCIES.json) |
| 十图回归与分层补证 | [本版结果](report/RESULTS.md)、[300次请求](report/static300/p6_static_requests.csv)、[400槽位记录](report/ablation400/requests.csv) |
| 源码与运行资产校验 | [SOURCE_LOCK.json](SOURCE_LOCK.json)、[MANIFEST.json](MANIFEST.json)、[资产清单](docs/RUNTIME_ASSETS.json) |
| 交付报告 | [P6工程报告](https://pudutech.feishu.cn/docx/KFVJdLrGWo71ZIxLCYcctwVKntd)、[结题验收说明](https://pudutech.feishu.cn/docx/YFuWdVOStoG9VnxCN9ocwtl6nNb) |

十图原配套100Q三轮回归 **300/300成功（297精确、3容差）**；独立四臂补证中的完整P6 **100/100成功（99精确、1容差）**。两批分别统计，详见结果页。

```bash
python3 verify_delivery.py
source ./env.bash
python3 -m unittest discover -s runtime/3YDL3-V13-L2/tests/p6 -p 'test_*.py' -v
```
