# 3YDL3 · 三层机器人全局规划

[![Public example](https://github.com/Doribelove/pudu_robot_ws/actions/workflows/public-demo.yml/badge.svg)](https://github.com/Doribelove/pudu_robot_ws/actions/workflows/public-demo.yml)
[可运行示例](examples/3ydl3_garage_b/README.md) · [许可范围](LICENSING.md) · [贡献指南](CONTRIBUTING.md) · [版本记录](CHANGELOG.md) · [上游维护](docs/UPSTREAM_MAINTENANCE.md)

3YDL3 研究如何把语义路线选择、栅格参考搜索与满足车体/曲率约束的完整曲线规划组合起来。公开示例采用 **3YDL3-V11-r4** 已验收架构和**合成地库 B**，提供固定 Q01–Q10、独立 L3、真实代价图读回 ACK、连续曲率与车体/交通严格校验。

A reproducible three-layer global-planning research example: semantic route selection,
2-D reference search, and continuously validated cubic B-spline planning.

| 层级 | 实际职责 | 可检查输出 |
|---|---|---|
| L1 | 在语义/自由空间图上选择路线与通道 | `l1_output.json` |
| L2 | 在所选范围内搜索二维参考，并构造软目标 | `l2_reference_xy.npy`、`l2_soft_target.json` |
| L3 | 完整路线三次 B 样条优化、连续证书与严格终验 | `path.npy`、`outcome.json`、`l3_scope.npz` |

![合成地库 B / Q01 实际输出](examples/3ydl3_garage_b/map/q01-preview.png)

## 第一次运行

支持 **Ubuntu 22.04、Linux x86_64、Python 3.10、ROS 2 Humble**。建议至少 4 GiB 可用内存。
先按[示例说明](examples/3ydl3_garage_b/README.md)安装固定 ROS/Nav2 依赖，然后：

```bash
git clone https://github.com/Doribelove/pudu_robot_ws.git
cd pudu_robot_ws
python3 -m venv .demo-venv
.demo-venv/bin/pip install --require-hashes -r examples/3ydl3_garage_b/requirements.lock
.demo-venv/bin/python examples/3ydl3_garage_b/demo.py --venv .demo-venv --output /tmp/garage-b-q01
```

默认 Q01 预期 **1/1 精确成功**，路径约 **422.902 m**，真实代价图 ACK 通过，连续曲率上界不超过 **2.5 m⁻¹**。`--all-queries` 运行完整固定十组；全部预期为 **10/10 精确成功**。结果由程序自动验证；耗时随机器变化，不是固定断言。

仅检查下载完整性，无需 ROS、地图服务或 Python 第三方包：

```bash
python3 examples/3ydl3_garage_b/demo.py --verify-only
```

运行器在私有命名空间内处理历史绝对路径，可从任意检出目录运行。它使用包内源码与合成地图，不读取原研究工作区。依赖、故障排查、完整十组预期值和原生重编译见[示例 README](examples/3ydl3_garage_b/README.md)。

## 适用范围与当前版本

当前公开入口是 **V11-r4 public demo.1**，发布范围为合成地库 B 的静态完整参考规划。最小转弯半径 0.40 m；车体半长/半宽 0.265/0.225 m。示例十组均使用精确端点。

原 r4 十图三轮验收为 300/300 政策内成功，其中 297 次精确、3 次既有容差；这与公开示例的十组复现是两个样本范围。r4 在另一张非语义大地图上仍有明显绕路，不能宣称全局最优或任务书路径质量已全部验收。r5 目前是独立实验候选。

本项目验证静态全局参考规划，不包含连续控制或实车安全认证。Python 端仍保存全图，L3 为完整路线曲线规划；没有移动窗口或线上全图 Smac 搜索回退。地图/服务加载不计入首次导航请求耗时，任务相关同步、恢复、真实 ACK 与严格终验均计入。

## 测试与开发

```bash
OPENBLAS_NUM_THREADS=1 .demo-venv/bin/python -m pytest tests/public_demo -q
```

[CI](https://github.com/Doribelove/pudu_robot_ws/actions/workflows/public-demo.yml)检查输入篡改拒绝、结果校验失败分支，并在隔离 ROS 环境运行完整十组。发布流程见[版本发布说明](docs/RELEASING.md)。

新增原创部分采用 Apache-2.0；第三方组件保留原许可，历史目录不可一概按根许可证复用，详见[许可范围](LICENSING.md)。

历史 r3 源码和单版本报告保留于 [`research/3ydl3_v11_r3`](research/3ydl3_v11_r3/README.md) 与 [`docs/3ydl3_v11_r3`](docs/3ydl3_v11_r3/RESULTS.md)。其他机器人集成的旧入口见 [USAGE.md](USAGE.md)。
