# 运行与接口

完整导航依赖ROS2 Humble、成本图服务、外部地图、交通配置和索引。`base/native`含Linux x86_64原生库及匹配源码，`base/deps`含数值依赖与许可证；其他平台需要重新构建。

在本目录检查文件完整性：

```bash
python3 verify.py
```

在原本机运行环境中选择配置：

```bash
# 当前采用的L2局部扩域版本
V13_PROFILE=p6 source ./env.bash
python3 -m unittest discover -s tests/p6 -p 'test_*.py' -v

# 最新障碍反馈L1候选，建议在另一个终端使用
V13_PROFILE=p7-r1 source ./env.bash
python3 tests/p7/test_runtime_l1.py
```

`env.bash`加载本机P5基础环境，再将本目录代码放到模块搜索路径前部。基础环境脚本可通过`V13_RUNTIME_ENV`指定。原配置记录在`config/original`。

事件接口为`IncrementalService._guarded_update(sequence, upsert=..., clear=..., observed_free=..., current_pose=...)`，调用时配置`TaskProgressService`、session和world。`current_pose`使用障碍出现时的实际位姿，目标沿用当前任务。返回值包含路径有效性、点列来源和失败码。

评测与复现脚本见`tools/original_p6`和`tools/original_p7`，使用原实验目录与外部资源；每次复现指定全新输出目录。

导航计时包含请求内全部尝试、选域、L1/L2/L3、同步ACK和终验。地图与服务准备、进程启动、通用初始化及请求外导出在计时范围之外。L2展开累计所有返回搜索，含恢复与重复；粗图提议和连通预检查单列。RSS为进程树20ms采样之和，CPU为进程树累计差值。
