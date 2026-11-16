# 运行与复现

已验证环境为Ubuntu22.04/x86_64、Python3.10、ROS2 Humble及匹配的Nav2环境。地图、原配套Q、语义/交通输入和ROS/Nav2安装环境由项目负责人单独提供，路径与SHA见[依赖清单](docs/DEPENDENCIES.json)。

## 本机检查

从本目录执行：

```bash
python3 verify_delivery.py
source ./env.bash
python3 tools/check_environment.py --json
python3 -m unittest discover -s runtime/3YDL3-V13-L2/tests/p6 -p 'test_*.py' -v
python3 tools/p6_runtime_binding.py
```

`env.bash`固定选择p6，运行适配器固定H-I。基础环境默认为原P5环境脚本，可通过`V13_RUNTIME_ENV`指定；算法模块优先来自本目录。配置及原生库补齐清单见`docs/RUNTIME_ASSETS.json`。

## 新输出目录复现

完整静态回归使用`tools/run_static_regression.py`，先检查环境，再运行原配套Q试点及正式批次。具体命令和输入取得方式见[原复现说明](docs/REPRODUCTION.md)。每次使用全新输出目录，ROS域和CPU由运行负责人分配。

`tools/isolated_original`保留已验证的隔离runner源，`runtime_supplement`保留95份ROS插件、运行索引和OMPL资产。隔离runner依赖单独交付的完整运行基座；对应两个原Q的成功与ACK证据见`report/isolated`。完整资料包的大小和SHA见[外部资料索引](metadata/EXTERNAL_INPUTS.json)。

导航计时从正式请求开始，包含在线搜索、失败恢复、同步ACK和终验；地图加载、服务启动与请求外导出单列。CPU和RSS采用规划进程树口径，worker CPU单独保存。

## 交付范围

本版按用户认可的P6范围交付。辅助字段采用分块、压缩、共享及释放；Nav2和L3保留部分整图数组，完整细图按需驻留列为C2后续工作。L1在线仍包含端点接入、交通判断和部分二维候选，L02职责范围与导师最终签收待确认。当前验证覆盖静态全局服务及真实Nav2成本图。
