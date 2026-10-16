# 2A-V1 静态规划服务

运行目标为当前实验机。输入是一张固定 0.05 m 地图和任意合法的起终点姿态；服务不发布速度命令。完整 Jackal footprint、DUBIN、48 heading bins、0.40 m 最小转弯半径、禁止倒车及原地旋转保持不变。仅 `final_valid_success=true` 的 `points` 是可使用的参考路径。

## 启动

先加载系统 ROS Humble、固定 Nav2 和本包的安装环境，再执行：

```sh
ros2 run arena_evaluation static_planner_service \
  --config /absolute/path/service.json \
  --output /absolute/path/new_service_run \
  --socket /tmp/pln-service.sock
```

`service.json` 示例：

```json
{
  "map_yaml": "/absolute/path/map.yaml",
  "map_id": "static_map",
  "cache": "/absolute/path/map_cache",
  "ros_domain_id": 186,
  "roi_max_cells": 83886080,
  "ready_context_max_cells": 33554432,
  "ready_ack_timeout_s": 90.0,
  "query_budget_s": 7.0
}
```

输出目录和 socket 必须尚不存在。该服务仅接受本机 Unix socket，权限为 0600；不提供外部网络监听。每个连接提交一行 JSON，接收一行 JSON 后关闭。请求上限 16 KiB，最多 8 个连接，最多 4 个排队请求。

```json
{"command":"status"}
{"command":"plan","request_id":"request-001","start":[1.0,2.0,0.0],"goal":[5.0,6.0,1.57]}
{"command":"cancel","request_id":"request-001"}
```

响应外层 `ok` 表示 API 调用是否完成；规划是否成功由 `result.final_valid_success` 决定。重复的活动请求 ID 或最近 256 个已结束请求 ID 被拒绝。取消是按请求 ID 执行的；连接断开本身不是取消。

## READY、请求和恢复

初始化加载固定地图和可复用分块图，准备审计及查询无关的 costmap 基线，完成 exact ACK 后进入 READY。请求端点的拓扑选择、connector、corridor、costmap 更新、完整内容 ACK、Smac 和 canonical PathAudit 都在请求预算内。排队时间也计入七秒预算。同步阶段可能被外层进程监管中断；过期结果不会暴露为有效路径。

同一地图仅有一个 ROS 工作进程组执行规划。活动请求取消、超时、后端进程退出或上下文不确定时，先按 PID 和进程出生标识确认旧树退出，再创建新一代工作进程并重新验收 READY。无法确认清理时服务进入 FAILED，不启动替代实例。SIGINT/SIGTERM 关闭服务并清理它创建的进程。

密集 ROI 最大 80 Mi-cell，小于 32 Mi-cell 的地图在 READY 中准备固定全图上下文；更大地图使用完整路线 ROI。查询图缓存默认上限 3 GiB（`query_cache_max_bytes` 可调低），分块压缩图缓存上限 128 MiB。进程树 RSS 以采样方式监督，默认上限 10 GiB；这不是内核实时内存硬限制。

服务输出采用只追加目录，ROS 日志也限制在该目录。默认容量为 512 MiB / 4096 文件，每秒检查；超限会安全停止并保留证据，返回 `SERVICE_OUTPUT_CAPACITY`。运行维护需归档输出后使用新的目录启动；服务不会自动删除失败记录或覆写旧日志。该容量监督同样是采样限制。Unix socket 的临时文件使用父进程创建的短路径私有目录，确认工作进程树退出后才清理，避免系统的 socket 路径长度限制。

## 当前验收状态

该入口已通过小图真实 20 查询、发布期间取消和恢复验证。九图同一候选正式验收、空缓存初始化、cold 扩样、持续运行、安装后运行以及日志存储边界仍以本轮最终报告为准。不得凭本文件宣称完整生产验收完成。
