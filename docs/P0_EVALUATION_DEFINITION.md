# P0：A2B 全局规划评测定义（冻结版 v1.0）

- 冻结日期：2026-08-19
- 适用范围：静态栅格地图上的单机器人 A2B 全局规划，不含定位、局部轨迹、速度控制、动态避障和多机调度
- 当前基线：Arena4 + Jackal + Nav2 `SmacPlanner2D`
- 变更原则：本文中的口径、单位和分类优先级在 W1-W2 基线完成前不得修改；确需修改时升级版本，并保留旧结果，禁止覆盖

本文冻结“怎样评测”，不提前冻结尚需由基线数据决定的性能合格线。碰撞路径数和运动学不可行段数的合格线已经由课题要求确定为 0。

## 1. 地图定义

### 1.1 文件格式

正式地图使用 ROS Map Server 格式：一份 YAML 元数据和一份 8 bit PGM（P5）灰度图。YAML 必须显式包含：

```yaml
image: map.pgm
mode: trinary
resolution: 0.1
origin: [origin_x, origin_y, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

Arena 旧地图若省略 `mode`，Nav2 会默认使用 `trinary`；进入正式地图集时仍须补全该字段。禁止在同一档实验中混用 PNG alpha、`scale` 或 `raw` 模式。地图清单必须记录 YAML 和图像的 SHA-256，文件变化即视为新地图版本。

主评测统一使用 `0.10 m/cell`。非 0.10 m 原图必须用最近邻规则离线重采样到 0.10 m，并同时保留原图；禁止双线性插值制造新的灰度占据概率。算法内部可以创建多分辨率层，但输入和最终碰撞判定一律回到 0.10 m 原生评测层。

### 1.2 占据与未知区

令灰度值为 `g in [0,255]`，`negate=0` 时占据概率 `p=(255-g)/255`。严格按当前 Nav2 Map Server 语义分类：

- `p > 0.65`（即 `g <= 89`）：占据，OccupancyGrid 值 100；
- `p < 0.196`（即 `g >= 206`）：自由，OccupancyGrid 值 0；
- 其余 `90 <= g <= 205`：未知，OccupancyGrid 值 -1。

正式 A2B 评测中，未知区与地图外区域均按不可通行处理，规划器必须设置 `allow_unknown=false`。障碍阈值不得由被测算法自行改变。

### 1.3 坐标系

- 所有输入、输出和指标统一使用 `map` 坐标系，单位为米和弧度；右手系，`+x` 向东、`+y` 向北、`+z` 向上，yaw 逆时针为正。
- YAML 的 `origin` 是图像左下角对应的地图位姿。PGM 第 0 行位于图像顶部；像素 `(row, col)` 的栅格中心为：`x=origin_x+(col+0.5)r`，`y=origin_y+(H-1-row+0.5)r`。
- Arena Jackal 运动回归使用 `map -> jackal/odom -> jackal/base_link`：`map -> jackal/odom` 为恒等静态变换，`jackal/odom -> jackal/base_link` 由 Gazebo 世界真值里程计唯一发布；AMCL 仅输出诊断位姿且不广播 TF。这样可避免滑移转向轮速积分和仿真/栅格几何差异破坏 RViz 与点云对齐。全局规划离线评测不依赖 `odom`、AMCL 或 TF。

### 1.4 地图规模分档

规模以有效栅格总数 `width * height` 为主口径，面积只作直观说明。每个正式档位的地图栅格数允许相对标称值上下浮动 10%，结果按实际栅格数绘图。

| 档位 | 标称栅格数 | 0.10 m 分辨率下标称面积 | 用途 |
| --- | ---: | ---: | --- |
| Smoke | `< 1,000,000` | `< 10,000 m²` | 功能回归，不进入三档规模曲线 |
| S | `1,000,000` | `10,000 m²` | 小档 |
| M | `5,000,000` | `50,000 m²` | 中档 |
| L | `20,000,000` | `200,000 m²` | 大档 |

每档至少 3 张不同拓扑结构的地图；每张地图至少 100 个可达起终点对。地图障碍密度、自由区比例和最大连通域比例必须随清单发布，不能只用缩放后的同一张图充数。

当前 `hospital` 地图属于 Smoke：800 x 800、0.10 m/cell、origin `[-40,-40,0]`。当前文件校验为：

- `map.yaml`: `fbf26149de28fd242ee862f4d9a540dce979ead8ccc44deb34d111da79bf430d`
- `map.pgm`: `c5eb8d4d802a4a954166bb2068c2623455be3312ddd9ccb6af9ca8335bd2c147`

## 2. 机器人与膨胀

### 2.1 目标车型

目标车型冻结为 Jackal 四轮滑移转向底盘，在规划接口中按差速模型处理，不是阿克曼模型。依据是 Arena Jackal 使用 `DiffDrive`/`DiffDriveController`，当前 MPPI 也使用 `motion_model: DiffDrive`。

- 允许前进、倒车和原地旋转；最小转弯半径为 0；
- 原地转向本身不是运动学违规，但必须输出为位置不变、yaw 连续变化的显式旋转段；
- 平移段的车体朝向须与运动方向一致；倒车段允许相差 pi；
- 任何阿克曼/Dubins 最小转弯半径结论不得混入本基线。若后续产品车型改为阿克曼，必须新建独立评测配置和版本。

### 2.2 轮廓

规划和碰撞评测使用当前 Arena Nav2 的较大矩形轮廓，而不是 URDF 中较小的 0.420 x 0.310 m 底盘盒：

```text
原始 footprint（base_link，逆时针/顺时针均可规范化）：
[(+0.255,+0.215), (+0.255,-0.215), (-0.255,-0.215), (-0.255,+0.215)] m

footprint_padding: 0.010 m
硬碰撞判定等效外包矩形：x in [-0.265,+0.265], y in [-0.225,+0.225] m
内切半径：0.225 m；外接半径：约 0.348 m（均含 padding）
```

不得用单一 `robot_radius` 替代正式碰撞检查。圆半径只允许用于起终点集的保守连通性预筛选。

### 2.3 膨胀方式

全局规划和 Arena 局部控制统一使用 Nav2 `InflationLayer`：

- `inflation_radius = 0.55 m`；
- `cost_scaling_factor = 3.0`；
- 障碍中心为 lethal cost 254；距离不大于内切半径时为 inscribed cost 253；其余膨胀代价为 `252 * exp(-3.0 * (d - r_inscribed))`，在 0.55 m 外为 0；
- 膨胀是搜索偏好和安全余量，不代替路径的精确 footprint 碰撞检查。

此前 Arena 全局/局部半径分别为 0.25/0.55 m，会让全局路径靠近局部控制器不愿进入的区域；v1.0 基线已经统一为 0.55 m。横向“靠边/靠中”代价必须作为独立项记录，不能通过偷偷修改膨胀半径实现。

## 3. 起终点集

每条 case 必须固定并记录：`case_id`、地图 SHA-256、`start[x,y,yaw]`、`goal[x,y,yaw]`、预期可达性、区域标签、横向偏好和随机种子。yaw 采样量化为 `pi/8`（16 个方向），但文件中存储完整浮点值。

有效起点和终点同时满足：

1. 位姿在地图边界内，所在栅格为已知自由区；
2. 含 0.01 m padding 的完整矩形 footprint 与占据、未知、地图外栅格均无相交；
3. 起终点欧氏距离至少 10.0 m，排除无意义的近距离样本；
4. 不得在生成后由某个被测规划器“吸附”到自由点；无效输入在运行前由数据集校验器拒绝，不计入算法无解率。

可达正例采用保守外接圆（0.348 m）膨胀后的自由栅格做 8 邻域连通性检查，同一连通域才入选。确定无解负例采用内切圆（0.225 m）膨胀后仍不连通的点对。两者之间的模糊样本不进入成功率主表，可单列压力测试。

正式成功率只以“可达正例”为分母；无解负例用于验证失败原因码，单独报告正确识别率和误报率。每个 case 文件一旦冻结，所有算法必须使用完全相同的点对和顺序。

## 4. 超时与结果分类

### 4.1 时间预算

- 在线搜索预算：2.000 s，使用单调墙钟；与当前 Smac2D `max_planning_time=2.0` 一致；
- 评测器硬截止：2.500 s。超过后取消请求并终止该 case 的规划进程；
- 地图加载、拓扑构建、多分辨率金字塔构建不计入在线搜索时间，但必须分别统计预计算墙钟、CPU、峰值内存和持久化字节数；
- 降级和回退属于一次端到端请求，所有层共享同一个 2.000 s 在线预算，不得每层重新获得 2 s。

### 4.2 分类与优先级

每次运行只能落入一个主结果，按以下优先级判定：

| 结果码 | 定义 |
| --- | --- |
| `COLLISION` | 返回了路径，但按第 5.1 节检查与占据/未知/地图外发生 footprint 重叠；优先于成功 |
| `KINEMATIC_INFEASIBLE` | 路径无碰撞，但存在非有限数、航向跳变或不符合第 2.1 节差速约束的段 |
| `SUCCESS` | 2.000 s 内返回非空、有限、无碰撞、运动学可行路径，首末位置误差均不大于 0.125 m，约束 yaw 的终点误差不大于 5 度 |
| `NO_PATH` | 在 2.000 s 内正常结束并明确返回无路径/搜索空间耗尽，且没有路径输出 |
| `TIMEOUT` | 算法命中 2.000 s 预算，或评测器在 2.500 s 仍未收到终态 |
| `EXCEPTION` | 进程崩溃、未捕获异常、协议错误、空成功响应、NaN/Inf、非法原因码或其他不能归入以上类别的情况 |

`INVALID_INPUT` 是数据集校验错误，不是算法结果；发现后整批实验作废并修复清单。失败必须同时带稳定的细分原因码和最后执行层，例如 `L1_NO_ROUTE`、`L2_CORRIDOR_EXHAUSTED`、`L3_KINEMATIC_SEARCH_TIMEOUT`，禁止只返回空路径。

## 5. 指标统计口径

### 5.1 碰撞与运动学检查

对输出路径按平移步长不大于 0.025 m、yaw 步长不大于 1 度进行 SE(2) 插值。每个插值位姿都将含 padding 的矩形 footprint 变换到 `map`，用多边形与栅格方形的相交测试检查占据、未知和地图外；不能只检查中心点。

差速可行性要求位置与 yaw 为有限数且连续。平移段朝向与切线误差不大于 5 度（倒车时与反向切线比较）；原地旋转段允许零位移。相邻输出点若同时发生大于上述插值上限的平移和未展开 yaw 跳变，先按最短角插值再检查。碰撞路径数和不可行段数均按 case 计数，同时保留违规段总数。

### 5.2 路径长度与质量

- 路径长度：`L = sum(hypot(x[i+1]-x[i], y[i+1]-y[i]))`，单位 m；原地旋转不增加长度，倒车仍取正长度；
- 绕行比：每个 case 的 `L_algorithm / L_reference`，先逐 case 求比再统计，不能用两组总长度相除；
- 航向变化率：对非零长度段计算 `abs(unwrapped_delta_yaw) / segment_length`，报告 P95；原地旋转单独统计次数与累计角度；
- 大转角：相邻非原地平移段的航向变化大于 30 度记 1 次。

### 5.3 时间、CPU 和内存

- 官方规划耗时：评测器发送请求前一刻到收到完整终态和路径后一刻的 `CLOCK_MONOTONIC_RAW` 差值，单位 ms；同时可记录插件内部时间，但不得替代端到端值；
- CPU 时间：为每个 case 建独立 cgroup v2，使用请求前后 `cpu.stat:usage_usec` 差值，包含规划器创建的线程/子进程，不包含评测器、Gazebo、RViz、AMCL；
- 平均核占用：`100 * CPU_time / wall_time`，因此多线程算法可超过 100%；
- 峰值内存：每个 case 使用新 cgroup，取 `memory.peak`，单位 MiB（`2^20` bytes）。进程完成后销毁 cgroup以复位峰值；报告地图就绪时内存、规划绝对峰值及二者差值；
- 拓扑图、金字塔和缓存的磁盘体积按实际文件字节数统计，不能算入 RAM，也不能不报。

成功耗时报告 P50/P95/P99；另报告超时率，并给出将超时样本按 2500 ms 右删失上限计入的保守 P50/P95/P99。分位数使用 NumPy `quantile(method="linear")`。峰值内存、CPU 时间和路径长度同样报告 P50/P95/P99，均按地图、规模档和算法拆分。

### 5.4 重复与随机性

- 确定性算法：每个 case 预热 1 次（丢弃），正式重复 5 次；
- RRT*/AO-RRT*：每个 case 使用固定种子 0..29 各运行 1 次，报告分布，禁止只挑最好一次；
- case 执行顺序用种子 `20260819` 固定洗牌；
- 成功率按 case 计数，不把同一 case 的重复运行伪装成更多样本；随机算法另报“至少 1/30 成功”和“单次种子成功率”。

## 6. 软件、编译和机器

### 6.1 冻结软件栈

| 项目 | 冻结值 |
| --- | --- |
| 工作区 commit | `d521514fdc08a7409015f2ad70792936640983a2` |
| Arena 基线补丁 | `dependencies/patches/arena4-jackal-baseline.patch`，SHA-256 `da6dc5a1291e3130583cd764fd1b9604985a3cec7257e584b0fc3dda2e9e68ad` |
| ROS | Humble，`ros-humble-ros-base 0.10.0-1jammy.20260804.204550` |
| Nav2 | 1.1.19 |
| Arena simulation setup | 4.0.0 |
| Gazebo Sim | Harmonic / gz-sim 8.15.0 |
| RMW | `rmw_fastrtps_cpp` |
| GCC / CMake / Python | 11.4.0 / 3.22.1 / 3.10.12 |

当前工作树包含未提交的 Arena 修复，因此 commit 之外必须同时记录补丁 SHA-256 和 `git status --short`。正式结果目录还必须保存：完整命令、参数展开后的 YAML、地图/点集哈希、算法 commit、随机种子和原始逐 case CSV/JSONL。

正式性能构建统一为 CMake `Release`（等价核心标志 `-O3 -DNDEBUG`）、`BUILD_TESTING=OFF`、无 sanitizer、无 LTO，使用 colcon `--symlink-install`。当前工作区部分包的历史 CMake cache 没有统一 `CMAKE_BUILD_TYPE`，只能用于功能回归；采集 W1-W2 性能基线前必须清理相关 build/install 后以 Release 重编，并把实际编译命令存档。

### 6.2 冻结机器

| 项目 | 当前评测机 |
| --- | --- |
| OS / kernel | Ubuntu 22.04.5 LTS / 6.8.0-136-generic x86_64 |
| CPU | Intel Core i7-14700，8 P-core + 12 E-core，20 核 28 线程，最高 5.4 GHz，L3 33 MiB |
| 内存 | 33,251,500,032 bytes（约 31.0 GiB） |
| 磁盘 | Thinkplus ST9000 1 TB NVMe |

官方纯规划实验只允许规划器使用逻辑 CPU `0-15`（8 个 P-core 的线程），评测器固定到 CPU `16`；CPU governor 设为 `performance`，禁用 swap，关闭 Gazebo/RViz/浏览器和非必要 ROS 节点。每批实验前后记录温度、governor、可用内存和后台进程快照。未满足这些条件的运行必须标为 `exploratory`，不能进入正式曲线。

## 7. 执行协议与验收输出

官方全局规划性能评测采用无 Gazebo的离线进程：加载冻结静态地图和 planner，等待 ready，执行单个 case，采集 cgroup 指标后退出。Arena headless 仿真只用于接口、TF、定位和可运动性回归，不进入全局规划 CPU/内存主表。

每次结果至少输出以下字段：

```text
run_id, definition_version, map_id, map_sha256, tier, case_id,
algorithm, algorithm_commit, seed, repeat, start, goal, preference,
result_code, reason_code, last_layer, wall_ms, cpu_ms, avg_cpu_percent,
ready_memory_mib, peak_memory_mib, incremental_memory_mib,
path_length_m, collision_segments, infeasible_segments,
heading_rate_p95, large_turn_count, fallback_trace
```

聚合报告必须包含：

1. 各地图/档位的成功、无解、超时、碰撞、不可行、异常数量；
2. 耗时、CPU、峰值内存随实际栅格数的曲线和 P50/P95/P99；
3. 路径长度比、平滑性和横向偏好满足度；
4. 拓扑预计算开销与持久化大小；
5. L1/L2/L3 开关消融和每条降级分支的触发次数；
6. 完整版本、机器、命令、哈希和原始数据位置。

P0 的完成标准是本文所有口径均有唯一、可实现的判定方式。性能合格线在 W1-W2 基线数据产生后由导师确认，确认结果作为 v1.0 的独立验收阈值附件，不反向修改本文件中的测量口径。
