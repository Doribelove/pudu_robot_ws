# Jackal Skid-Steer TEB 开发基线

## 模型边界

Jackal 是四轮滑移转向底盘。对局部规划而言，其控制输入仍是车体线速度 `vx` 和角速度 `wz`，
不会命令横向速度 `vy`；轮胎侧滑通过有限的非完整约束权重表达。Arena4 的 Gazebo `DiffDrive`
插件是车体速度接口抽象，不会精确复现轮胎力学。将来接入真车时应通过轮距倍率、里程计和底盘
控制器标定滑移误差，不应把 TEB 改成全向模型。

## 当前基线

| 目标 | 参数/机制 | 基线值 |
|---|---|---:|
| 保持全局路线 | `weight_viapoint` | 30.0 |
| 允许局部绕障 | `weight_obstacle` / `weight_inflation` | 80.0 / 8.0 |
| 障碍净空 | `min_obstacle_dist` / `inflation_dist` | 0.20 m / 0.60 m |
| 表达滑移但禁止横移命令 | `v_max_y` / `weight_kinematics_nh` | 0 / 1000.0 |
| 偏好前进但允许倒车 | `weight_kinematics_forward_drive` | 5.0 |
| 前进/倒车速度 | `v_max_x` / `v_max_x_backwards` | 1.2 / 0.4 m/s |
| 旋转速度 | `v_max_theta` | 1.8 rad/s |
| 线/角加速度 | `a_max_x` / `a_max_theta` | 2.5 / 2.0 |
| 全局路线前视/碰撞硬检查 | `max_global_plan_lookahead_dist` / `feasibility_check` | 2.5 / 2.5 m |
| 同终点换路重置 | `reinit_path_dist` | 0.35 m |

Ackermann 专属的最小转弯半径、转向角、转向速率和 G3 转向连续性边不会加入 skid-steer 的
优化图。角速度、角加速度和角 jerk 仍限制原地旋转与快速换向，避免命令突变。
2.5 m 前视在最高车速下覆盖约 2.1 s；控制器直接按该距离截取全局路线，不再使用与速度相关的
隐式窗口。碰撞硬检查覆盖完整前视范围，velocity smoother 使用同一组速度与加速度边界。

## 调参顺序

1. 直线无障碍却偏航：检查全局路径和定位，再小幅提高 `weight_viapoint`，不要先提高运动学刚度。
2. 能跟线但绕不开障碍：降低 `weight_viapoint` 或 `weight_inflation`，确认 footprint 和 costmap。
3. 绕障轨迹横向漂移过大：提高 `weight_kinematics_nh`，但保持 `v_max_y: 0.0`。
4. 不愿倒车：先检查首段路径是否确实位于车后，再降低前进偏置；不要直接取消速度安全限制。
5. 倒车过猛：同时降低 TEB 的 `v_max_x_backwards` 与 velocity smoother 的负向最小速度。

每次修改至少回归三类行为：无障碍直线跟随、偏置障碍绕行后回线、先倒出再向前。当前单元测试
已覆盖这三类轨迹逻辑；Hospital 场景用于验证完整 ROS 参数链和实时计算周期。
