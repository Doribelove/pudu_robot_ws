# Optional SCURM integration

## Result

The upstream project is pinned at commit
`46e6425c692ec98f8e65446fb6fdd360f44ef8e5` in
`/home/robot/pudu_robot_ws/external/scurm_sentry_ws`. It is a separate
colcon underlay inside the integrated project tree; the default PUDU and
Linorobot startup paths do not source or launch it.

The local build contains 22 source packages from SCURM and its source
dependencies. `pcl_conversions` is intentionally supplied by ROS Humble rather
than rebuilt a second time. The reproducible manifest and reviewed upstream
compatibility/build patches live under `dependencies/`.

## What is available

| Capability | Integration state | Entry point |
| --- | --- | --- |
| Theta* + constrained smoothing + MPPI | Adapted for the current 2WD model and verified in Gazebo | `./start_linorobot.sh --scurm-nav` |
| Collision-aware backup | PUDU-owned Nav2 behavior; samples forward/reverse differential-drive arcs | Used automatically by the optional behavior tree |
| FAST-LIO2 mapping | Upstream node and MID360 driver, isolated from normal startup | `./start_scurm.sh --mapping` |
| ICP + prior-map FAST-LIO localization | Upstream nodes with configurable PCD path | `./start_scurm.sh --localization map_path:=...` |
| Gazebo 3D ICP + FAST-LIO navigation | 360°×16 PointCloud2 lidar, 200 Hz IMU, no AMCL/EKF localization authority | `./start_linorobot.sh --scurm-lio` |
| Optional 1.5x Gazebo profile | Runtime-only 2 ms/750 Hz physics target and 120x16 lidar; normal mode remains 360x16 | `./start_linorobot.sh --scurm-lio --fast-sim` |
| Optional known-map coverage | Obstacle-aware Boustrophedon cells, continuous Nav2 paths, covered-mask tracking and residual repair | `./start_linorobot.sh --scurm-lio --coverage` |
| Terrain analysis and intensity costmap | Compiled and available for a future 3D sensor profile | Source the SCURM runtime and launch explicitly |
| RoboMaster decision, chassis, and operator UI | Compiled as reference only | Not connected to the cleaning robot |

The SCURM controller parameters were not copied verbatim. Its original profile
assumes a high-speed omnidirectional sentry (`chassis_link`, lateral velocity,
competition-specific topics). The PUDU profile uses `DiffDrive`, conservative
velocity limits, `/scan`, `base_link`, and an arc-based recovery that never
commands lateral velocity.

## Build and checks

```bash
cd /home/robot/pudu_robot_ws
./build_scurm_reference.sh

source setup_underlays.bash
colcon build --symlink-install

./start_scurm.sh --check
```

The SCURM runtime can be loaded manually when inspecting its packages:

```bash
source /home/robot/pudu_robot_ws/setup_scurm_runtime.bash
```

## Run the verified simulation profile

```bash
cd /home/robot/pudu_robot_ws
./start_linorobot.sh --scurm-nav
```

Set a Nav2 goal in RViz as usual. Stop it with `./stop_linorobot.sh`. This mode
has been verified with the bundled playground map: all Nav2 lifecycle nodes
became active and a 1 m navigation goal completed successfully.

This command is the lightweight 2D profile and still uses AMCL. To exercise the
actual SCURM ICP + FAST-LIO localization chain, use:

```bash
./start_linorobot.sh --scurm-lio
```

To run the identical world and localization pipeline at a target 1.5x wall
clock rate, select the optional profile:

```bash
./start_linorobot.sh --scurm-lio --fast-sim
```

Sensor and Nav2 rates stay expressed in simulation time. They therefore run
1.5x faster in wall time without multiplying FAST-LIO `scan_rate` or controller
frequencies a second time. Only the horizontal lidar sampling changes from 360
to 120 in this profile; all 16 vertical channels and the 10 Hz simulation-time
scan rate are preserved.

Full-area coverage can be attached independently, including to the 1.5x
profile:

```bash
./start_linorobot.sh --scurm-lio --coverage
./start_linorobot.sh --scurm-lio --coverage --fast-sim
```

The coverage node consumes the known `/map` and the SCURM-owned `map -> odom`
localization chain; it does not publish localization TF. It submits connector
and sweep actions to Nav2, so the existing MPPI controller and costmaps remain
responsible for live collision avoidance.

That mode starts the simulation-only 3D lidar robot, aligns the scan against
`pudu_nav2_bringup/maps/playground_3d.pcd`, publishes `map -> odom` from the
SCURM ICP result, and publishes `odom -> base_footprint` from FAST-LIO. The
URDF then supplies `base_footprint -> base_link`, and wheel transforms are
published only by `robot_state_publisher` from Gazebo joint states. The Gazebo
EKF and AMCL are intentionally absent, so there is only one owner for each
localization transform.

Useful validation topics are:

```text
/scurm/lidar_points
/imu/data
/icp_result
/odom
/ground_truth/odom
/scurm/localization_error/position
/scurm/localization_error/yaw
/diagnostics
```

The evaluator also writes
`${XDG_RUNTIME_DIR:-/tmp}/scurm-localization-error-$UID.csv`.

## Build or replace the simulation PCD map

FAST-LIO mapping uses the same simulated lidar and IMU:

```bash
./start_linorobot.sh --scurm-lio-map \
  map_output_path:=/tmp/my_scurm_map.pcd

source ./setup_linorobot_runtime.bash
source ./external/scurm_sentry_ws/install/local_setup.bash
source ./install/local_setup.bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

For deterministic regression testing, a Gazebo-ground-truth PCD can be
captured while the robot is moving through the scene:

```bash
ros2 run pudu_scurm_sim ground_truth_map_builder.py --ros-args \
  -p use_sim_time:=true \
  -p duration:=30.0 \
  -p output_path:=/tmp/playground_reference.pcd
```

Pass a replacement map to localization with
`map_path:=/absolute/path/map.pcd`. A map captured from another world must not
be used with the bundled playground world.

## Run the hardware pipeline

Mapping with the bundled MID360 driver:

```bash
./start_scurm.sh --mapping
```

Use an already-running driver or rosbag topics:

```bash
./start_scurm.sh --mapping --no-livox
```

Localize in a prior PCD map:

```bash
./start_scurm.sh --localization map_path:=/absolute/path/GlobalMap.pcd
```

Before using hardware, review the Livox host/sensor addresses in the installed
`MID360_config.json`, calibrate the LiDAR-to-IMU extrinsics, confirm
`/livox/lidar` and `/imu/data`, and replace the example initial pose and map.
Do not start AMCL and the FAST-LIO/ICP `map -> odom` publisher at the same time;
only one localization authority may own that transform.

## Upgrade and licensing boundary

Update `dependencies/scurm_sentry.repos` deliberately and rerun the build patch
check before accepting a new upstream revision. The upstream repository has no
single root license and some packages declare no license, so its maps, robot
description, and competition application remain reference-only pending a
package-level legal review.
