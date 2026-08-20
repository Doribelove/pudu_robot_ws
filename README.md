# PUDU Robot Workspace

中文启动、停止和功能说明见 [USAGE.md](USAGE.md)。
Arena4 单次录制、指标计算和绘图见
[docs/ARENA4_EVALUATION_USAGE.md](docs/ARENA4_EVALUATION_USAGE.md)。
超大地图 A2B 课题的 P0 冻结口径见
[docs/P0_EVALUATION_DEFINITION.md](docs/P0_EVALUATION_DEFINITION.md)。

Product workspace for PUDU-specific robot description, Nav2 integration,
plugins, coverage planning, cleaning behavior trees and navigation tests.
Related third-party workspaces are grouped under `external/` while remaining
separate colcon overlays.

## Workspace boundaries

- `external/nav2_reference_ws`: upstream Nav2 and TurtleBot3 simulation.
- `external/linorobot_sim_ws`: Linorobot2 Gazebo/SLAM/Nav2 integration.
- `external/exploration_reference_ws`: pinned ROS 2 frontier exploration.
- `external/scurm_sentry_ws`: pinned optional SCURM/FAST-LIO/ICP underlay.
- `external/arena4_ws`: optional Arena simulation/evaluation environment;
  independently built and runtime-isolated from the PUDU overlay.
- `src`: PUDU-owned packages.

`stack_paths.bash` is the single path registry. Legacy `/home/robot/*_ws`
locations remain compatibility symlinks, so existing commands and old colcon
artifacts continue to work. `external/COLCON_IGNORE` keeps the PUDU top-level
build from discovering external packages twice.

## Build

```bash
cd /home/robot/pudu_robot_ws
./build_all.bash
```

For a relocation or a full CMake reconfigure, run
`PUDU_CLEAN_CMAKE_CACHE=true ./build_all.bash`. For PUDU-only incremental
changes, `source setup_underlays.bash && colcon build --symlink-install`
remains available.

## Linorobot2 simulation shortcuts

If Gazebo is not installed system-wide, install the required simulator and ROS
plugins into the existing local dependency root (no sudo required):

```bash
./install_gazebo_local.sh
```

Start Gazebo, Nav2 with the existing `playground` map, and the navigation RViz
view in the background:

```bash
./start_linorobot.sh
```

Start Gazebo, online SLAM, and the mapping RViz view instead:

```bash
./start_linorobot.sh --slam
```

Start Gazebo, SLAM/Nav2/RViz, and autonomous frontier exploration:

```bash
./start_linorobot.sh --explore
```

The simulation uses ROS domain `42` by default to avoid interference from
unrelated ROS nodes. In every additional terminal, load the matching runtime
environment before using `ros2` commands:

```bash
source /home/robot/pudu_robot_ws/setup_linorobot_runtime.bash
```

Stop all processes started by that command:

```bash
./stop_linorobot.sh
```

The default base is `2wd`. Override it or pass Gazebo launch arguments when
needed, for example:

```bash
LINOROBOT2_BASE=4wd ./start_linorobot.sh spawn_x:=1.0 spawn_y:=2.0
```

## Optional SCURM integration

Build or reproduce the pinned upstream workspace:

```bash
./build_scurm_reference.sh
```

Run the SCURM-inspired PUDU navigation profile in the existing simulation:

```bash
./start_linorobot.sh --scurm-nav
```

Run the full 3D SCURM localization path in Gazebo (ICP + FAST-LIO, no AMCL):

```bash
./start_linorobot.sh --scurm-lio
```

Run the same SCURM pipeline with an isolated 1.5x Gazebo time profile:

```bash
./start_linorobot.sh --scurm-lio --fast-sim
```

The normal command remains at the source world's default speed and 360x16
lidar sampling. The fast profile changes only the running Gazebo process to a
2 ms physics step, a 750 Hz update target, and a 120x16 lidar profile; ROS
nodes continue to use simulation time.

Optionally add known-map full-area coverage without changing the normal SCURM
startup path:

```bash
./start_linorobot.sh --scurm-lio --coverage
# Or combine both independent options:
./start_linorobot.sh --scurm-lio --coverage --fast-sim
```

The coverage option now builds a plan from the robot-reachable known-free
component of `/map` automatically. After inspecting the full-map route, run:

```bash
source ./setup_linorobot_runtime.bash
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
```

Using RViz **Publish Point** plus `/coverage/close_area` remains available when
only a manually selected sub-area should be covered.

The optional planner decomposes free space around obstacles, follows continuous
boustrophedon lanes through Nav2, records actual covered cells from TF, and
replans residual missed regions. See
[`src/pudu_coverage_planner/README.md`](src/pudu_coverage_planner/README.md) for
controls, topics, and tuning.

Run FAST-LIO 3D mapping and save the PCD through `/map_save`:

```bash
./start_linorobot.sh --scurm-lio-map \
  map_output_path:=/tmp/scurm_playground_map.pcd
```

This selects Theta*, Constrained Smoother, differential-drive MPPI, and the
PUDU adaptive backup behavior. The normal `./start_linorobot.sh` path is
unchanged. Check or start the optional FAST-LIO/Livox hardware pipeline with:

```bash
./start_scurm.sh --check
./start_scurm.sh --mapping
./start_scurm.sh --localization map_path:=/absolute/path/map.pcd
```

See [SCURM_INTEGRATION.md](SCURM_INTEGRATION.md) for boundaries, verified
features, and hardware prerequisites.

## Optional Arena4 environment

Arena4 remains an independent ROS overlay because it carries 498 source
packages, its own Python environment, Gazebo Harmonic, Fast DDS, and ROS domain
`1`. It is nevertheless managed from this project root:

```bash
./start_arena4.sh --check
./start_arena4.sh                 # standard GUI
./start_arena4.sh --scenario      # reproducible task setup
./start_arena4.sh --headless --scenario
./stop_arena4.sh
```

Use `./build_arena4.bash` for an isolated Arena rebuild and source
`setup_arena4_runtime.bash` in an additional terminal for Arena ROS commands.
The existing PUDU build and Linorobot/SCURM startup paths are unchanged.
