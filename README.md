# PUDU Robot Workspace

Product workspace for PUDU-specific robot description, Nav2 integration,
plugins, coverage planning, cleaning behavior trees and navigation tests.
Third-party sources do not live in this workspace.

## Workspace boundaries

- `~/nav2_reference_ws`: upstream Nav2 and official TurtleBot3 simulation.
- `~/linorobot_sim_ws`: Linorobot2 Gazebo/SLAM/Nav2 reference integration.
- `~/exploration_reference_ws`: pinned ROS 2 frontier-exploration underlay.
- `~/pudu_robot_ws`: PUDU-owned packages only.

## Build

```bash
cd ~/pudu_robot_ws
source setup_underlays.bash
colcon build --symlink-install
source install/setup.bash
```

`setup_underlays.bash` uses `~/nav2_reference_ws` as the normal third-party
underlay. Linorobot2 is an independent reference workspace and is not sourced
into the product workspace by default.

The packages are intentionally minimal scaffolds at this stage. Add product
code within the package boundary matching its README.

## Linorobot2 simulation shortcuts

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
