# pudu_coverage_planner

Owns coverage geometry, sweep generation, edge cleaning, obstacle holes,
missed-area recovery and coverage-state interfaces.

## Autonomous mapping

Unknown-space mapping uses frontier exploration from `explore_lite`. Frontier
cells lie between known free space and unknown space. Frontier clusters are
ranked by travel cost and information gain, then sent to Nav2 as
`NavigateToPose` goals. SLAM Toolbox grows `/map` while the robot moves.

This is the mapping stage, where the environment is initially unknown. A
Linorobot-specific goal selector handles frontiers that curve around the robot:
when their centroid falls within `minimum_goal_distance`, it chooses a frontier
point near `preferred_goal_distance` instead. This prevents Nav2 from repeatedly
accepting a goal at the robot's current pose. Both distances are configurable in
`config/linorobot_frontier_exploration.yaml`.

For full floor coverage after a map is available, use a separate known-map
coverage stage (Boustrophedon/room-cell decomposition plus sweep paths). It
solves a different problem from frontier exploration and should not replace the
unknown-space mapping stage.

Start Linorobot2, SLAM, Nav2, RViz and autonomous exploration together:

```bash
cd /home/robot/pudu_robot_ws
./start_linorobot.sh --explore
```

A custom Gazebo world can be supplied as a regular Gazebo launch argument:

```bash
./start_linorobot.sh --explore \
  world:=/absolute/path/to/custom.world
```

Frontiers are published as a `visualization_msgs/MarkerArray` on
`/explore/frontiers`. Add a MarkerArray display in RViz to inspect them.

Pause and resume exploration:

```bash
source /home/robot/pudu_robot_ws/setup_linorobot_runtime.bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

Exploration status is published on `/explore/status`. Stop the complete stack
with `./stop_linorobot.sh`.

The frontier stage builds an unknown map. Product floor-cleaning coverage over
an already built map remains a separate stage and will use room/cell
decomposition with sweep paths.
