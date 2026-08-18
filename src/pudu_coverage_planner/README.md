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

For full floor coverage after a map is available, this package provides a
separate Boustrophedon coverage stage. It solves a different problem from
frontier exploration and does not replace the unknown-space mapping stage.

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

## Known-map full coverage

Add the coverage node to normal navigation, the SCURM 2D profile, or the full
SCURM 3D localization profile:

```bash
./start_linorobot.sh --navigation --coverage
./start_linorobot.sh --scurm-nav --coverage
./start_linorobot.sh --scurm-lio --coverage
./start_linorobot.sh --scurm-lio --coverage --fast-sim
```

`--coverage` is opt-in and does not alter ordinary navigation. It cannot be
combined with online SLAM, frontier exploration, or `--scurm-lio-map`, because
coverage requires both a stable known map and active Nav2 action servers.
Generic Nav2 uses `general_goal_checker`; the SCURM startup modes automatically
select their tighter `coverage_goal_checker` only for sweep actions.

After `/map` arrives, the default planner automatically extracts the connected
known-free component reachable by the robot, removes cells that cannot fit the
robot footprint, and displays a full-map Boustrophedon plan. Inspect it, then
explicitly start motion:

```bash
source /home/robot/pudu_robot_ws/setup_linorobot_runtime.bash
ros2 service call /coverage/start std_srvs/srv/Trigger '{}'
```

To regenerate the full-map plan explicitly, call:

```bash
ros2 service call /coverage/plan_map std_srvs/srv/Trigger '{}'
```

Manual sub-area coverage remains optional. In RViz, select **Publish Point**,
click the desired polygon vertices in `map`, then call `/coverage/close_area`.
That polygon replaces the automatic full-map plan until `/coverage/plan_map` is
called again.

The planner inflates the polygon boundary and map obstacles by the robot
radius, selects a low-turn sweep direction, performs discrete Boustrophedon
cell decomposition around obstacle holes, and builds densely sampled lane
paths. Nav2 first uses `NavigateToPose` to reach each long sweep entrance and
then uses `FollowPath` continuously along that strip. Tight U-turns are handled
as connector navigation so Nav2's normal progress checker does not mistake an
in-place lane change for a stall. This is still more faithful to a cleaning
path than sending every sampled point as an independent goal.

Execution controls are:

```bash
ros2 service call /coverage/pause std_srvs/srv/Trigger '{}'
ros2 service call /coverage/resume std_srvs/srv/Trigger '{}'
ros2 service call /coverage/cancel std_srvs/srv/Trigger '{}'
ros2 service call /coverage/query std_srvs/srv/Trigger '{}'
ros2 service call /coverage/clear std_srvs/srv/Trigger '{}'
```

While the robot is sweeping, its `map -> base_link` pose paints a coverage disk.
After a pass, uncovered connected regions are planned again for up to two
repair passes. A dynamic obstacle can therefore be avoided by Nav2 and the
temporarily missed floor revisited after it clears. Coverage is complete at
98% by default.

Useful visualization and state topics:

- `/coverage/path` (`nav_msgs/Path`): the current and remaining lanes; the path
  shortens as segments finish.
- `/coverage/uncovered_cells` (`nav_msgs/GridCells`): pending floor cells.
- `/coverage/covered_cells` (`nav_msgs/GridCells`): cells painted by the actual
  robot TF while sweeping.
- `/coverage/traveled_path` (`nav_msgs/Path`): actual connector and sweep route.
- `/coverage/pose_arrows` (`visualization_msgs/MarkerArray`): periodic robot
  orientation arrows without numeric text labels.
- `/coverage/markers` (`visualization_msgs/MarkerArray`): boundary and remaining
  executable sweep segments.
- `/coverage/grid` (`nav_msgs/OccupancyGrid`): 0 pending, 100 covered, -1 outside.
- `/coverage/progress` (`std_msgs/Float32`): fraction from 0.0 to 1.0.
- `/coverage/status` (`std_msgs/String`): execution state and explanation.

The `--coverage` startup option automatically loads `rviz/area_coverage.rviz`.
Its enabled layers use orange for pending cells, green for covered cells,
yellow for the upcoming Boustrophedon route, blue for the actual route, and
magenta for pose breadcrumbs. The local Nav2 obstacle costmap is also enabled
by default. Normal launches keep the original Linorobot RViz configuration.

Pose breadcrumbs default to every 0.50 m. Set `pose_arrow_trigger` to `time` for
the `pose_arrow_period` interval, or `either` to leave an arrow when either the
distance or time threshold is reached. `trail_sample_distance` and
`trail_sample_period` control only the resolution of the blue actual path.

Geometry and recovery parameters live in `config/area_coverage.yaml`. The
default 0.38 m lane spacing is intentionally smaller than twice the 0.38 m
effective scan/cleaning radius. The latter includes the tool or sensor swath and
tracking tolerance; it is not the collision footprint. The separate 0.22 m
robot radius remains responsible for polygon and obstacle safety inflation.
