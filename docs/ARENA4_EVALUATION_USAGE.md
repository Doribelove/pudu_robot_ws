# Arena4 Evaluation

Arena4 evaluation has two phases: record one or more navigation episodes, then
stop the simulator and calculate metrics from the CSV files.

## Record

```bash
cd /home/robot/pudu_robot_ws
./start_arena4.sh \
  world:=hospital \
  robot:=jackal \
  local_planner:=teb \
  global_planner:=navfn \
  inter_planner:=navigate_w_replanning_time \
  tm_robots:=scenario \
  record_data_dir:=/home/robot/pudu_robot_ws/experiments/teb_hospital
```

The recorder writes `scan.csv`, `odom.csv`, `cmd_vel.csv`, `nav_status.csv`,
`episode.csv`, `start_goal.csv`, and `params.yaml` below the requested
directory. Check that the files are growing while the experiment runs:

```bash
find /home/robot/pudu_robot_ws/experiments/teb_hospital \
  -maxdepth 1 -type f -printf '%f %s bytes\n' | sort
```

After the episodes are complete, stop Arena4 before calculating metrics so the
CSV files are not being written concurrently:

```bash
./stop_arena4.sh
source ./setup_arena4_runtime.bash
ros2 run arena_evaluation metrics \
  --dir /home/robot/pudu_robot_ws/experiments/teb_hospital
```

This creates `metrics.csv` in the same directory. The main columns are
`result`, `path_length`, `time_diff`, `collision_amount`, `velocity`,
`acceleration`, `jerk`, `curvature`, and `path`.

## Plot

The workspace helper creates a path plot, one velocity plot per episode, and a
compact `summary.csv`:

```bash
cd /home/robot/pudu_robot_ws
source ./setup_arena4_runtime.bash
python tools/plot_arena4_metrics.py \
  --dir /home/robot/pudu_robot_ws/experiments/teb_hospital
```

Outputs are written to `experiments/teb_hospital/plots/`:

```text
paths.png
episode_<N>_velocity.png
summary.csv
```

The Arena package also contains the declaration-based plotting tool, but the
workspace helper above is the recommended single-dataset workflow. For a
batch comparison, place each dataset directory under one parent and use the
declarations in `external/arena4_ws/src/arena/evaluation/arena_evaluation/plot_declarations/`.

## Important

Always use an absolute directory with the current launcher. The recorder launch
must pass `--dir <path>` as two separate arguments; otherwise it falls back to
an automatically generated timestamp directory. If an older run was started
before this fix, locate its data with:

```bash
find /home/robot/pudu_robot_ws/external/arena4_ws/install/arena_evaluation/share/arena_evaluation/data \
  -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -n | tail
```
