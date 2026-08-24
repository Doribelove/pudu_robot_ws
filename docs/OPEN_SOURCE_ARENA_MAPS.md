# Open-source Arena maps

The following maps were downloaded on 2026-08-24 and installed below
`external/arena4_ws/src/arena/simulation-setup/worlds/`. The external Arena
workspace is intentionally ignored by the top-level repository, so these
assets are local runtime data rather than changes to the PUDU source overlay.

## Existing Arena extents

Arena computes the physical extent as `image_width * resolution` by
`image_height * resolution`:

| Existing world | Grid | Resolution | Physical extent |
| --- | ---: | ---: | ---: |
| `hospital` | 800 x 800 | 0.10 m | 80 x 80 m (largest area, 6400 m²) |
| `.generated` | 1700 x 1100 | 0.05 m | 85 x 55 m (largest single span) |
| `factory` | 600 x 600 | 0.10 m | 60 x 60 m |
| `house17` / `map_empty` | 626 x 481 | 0.05 m | 31.30 x 24.05 m |
| `ignc` | 250 x 250 | 0.10 m | 25 x 25 m |

The default launcher world is `map_empty`; it is not the largest map.

## Installed maps

| Arena world | Source | Grid | Resolution | Physical extent | License |
| --- | --- | ---: | ---: | ---: | --- |
| `nav2_100by100_10` | [Nav2 planner benchmarking](https://github.com/ros-navigation/navigation2/tree/humble/tools/planner_benchmarking), `humble` at `3c3db59d6969d8ecee8e68468693d006397f4a0c` | 2000 x 2000 | 0.05 m | 100 x 100 m | Apache-2.0 (Nav2 default) |
| `nav2_100by100_15` | same as above | 2000 x 2000 | 0.05 m | 100 x 100 m | Apache-2.0 (Nav2 default) |
| `nav2_100by100_20` | same as above | 2000 x 2000 | 0.05 m | 100 x 100 m | Apache-2.0 (Nav2 default) |
| `aws_small_warehouse_002` | [AWS Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros2/maps/002), `ros2` at `ee0af733315e78432408c3cd98d378ecee5f767c` | 1536 x 1504 | 0.02 m | 30.72 x 30.08 m | MIT-style upstream `LICENSE` |

The Nav2 maps differ by obstacle density (`10`, `15`, `20`) and are intended
for large-grid planner benchmarking. The AWS map is a real warehouse-world
occupancy capture. The AWS repository is archived; use it as a research asset
and keep its license notice with the copied map. Each installed map directory
contains the corresponding `SOURCE_LICENSE` file.

## Use

Rebuild only the Arena simulation-setup package after adding or replacing map
files:

```bash
cd /home/robot/pudu_robot_ws
./build_arena4.bash --packages-select arena_simulation_setup
```

Then launch a map-only world. With `sim:=dummy`, this exercises the Arena map,
task generation and Nav2 stack. With `sim:=gazebo`, Arena starts its empty SDF
and derives/spawns 2-D walls from the occupancy map:

```bash
./start_arena4.sh --headless \
  world:=nav2_100by100_10 sim:=dummy \
  tm_robots:=explore tm_obstacles:=environment

./start_arena4.sh --headless \
  world:=aws_small_warehouse_002 sim:=gazebo \
  tm_robots:=explore tm_obstacles:=environment
```

For the 100 m maps, use a planner configuration with enough costmap memory and
allow extra startup/planning time at 5 cm resolution (4 million cells).

## Source checksums

The following SHA-256 values identify the downloaded raster files:

```text
nav2_100by100_10/map/map.pgm  ce5456001916caa0b8049425cbadac4877f3889c79d1e48462df08a4c1cac72e
nav2_100by100_15/map/map.pgm  3088b3720fa263e9d596bb96aa04a92f84a2e2f06503580409a0d1a17399dd73
nav2_100by100_20/map/map.pgm  7dcbdd34cac9e007fb36deb3717aaf3988bd260e98f5069d9bd4cc497bec14b6
aws_small_warehouse_002/map/map.pgm  d512370ff571562e11b80fa1f55264836a0cc9c3a8cfd16bbd92d2b8f67a4827
```
