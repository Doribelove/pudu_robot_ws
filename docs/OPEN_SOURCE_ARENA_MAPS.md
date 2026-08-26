# Open-source Arena maps

The following maps were downloaded or generated on 2026-08-24 and installed below
`external/arena4_ws/src/arena/simulation-setup/worlds/`. The external Arena
workspace is intentionally ignored by the top-level repository, so these
assets are local runtime data rather than changes to the PUDU source overlay.

## Four size bands

For converted maps, `A` is the output canvas area:
`width_pixels * height_pixels * resolution^2`. It includes the small 0.25 m
conversion margin and is not the percentage of free cells. Existing Arena
maps retain their upstream resolution; every map in the generated table below
is standardized to `0.05 m/cell`.

| Area band | Existing/direct Arena maps | Source-derived generated 2-D maps (`0.05 m/cell`) |
| --- | --- | --- |
| `0 <= A <= 10000 m²` | `aws_small_warehouse_002` 924.06; `hospital` 6,400; `factory` 3,600; `.generated` 4,675; `nav2_100by100_10/15/20` 10,000 (planner stress maps) | `hawker_centre_large_2d_005` 44.22; `hospital_walls_small_2d_005` 585.10; `hospital_walls_2d_005` 908.30; `hawker_centre_2d_005` 2,332.85 |
| `10000 < A <= 50000 m²` | No additional verified direct semantic map | `campus1_2d_005` 13,255.73; `airport_terminal_l1_2d_005` 36,481.20 |
| `50000 < A < 200000 m²` | No verified direct semantic map | `campus2_2d_005` 90,866.30; `campus3_2d_005` 110,529.78 |
| `A > 200000 m²` | No verified direct ROS occupancy map | No source-derived candidate found in the checked open-source Arena assets |

The generated area is the raster canvas, while the source model envelope is
only a geometry-size estimate. It must not be confused with traversable floor
area. The airport model in particular keeps its upstream floor/wall placement,
so its projection should be visually validated before benchmark use.

`campus3_2d_005` has an additional connectivity pass. It inserts 90 doors,
each 40 cells wide (`2.0 m` at `0.05 m/cell`), between room-sized free-space
components. Unknown exterior pixels are preserved, and the original raster is
kept as `map.before_doors.pgm`; door coordinates and validation statistics are
recorded in `map_doors.json`.

The eight `arena_*_2d_005` maps listed below are procedural layouts generated
for scale and planner testing. They are not upstream open-source floor plans;
each directory contains `SOURCE_LICENSE` and `layout_design.json` describing
the generated geometry. All use a 5 cm grid, 2 m door openings, a central open
plaza, intersecting east-west/north-south circulation, wide halls, office or
teaching-room partitions, warehouse shelf aisles, and passages at least 2 m
wide (the wide circulation gaps are at least 4 m). A connectivity repair pass
was applied to every map with `--min-component-area 1`, so the complete free
space is one 8-connected component.

The layouts can be reproduced with
`python3 tools/generate_arena_scale_maps.py --output-root
external/arena4_ws/src/arena/simulation-setup/worlds`, followed by
`tools/open_map_doors.py` for each map directory if the layout is changed.

The 3-D assets already exist in the local Arena checkout at
`external/arena4_ws/src/arena/simulation-setup/gazebo_models/`. They are from
[voshch/arena-simulation-setup](https://github.com/voshch/arena-simulation-setup),
ROS 2 commit `3f142b25d88ce962c803b57cf20f38985d376dea`, whose package metadata
declares BSD. The model metadata identifies the original authors (for example,
Nam Truong Tran for Campus and hawker/hospital assets). Keep the upstream
model metadata and attribution when redistributing the meshes.

Useful source links:

- [Arena simulation setup models](https://github.com/voshch/arena-simulation-setup/tree/ros2/gazebo_models)
- [Campus2 model](https://github.com/voshch/arena-simulation-setup/tree/ros2/gazebo_models/Campus2)
- [Campus3 model](https://github.com/voshch/arena-simulation-setup/tree/ros2/gazebo_models/Campus3)
- [Hospital walls model](https://github.com/voshch/arena-simulation-setup/tree/ros2/gazebo_models/hospital_walls_large)
- [Hawker centre walls model](https://github.com/voshch/arena-simulation-setup/tree/ros2/gazebo_models/arena_hawker_centre_walls)
- [Airport terminal level 1](https://github.com/voshch/arena-simulation-setup/tree/ros2/gazebo_models/airport_terminal_L1)
- [Gazebo model database](https://github.com/osrf/gazebo_models) (CC BY 3.0; useful small building models)

### Structure coverage

`hospital` is still the best upstream ready-to-run Arena map for the requested
semantics: long and short corridors, rooms, door openings, elevator lobby and
open public space. `aws_small_warehouse_002` provides shelves and wide/narrow
aisles. The generated hospital maps preserve door gaps; the hawker map has a
central hall and radial corridors; the Campus maps contain large open grounds,
building interiors and mixed-width passages. The airport projection contains
the terminal floor and wall geometry but should be checked against the source
model because its OBJ parts occupy separate Y ranges.

### Converting a 3-D asset to an Arena map

The reproducible converter is
`tools/mesh_to_occupancy.py`. It applies Collada scene matrices and units,
projects geometry in the robot-height band, uses `0=occupied`, `254=free`,
`205=unknown`, and writes `map.pgm`, `map.yaml`, `map_stats.json` and
`SOURCE_LICENSE`. Re-run it only at `--resolution 0.05`; after conversion,
rebuild `arena_simulation_setup` and run a `sim:=dummy` map-server loading
smoke test before starting Gazebo. Dummy mode does not provide robot Nav2/TF.

For a mesh whose floor plan has sealed rooms, run
`tools/open_map_doors.py --map-dir <world>/map --in-place`. The default pass
connects components of at least 200 cells using 2 m doors and never converts
unknown space to free space.

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

## Generated maps

| Arena world | Source asset | Grid | Resolution | Physical extent | Canvas area |
| --- | --- | ---: | ---: | ---: | ---: |
| `hawker_centre_large_2d_005` | `arena_hawker_centre_walls_large.dae` | 133 x 133 | 0.05 m | 6.65 x 6.65 m | 44.22 m² |
| `hospital_walls_small_2d_005` | `hospital_walls_small.dae` | 458 x 511 | 0.05 m | 22.90 x 25.55 m | 585.10 m² |
| `hospital_walls_2d_005` | `hospital_walls.dae` | 711 x 511 | 0.05 m | 35.55 x 25.55 m | 908.30 m² |
| `hawker_centre_2d_005` | `arena_hawker_centre_walls.dae` | 970 x 962 | 0.05 m | 48.50 x 48.10 m | 2,332.85 m² |
| `campus1_2d_005` | `Campus1/meshes/Campus_world.obj` | 2801 x 1893 | 0.05 m | 140.05 x 94.65 m | 13,255.73 m² |
| `airport_terminal_l1_2d_005` | `airport_terminal_L1/meshes/{floor,wall}_*.obj` | 5656 x 2580 | 0.05 m | 282.80 x 129.00 m | 36,481.20 m² |
| `campus2_2d_005` | `Campus2/meshes/Campus_world.obj` | 5107 x 7117 | 0.05 m | 255.35 x 355.85 m | 90,866.30 m² |
| `campus3_2d_005` | `Campus3/meshes/Campus_world.obj` | 13249 x 3337 | 0.05 m | 662.45 x 166.85 m | 110,529.78 m² |

### Synthetic scale maps

These maps satisfy the requested two-map-per-band design. `A` is the full
canvas area, including walls and circulation space.

| Arena world | Area band | Grid | Physical extent | A |
| --- | --- | ---: | ---: | ---: |
| `arena_small_080x100_2d_005` | `0 <= A <= 10000 m²` | 1600 x 2000 | 80 x 100 m | 8,000 m² |
| `arena_small_100x100_2d_005` | `0 <= A <= 10000 m²` | 2000 x 2000 | 100 x 100 m | 10,000 m² |
| `arena_medium_160x200_2d_005` | `10000 < A <= 50000 m²` | 3200 x 4000 | 160 x 200 m | 32,000 m² |
| `arena_medium_200x200_2d_005` | `10000 < A <= 50000 m²` | 4000 x 4000 | 200 x 200 m | 40,000 m² |
| `arena_large_250x300_2d_005` | `50000 < A < 200000 m²` | 5000 x 6000 | 250 x 300 m | 75,000 m² |
| `arena_large_300x500_2d_005` | `50000 < A < 200000 m²` | 6000 x 10000 | 300 x 500 m | 150,000 m² |
| `arena_xlarge_500x500_2d_005` | `A > 200000 m²` | 10000 x 10000 | 500 x 500 m | 250,000 m² |
| `arena_xlarge_600x400_2d_005` | `A > 200000 m²` | 12000 x 8000 | 600 x 400 m | 240,000 m² |

An older local world named `arena_ultra_500x400_2d_005` may also be visible in
`World.list()`. Its canvas is exactly 200,000 m², so it is outside the strict
`A > 200000 m²` band and is not counted among these eight maps; it has also
been repaired to one connected free-space component for compatibility.

The Nav2 maps differ by obstacle density (`10`, `15`, `20`) and are intended
for large-grid planner benchmarking. The AWS map is a real warehouse-world
occupancy capture at its upstream 0.02 m resolution. The AWS repository is
archived; use it as a research asset and keep its license notice with the
copied map. Each generated map directory contains the corresponding
`SOURCE_LICENSE` file.

## Use

Rebuild only the Arena simulation-setup package after adding or replacing map
files:

```bash
cd /home/robot/pudu_robot_ws
./build_arena4.bash --packages-select arena_simulation_setup
```

Then launch a map-only world. With `sim:=dummy`, Arena starts map_server and
confirms that the PGM was loaded; it intentionally does not start Gazebo,
robot odometry, AMCL, or a complete robot Nav2/TF stack. With `sim:=gazebo`,
Arena starts its empty SDF and derives/spawns 2-D walls from the occupancy map
for full robot navigation:

```bash
./start_arena4.sh --headless \
  world:=nav2_100by100_10 sim:=dummy

./start_arena4.sh --headless \
  world:=aws_small_warehouse_002 sim:=gazebo \
  tm_robots:=explore tm_obstacles:=environment

./start_arena4.sh --headless \
  world:=arena_xlarge_500x500_2d_005 sim:=dummy
```

For the 100 m maps, use a planner configuration with enough costmap memory and
allow extra startup/planning time at 5 cm resolution (4 million cells). The
250,000 m² map has 100 million cells and needs substantially more memory and
startup time than a normal Arena world. For full Gazebo navigation on the
largest maps, set `ARENA4_STARTUP_TIMEOUT_S` to a larger value and expect high
RAM usage. Arena4 detects maps larger than 12 million cells and initializes the
task generator directly from the local PGM instead of synchronously waiting for
the reliable 100 MB ROS `OccupancyGrid`; map_server still loads and publishes
the complete map for Nav2 and AMCL. This avoids the startup deadlock observed
with the xlarge maps while preserving the requested 0.05 m navigation grid.
Use `sim:=dummy` when only validating map_server loading.

The xlarge Gazebo path was verified with `arena_xlarge_500x500_2d_005`:
Jackal spawned, `controller_server`, `planner_server`, and `bt_navigator`
reached `active`, AMCL pose and `map -> base_link` TF were available, and the
Arena baseline check passed.

## Source checksums

The following SHA-256 values identify the downloaded raster files:

```text
nav2_100by100_10/map/map.pgm  ce5456001916caa0b8049425cbadac4877f3889c79d1e48462df08a4c1cac72e
nav2_100by100_15/map/map.pgm  3088b3720fa263e9d596bb96aa04a92f84a2e2f06503580409a0d1a17399dd73
nav2_100by100_20/map/map.pgm  7dcbdd34cac9e007fb36deb3717aaf3988bd260e98f5069d9bd4cc497bec14b6
aws_small_warehouse_002/map/map.pgm  d512370ff571562e11b80fa1f55264836a0cc9c3a8cfd16bbd92d2b8f67a4827
```
