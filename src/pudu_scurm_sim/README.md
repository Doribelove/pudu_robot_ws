# PUDU SCURM simulation support

This package contains simulation-only tools used by the optional SCURM
FAST-LIO profile:

- `localization_error.py` compares `map -> base_link` with Gazebo ground truth,
  publishes scalar/vector errors and writes a CSV report.
- `ground_truth_map_builder.py` accumulates `/scurm/lidar_points` in the Gazebo
  world frame to make a deterministic reference PCD for localization tests.
- `generate_3d_map.py` can extrude a ROS occupancy map into a coarse PCD when a
  ground-truth sensor capture is not available.

The tools do not run in the default Linorobot2 or AMCL profiles.
