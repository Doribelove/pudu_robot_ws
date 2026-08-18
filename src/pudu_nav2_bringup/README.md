# pudu_nav2_bringup

Owns product launch files, Nav2/SLAM/localization parameters, maps, RViz
configurations and lifecycle startup policy.

Optional SCURM integration:

- `scurm_navigation.launch.py` runs the simulation-safe 2WD profile in
  `config/scurm_nav2.yaml`.
- `behavior_trees/scurm_navigation.xml` connects Theta* planning, constrained
  smoothing, MPPI tracking, and adaptive recovery.
- `scurm_lio.launch.py` exposes the pinned upstream FAST-LIO, Livox, and ICP
  localization pipeline without making it part of normal PUDU startup.
- `scurm_sim_gazebo.launch.py` starts the 3D lidar/IMU Gazebo robot without the
  Linorobot EKF, leaving `odom -> base_footprint` to FAST-LIO. Gazebo publishes
  wheel joint states while `robot_state_publisher` exclusively owns wheel TF.
- `scurm_sim_pipeline.launch.py` runs PointCloud2 FAST-LIO mapping or ICP prior
  map localization, optionally followed by Nav2 without AMCL.

The optional 1.5x runtime profile is selected with:

```bash
./start_linorobot.sh --scurm-lio --fast-sim
```

Without `--fast-sim`, the original Gazebo world physics settings are unchanged.
Normal mode also keeps the 360x16 lidar; the optional profile uses 120x16 while
preserving all vertical channels and the 10 Hz simulation-time rate.
