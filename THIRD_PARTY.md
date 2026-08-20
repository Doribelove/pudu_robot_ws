# Third-party dependency policy

Third-party repositories must not be copied directly into `src/`.

- Use `external/nav2_reference_ws` as the Nav2/TurtleBot underlay.
- Use `external/linorobot_sim_ws` as an independent integration overlay.
- Use `external/exploration_reference_ws` as the pinned `m-explore-ros2` underlay;
  reproduce its source with `dependencies/exploration.repos`, then apply
  `dependencies/patches/m-explore-linorobot-frontier-goal.patch`. The patch
  avoids selecting the robot-centred arithmetic centroid of a surrounding
  frontier and keeps failed-goal blacklisting consistent with the selected
  target point.
- Use `external/scurm_sentry_ws` as the optional sentry-navigation underlay. Its five
  pinned source repositories are recorded in `dependencies/scurm_sentry.repos`.
  `build_scurm_reference.sh` applies the reviewed patches in
  `dependencies/patches/scurm-*.patch`, including omitted behavior sources,
  deterministic ICP frame publication, and missing first-build dependency
  declarations.
- Keep `external/arena4_ws` isolated from the PUDU overlay. It carries its own
  large ROS/Arena environment and is exposed only through the independent
  `build_arena4.bash`, `start_arena4.sh`, and `stop_arena4.sh` entry points.
  `build_arena4.bash` applies the reviewed
  `dependencies/patches/arena4-jackal-baseline.patch`, which removes duplicate
  TF/lifecycle publishers and the disabled collision-monitor process.
- Add future source dependencies to a reviewed `.repos` manifest and import
  them into a separate underlay workspace.
- Pin manifests to reviewed commit SHAs, record licenses, and update them
  deliberately.

This keeps product packages buildable against controlled interfaces and makes
upstream upgrades auditable.

The SCURM repository has no single root license and several packages still
declare `TODO: License declaration`. Therefore its source, maps, robot model,
and competition decision stack remain in the reference underlay. PUDU product
code only links packages with an identified license or independently implements
the architectural idea; redistribution still requires a package-by-package
license review.
