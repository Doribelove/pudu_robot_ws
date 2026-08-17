# Third-party dependency policy

Third-party repositories must not be copied directly into `src/`.

- Use `~/nav2_reference_ws` as the Nav2/TurtleBot underlay.
- Use `~/linorobot_sim_ws` only as an independent integration reference.
- Use `~/exploration_reference_ws` as the pinned `m-explore-ros2` underlay;
  reproduce its source with `dependencies/exploration.repos`, then apply
  `dependencies/patches/m-explore-linorobot-frontier-goal.patch`. The patch
  avoids selecting the robot-centred arithmetic centroid of a surrounding
  frontier and keeps failed-goal blacklisting consistent with the selected
  target point.
- Add future source dependencies to a reviewed `.repos` manifest and import
  them into a separate underlay workspace.
- Pin manifests to reviewed commit SHAs, record licenses, and update them
  deliberately.

This keeps product packages buildable against controlled interfaces and makes
upstream upgrades auditable.
