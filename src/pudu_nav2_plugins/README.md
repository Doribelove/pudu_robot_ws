# pudu_nav2_plugins

Owns reusable PUDU-specific Nav2 plugins. Keep cleaning task orchestration and
coverage generation in their dedicated packages.

## BackUpToFreeSpace

`pudu_nav2_plugins/BackUpToFreeSpace` is an optional replacement for Nav2's
standard backup behavior. Before moving, it samples collision-free forward and
reverse arcs in the local costmap. It is designed for differential-drive robots
and never commands lateral velocity.

The behavior is inspired by SCURM SentryNavigation's free-space recovery, but
uses Nav2's in-process collision checker instead of synchronously requesting a
full costmap. The default PUDU launch remains unchanged unless the optional
SCURM-style navigation profile is selected.
