# TriLayer master snapshot

`trilayer_master_snapshot::MasterSnapshotLayer` is an independent, read-only Nav2
Layer plugin. It exposes the relative service `trilayer_get_costmap` using
`nav2_msgs/GetCostmap`. The service reads the actual
`LayeredCostmap::getCostmap()` under that Costmap2D's mutex, the same master mutex
used by Nav2 updateMap. Metadata, cells, hash and delta encoding all belong to the
same locked observation. `updateBounds` and `updateCosts` are no-ops: this plugin
never adds costs, changes bounds, or modifies the master grid.

## Configuration

Add `snapshot` to the existing costmap plugin list, preserving the other plugins:

```yaml
snapshot:
  plugin: trilayer_master_snapshot::MasterSnapshotLayer
  service_name: trilayer_get_costmap
```

Use the costmap namespace plus this service name for P6 readback. The service name
must be relative and is configurable at initialization. The standard system
`get_costmap` service and stock Nav2 libraries remain unchanged.

Normal requests return full master content. Valid `3yd_delta_v1|...` and
`3yd_delta_v2|...` requests use the unmodified frozen P6 encoder. Invalid delta
requests produce `3yd_delta_error` with empty data. Activation starts a new codec
epoch; reset requests a new epoch on the next observation; inactive/uninitialized
services return empty data. This plugin has no topic cache.

## What the ACK means

The original P6 ACK verifies observed effective-master content: affected cells
(source changes, old/new effective dirty cells, hard semantics and soft costs)
match exactly; stable observations meet the original count; publication state
and content signatures remain unchanged. The delta decoder verifies the complete
reconstructed grid SHA. The original server stamped snapshots at read time;
these timestamps are not server-applied publication revision numbers. This plugin
preserves that behavior and does not invent an applied-version ACK.

`test/p6_ack` contains the frozen original ACK/decoder source plus a test adapter
that obtains real DDS service snapshots. It exercises actual StaticLayer updates
and rejects missing changes, stale freshness and concurrent client publication
changes. These tests validate the content ACK path on synthetic maps, not the
complete planner or all optional performance modes.

## Thread and lifecycle boundaries

The service owns a non-auto-added callback group and a dedicated executor/thread.
A context gate serializes reads with activation/deactivation; it precedes the
master mutex. Bounds/cost updates never take that gate. Reset only sets an atomic
flag, avoiding inversion when Nav2 calls reset while holding master. Deactivation
waits for in-flight reads, then rejects new reads. Destruction cancels and joins
the dedicated executor before releasing service references. Callback context
outlives queued callbacks and cannot dereference the parent while inactive.

## Actual tests and host constraint

The test host loads the plugin through pluginlib into stock Humble Costmap2DROS
with a real StaticLayer, on isolated localhost DDS domains. It accepts full maps
and OccupancyGridUpdate, without a controller, robot driver or navigation action
server. Tests verify full/master content, unchanged maps after reads, incremental
updates, original delta requests, lifecycle deactivate/reactivate, and that a
snapshot waits while another callback holds the actual master mutex (350 ms).

An initial host cleanup crashed in rclcpp CallbackGroup destruction. A control
with **only stock StaticLayer and no TriLayer plugin** reproduced this installed
Humble library unload/lifetime defect. The test host now holds a supported
pluginlib ClassLoader reference to the stock StaticLayer library until NodeBase
has released its callback weak references. It does not pin the TriLayer library.
Final cleanup stops and destroys the host executor before Costmap2DROS cleanup.
Two full smoke runs with the stock library guard exited successfully; subsequent
CTest includes the final cleanup ordering and asserts exit code zero. This is an
explicit host lifecycle requirement for the tested installation, not a claim
that every production Nav2 host/unload configuration has been validated.

No original workspace, system library or robot configuration is changed by this
package. The maintainer address is a local prototype placeholder to replace
before release. All tests are bounded and operate on synthetic grids.

## Private pre-install test fixture

Default CTest first stages this build into `build/test_install` using an owned
fixture directory, without touching CMAKE_INSTALL_PREFIX. Both synthetic
localhost DDS tests require that fixture and assert their plugin package index
resolves this fresh stage, not an older installed version. Native test-host
executables are invoked directly from the current build.
