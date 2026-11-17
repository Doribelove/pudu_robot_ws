# TriLayer costmap snapshot bridge

A separate ROS 2 service that reads a standard `nav2_msgs/Costmap` topic and
returns validated snapshots through `nav2_msgs/GetCostmap`. It does not replace
or modify the system Nav2 costmap implementation.

Parameters (startup-only): `topic=costmap_raw`, `service=get_costmap`,
`max_age=1.0` seconds since receipt, `enable_delta=false`. Names are relative so
the caller can isolate the bridge in a namespace. Input QoS is reliable/volatile.

With `enable_delta=true`, the bridge uses `trilayer_delta_codec` and the unchanged
original P6 encoder. Requests use `3yd_delta_v1|token|epoch|sequence` or v2. Replies
have the original `3YDACK06`/`3YDACK07` wire format and owner binding. The frozen
P6 `DeltaDecoder` is used directly in integration tests to validate reconstruction
and one-use SHA evidence. Invalid requests return an empty map with the
`3yd_delta_error` layer. Normal full reads remain available.

A malformed input map clears the previous cache. No map or an expired map returns
an empty response, not old planning data. Expiry, an upstream gap beyond max_age,
and invalid input reset the codec epoch. Geometry changes force full snapshots.
The original protocol already forces full resynchronization after a dropped
reply, stale base, owner switch, identity change or restart.

## Explicit differences from the original in-process Nav2 publisher

- The bridge observes a topic snapshot, not the master map under Nav2's update
  mutex. It cannot prove that the topic includes the latest acknowledged obstacle
  update. Full P6 ACK integration must establish that causal link separately.
- Response timestamps and geometry come from the accepted topic message rather
  than being overwritten with service-request time.
- Epoch/cache resets on malformed input and stream expiry are additional safety
  behavior. Receipt age is not proof of source observation age.
- Request parsing requires canonical unsigned decimal uint64 fields and no extra
  fields; the old C++ string parser accepted some noncanonical integer syntax.
- Unknown normal layers are rejected; the original callback returned master.
- The protocol codec is compatible for valid requests, but this package is not
  the full three-layer planner or the complete ACK lifecycle implementation.

## Tests

`full_snapshot_dds_smoke` verifies full reads and invalid/expired map behavior.
`p6_delta_dds_smoke` sends real DDS service requests, runs the original P6 decoder,
and covers full/closed-full/delta/no-change, dropped replies, out-of-order replies,
checksum corruption, owner/version switching, malformed requests, expiry and
upstream restart/gap recovery. Tests use localhost, unique namespaces and separate
explicit ROS domains. No robot drivers or navigation action servers are started.

The manifest's maintainer address is a temporary placeholder for local prototyping
and must be replaced before release.
