# PLN-02 2A-V2 r4 SE(2) guide/interface feasibility report

## Identity and verdict

- `architecture_id`: `2A-V2`
- `revision_id`: `r4-se2-guide-interface`
- `protocol_id`: `PLN-02-2A-V2-R4-SE2-GUIDE-INTERFACE-V1`
- final verdict: **C1 — Stage 2 bounded feasibility gate failed**
- online E5: `NOT_RUN_STAGE2_GATE_FAILED`
- frozen selected8: `NOT_RUN_STAGE2_GATE_FAILED`
- production promotion: **prohibited**

This is a time-boxed C1 result: no qualifying witness was produced for mirror
positive within the frozen one-million-state / 120 s oracle budget. Because the
search stopped on time rather than exhausting its open set, the result is not a
mathematical proof that no path exists in the continuous world. It is sufficient
for the protocol's hard stop, but the distinction must be preserved.

## Frozen parent facts

r3 remains unchanged and retains its C verdict. Its two-dimensional guide passed
both frozen mirror queries offline, while its real online results were:

- mirror positive: correct-side `0.0`, lateral error `4.950 m`;
- mirror negative: correct-side `1.0`, lateral error `0.950 m`;
- long south: correct-side `1.0`, lateral error `0.534 m`;
- R0, exact ACK, latency and static safety passed.

The r4 audit also found that the immutable historical r3 ROS parameter file uses
`angle_quantization_bins: 72`, although the r3 report described 48 bins. Historical
r3 data were not rewritten. r4 uses the explicitly required 48 bins. Consequently,
historical E0/E4-r3 rows are context, not a new fair 48-bin E0/E4/E5 comparison.

## Stage 1: native Smac interface audit

Pinned Humble `SmacPlannerHybrid` cannot consume an external per-state SE(2)
reference corridor. `nav2_core::GlobalPlanner::createPlan()` supplies start and
goal, and Smac owns a two-dimensional master costmap plus scalar search penalties.
There is no public input for a per-state corridor, yaw-bin admissibility or
reference-deviation term.

Therefore another two-dimensional raster cannot honestly be called an SE(2)
interface. The only allowed E5 path would be a separate experimental planner
adapter/plugin, explicitly labelled as a new algorithm arm. The protocol forbids
implementing it before all three Stage 2 witnesses exist.

Detailed evidence and source line bindings are in
`r4_stage1_smac_interface_audit_20260904_01/smac_interface_audit.md` and its JSON.
Pinned Nav2 source was not modified.

## Stage 2: what r4 actually implemented

r4 implements a genuine **offline** explicit SE(2) guide oracle, not an online
E5 interface. Its state is `(row, column, yaw_bin)` with exactly 48 yaw bins. It:

1. reproduces the pinned Smac formula for straight, forward-left and
   forward-right DUBIN projections;
2. forbids reverse and rotate-in-place motion;
3. fixes `Rmin=0.40 m` and `|curvature|<=2.50 1/m`;
4. dense-samples each primitive at at most `0.025 m` and sweeps the complete
   Jackal footprint with Nav2's `0.01 m` padding;
5. checks the r3 R0 exact expected effective master, including static, semantic,
   ROI-boundary and inflation content;
6. rejects a primitive whose centre leaves the selected semantic lane instance;
7. explicitly consumes the directed r3 reference position and reference yaw-bin
   sequence in its successor cost;
8. binds map, semantic map, query, endpoint poses, route, ROI, footprint, bin
   count, motion model, expected-master hash and SE(2)-reference hash;
9. uses deterministic tie-breaking and reports primitive traces, expanded yaw-bin
   distribution, cost decomposition and explicit failure codes.

This oracle is a feasibility-first experimental implementation. It is not native
Smac, does not publish `nav_msgs/Path`, and is not production code.

## Targeted results

The authoritative result is `r4_stage2_se2_oracle_targeted_v4`. All queries kept
their frozen x/y/yaw, order, route policy, map, lane instance, footprint and R0
costmap.

| targeted query | witness | side ratio | lateral P50 | target-band ratio | max curvature | reverse / in-place | path length | oracle wall | result |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `r3-mirror-1-positive` | no | N/A | N/A | N/A | N/A | N/A | N/A | 120.002 s | `SEARCH_TIMEOUT` after 344,280 states |
| `r3-mirror-2-negative` | yes | 1.000 | 0.396 m | 0.600 | 2.50 1/m | 0 / 0 | 9.143 m | 0.326 s | pass |
| `cmp2-02-lane-south` | yes | 1.000 | 0.350 m | 0.613 | 2.50 1/m | 0 / 0 | 57.835 m | 7.574 s | pass |

The best valid-footprint analytic candidate observed for mirror positive had
correct-side ratio `0.734`, lateral P50 `2.575 m`, target-band ratio `0.0`, no
reverse and no in-place rotation. It therefore missed all semantic acceptance
requirements materially; `0.734` is not rounded into the `0.80` threshold.

The positive geometry explains the asymmetry. Its target-side r3 guide lies about
three metres laterally from the frozen endpoints over only about 8.5 m of route,
while the final pose also demands a 135-degree yaw. The 48-bin forward-Dubins
search can reach many collision-free goal connectors, but 40,144 valid connectors
still failed the frozen path-level semantic metrics before timeout. This is
evidence of endpoint/short-route motion-geometry tension, not an ACK or static
safety regression. It remains bounded evidence rather than a completeness proof.

The successful witnesses had full-footprint collision-free paths, no reverse,
no in-place rotation, and curvature at the fixed limit. Their minimum approximate
clearance margins were `0.902 m` and `0.202 m`, respectively. No online canonical
PathAudit was run because E5 was gated off.

## E0 / E4-r3 / E5-r4 comparison boundary

Historical 72-bin r3 context is shown without relabelling the arms:

| query | E0 side / error | E4-r3 side / error | E5-r4 online |
|---|---|---|---|
| mirror positive | `0.0 / 3.300 m` | `0.0 / 4.950 m` | not run |
| mirror negative | `1.0 / 1.350 m` | `1.0 / 0.950 m` | not run |
| long south | `1.0 / 1.540 m` | `1.0 / 0.534 m` | not run |

There is no new fair 48-bin online E0/E4-r3/E5-r4 result. Starting those arms
after the Stage 2 failure would violate the frozen protocol. No E5/E0 latency
ratio can be reported.

## ACK, safety and performance

- r3 historical exact effective-content ACK remains passed and unchanged.
- Stage 2 only reads the exact expected-master model; server content ACK is
  `NOT_APPLICABLE_OFFLINE_STAGE2`.
- No r4 ROS costmap was published, so claiming r4 exact ACK would be false.
- Successful offline witnesses passed full-footprint collision, lane-instance,
  reverse, in-place and curvature checks.
- Mirror positive did not produce a qualifying path, so safety fields are N/A,
  not false and not counted as a violation.
- Oracle peak process RSS reached about `1.64 GB`; this is research tooling and
  is not an acceptable online planner memory result.
- Online cold-request latency and the `<=2.0x E0` gate are not measured because
  E5 was not permitted to exist.

## Stopped stages

- Stage 3 independent E5 adapter/plugin: `NOT_RUN_STAGE2_GATE_FAILED`.
- Stage 4 real ROS/Nav2 targeted comparison: `NOT_RUN_STAGE2_GATE_FAILED`.
- Stage 5 frozen selected8: `NOT_RUN_STAGE2_GATE_FAILED`.
- 30–50 query expansion: not eligible and not started.

No query was moved, yaw changed, endpoint reattached, safety mask relaxed,
iteration budget raised, near-lethal preference introduced or pinned Nav2 source
changed.

## Discarded diagnostic directories

- `r4_stage2_se2_oracle_targeted_v1`: excluded because each state retained dense
  Python sample tuples, causing about 2.15 GB RSS and a performance-driven timeout.
- `r4_stage2_se2_oracle_targeted_v2`: excluded because it searched SE(2) states
  but had not yet consumed the guide's position/yaw reference explicitly.
- `r4_stage2_se2_oracle_targeted_v3`: valid explicit-reference precursor; v4 adds
  best-rejected-candidate telemetry without changing policy or gates.

All directories are retained. None was overwritten or deleted.

## Verification

- r0-r4 semantic regression: `59 passed`;
- full package test in the isolated ROS/install environment: `377 passed`;
- isolated `colcon build --packages-select arena_evaluation`: passed;
- r4 installed CLI `--help`: passed;
- `/usr/bin/python3 compileall`: passed;
- `git diff --check`: passed;
- the first full-test attempt (`376 passed, 1 failed`) is excluded because the
  existing install could not locate `arena_evaluation`; the failed test passed
  after the isolated build;
- the first ordinary colcon attempt is excluded because a pre-existing build
  symlink pointed to `/home/robot/arena4_ws`; the successful build used the fresh
  isolated directory `/tmp/pln02_2a_v2_r4_build.nUk3dB` and did not clean the
  shared workspace.

## Conclusion and next architecture decision

**C1. 2A-V2 must stop for this three-query target and must not be promoted or
used as a production semantic planner.** r4 did not show that an explicit SE(2)
guide can satisfy all targeted cases, and the protocol correctly prevented an
online adapter experiment from turning a failed offline gate into expensive
trial-and-error.

The next step should not be another 2A-V2 soft-weight sweep. If mirror positive
is a mandatory product requirement with the exact frozen endpoint poses, first
run a completeness-oriented constrained state-lattice/optimal-control feasibility
proof that tracks path-level side/target-band budgets rather than collapsing all
histories at one `(x,y,yaw_bin)` state. Only if that proof produces a witness is
it rational to define **2A-V3** as a restricted state-lattice/reference-trajectory
planner. If it proves infeasibility, the product must revisit the query endpoint
or semantic acceptance definition explicitly; a new planner architecture cannot
solve an impossible contract. r4 itself does not implement 2A-V3.

## Key artifacts

- implementation: `external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/se2_semantic_guide.py`
- runner: `external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/two_layer_v2_semantic_r4_benchmark.py`
- config: `external/arena4_ws/src/arena/evaluation/arena_evaluation/config/two_layer_v2_semantic_r4.yaml`
- tests: `external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_v2_semantic_r4.py`
- Stage 0 freeze: `private_data/pudu_wanda_3f/results/r4_stage0_freeze_20260904_01`
- Stage 1 interface audit: `private_data/pudu_wanda_3f/results/r4_stage1_smac_interface_audit_20260904_01`
- authoritative Stage 2: `private_data/pudu_wanda_3f/results/r4_stage2_se2_oracle_targeted_v4`

No files were staged, committed or pushed.
