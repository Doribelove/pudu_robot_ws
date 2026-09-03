# PLN-02 2A-V1-r2 remote delivery

## Immutable revisions

- Root behavior commit: `57549bcc64d83f752f6560aeb65e5bd7b22bf67e`
- Nav2 upstream baseline: `8e34dcf5790671085a893dc58d2a0940ec80dd1e`
- Nav2 project fork: `https://github.com/Doribelove/navigation2.git`
- Nav2 instrumentation commit: `656ae8d4c56978efbdd446fe85582f2bcd06e920`
- Pinned vcs manifest: `dependencies/navigation2-2a-v1-r2.repos`
- Offline recovery patch: `dependencies/patches/2a-v1-r2-nav2-smac-instrumentation.patch`
- Recovery patch SHA-256: `2e52ba7e6a158626a05642e07552e4e9b2926a2fe0a18f3f6e9baf36e8a7d8b2`
- Reproducibility archive: `2a_v1_r2_roi_pathaudit_repro_bundle_v1_20260903T050314Z.tar.zst`
- Archive SHA-256: `156818afd8f7f63a900948f517bb8094cd486f49441c28903be6542c78920d3a`

The Nav2 commit contains runtime-gated benchmark instrumentation. Its `benchmark_instrumentation` parameter defaults to `false`, so the formal execution path does not enable the instrumentation. The production experiment uses 48 heading bins; this is an explicitly measured Smac parameter change selected by the A2B-19 single-variable ablation and must not be described as an unchanged-parameter result.

## Restore Nav2

Preferred restoration uses the exact commit, never the branch tip:

```bash
mkdir -p /tmp/2a-v1-r2-nav2-import
vcs import /tmp/2a-v1-r2-nav2-import < dependencies/navigation2-2a-v1-r2.repos
git -C /tmp/2a-v1-r2-nav2-import/navigation2 rev-parse HEAD
```

For an existing checkout at the upstream baseline, the offline patch is equivalent:

```bash
git -C external/arena4_ws/src/deps/nav2/navigation2 switch --detach 8e34dcf5790671085a893dc58d2a0940ec80dd1e
git -C external/arena4_ws/src/deps/nav2/navigation2 apply --check \
  /home/robot/pudu_robot_ws/dependencies/patches/2a-v1-r2-nav2-smac-instrumentation.patch
git -C external/arena4_ws/src/deps/nav2/navigation2 apply \
  /home/robot/pudu_robot_ws/dependencies/patches/2a-v1-r2-nav2-smac-instrumentation.patch
```

The patch was applied in an isolated worktree at `8e34dcf5`; all four resulting files matched commit `656ae8d4` byte-for-byte.

## Build and test

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/pudu_robot_ws/external/arena4_ws
colcon build --packages-select nav2_smac_planner --symlink-install
source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash
colcon build --packages-select arena_evaluation --symlink-install
python3 -m pytest -q \
  /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_two_layer_v1_r2_roi_pathaudit.py \
  /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_unified_four_backends_smoke.py \
  /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_l1_l3_corridor_hybrid_smoke.py \
  /home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_l1_l3_cache_optimization.py
colcon test --packages-select nav2_smac_planner
```

The complete Nav2 `pep257` program additionally requires Python package `pydocstyle`. On the delivery host, the code-only Nav2 subset passed 18/18 test programs, while the full aggregate correctly reported the missing dependency rather than a code pass. The combined Python suite passed 46 tests.

## Formal runner

Use a new output directory and unused ROS domain. This command preserves the formal static-map, forward-only, 48-bin ROI/content-ACK protocol:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/pudu_robot_ws/external/arena4_ws/install/setup.bash
ros2 run arena_evaluation two_layer_v1_r2_roi_pathaudit_benchmark \
  --output-dir /home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/2a_v1_r2_reproduction_NEW_TIMESTAMP \
  --costmap-mode roi_ack --endpoint-mode baseline \
  --warmups 3 --repetitions 5 --ros-domain-id 122 \
  --planner-overrides-json '{"angle_quantization_bins":48}' \
  --failure-parity-file /path/to/bundle/config/failure_parity_clean_commit.json \
  --no-dynamic-obstacles
```

The complete clean-room commands, source map, checksums, failure-parity files and validator are in `REPRODUCE.md` inside the reproducibility archive.

## Verified result

The post-commit delivery verification ran 20 queries with three warmups and five measured repetitions per query, 160 runs total:

- final-valid: **95/100**; V7 baseline: 90/100;
- successful online P50/P95/P99: **313.21 / 528.72 / 719.16 ms**;
- server-content ACK: 100/100; final mismatch cells: 0;
- accepted-path collision, reverse-motion, in-place-rotation and kinematic violations: 0;
- route, mask, failure code and canonical geometry parity with authoritative r2: 100/100.

A2B-16 remains unresolved and visible as five deterministic `NO_PATH_IN_CORRIDOR` failures. Full-map original-yaw, yaw-aligned and reverse diagnostics all failed. A2B-19 remains the dominant long tail despite the safe 48-bin improvement.

These results are limited to one map (`mentor_map_20260825_005`), one 20-query set, a static environment and hot topology/cache conditions. They do not establish cold-start, multi-map or dynamic-obstacle performance, and the measured successful P95 is 528.72 ms—not below 500 ms.
