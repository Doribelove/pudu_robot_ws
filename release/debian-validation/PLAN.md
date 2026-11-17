# Debian validation plan for the 12 TriLayer packages

**Review draft only.** No Debian build, install, smoke test, workflow edit, push, release or repository registration was performed for this task. A local `.deb` is not an official ROS apt package.

## 1. Verified Bloom behavior

Read official Bloom source at commit `eb82d36e658249b8f9621714724c1c26a9f48225`:

- [Local generate CLI](https://github.com/ros-infrastructure/bloom/blob/eb82d36e658249b8f9621714724c1c26a9f48225/bloom/generators/debian/generate_cmd.py): `bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble` accepts one package directory, rejects multiple packages, and accepts `--debian-inc`/`-i`. It creates local Debian metadata, not a release repository.
- [ROS Debian local get_subs](https://github.com/ros-infrastructure/bloom/blob/eb82d36e658249b8f9621714724c1c26a9f48225/bloom/generators/rosdebian.py): the local CLI explicitly does **not** supply the release generator's peer-package fallback. Unregistered TriLayer peers therefore require explicit local resolution.
- [Dependency substitutions](https://github.com/ros-infrastructure/bloom/blob/eb82d36e658249b8f9621714724c1c26a9f48225/bloom/generators/debian/generator.py): build/buildtool dependencies become Build-Depends, runtime/export dependencies become Depends, and test dependencies enter Build-Depends with `<!nocheck>`.
- [ament_cmake Debian rules](https://github.com/ros-infrastructure/bloom/blob/eb82d36e658249b8f9621714724c1c26a9f48225/bloom/generators/debian/templates/ament_cmake/rules.em): tests happen before dh_auto_install; `DEB_BUILD_OPTIONS=nocheck` disables testing. Crucially, the current template calls **`dh_auto_test || true`**, allowing a package build to succeed despite test failures.

Do not treat dpkg-buildpackage exit0 alone as proof that tests passed. The validation draft saves original generated rules, records a small explicit local rules patch, removes the `|| true`, enables BUILD_TESTING, and rejects unknown template shapes. This is a transparent stricter **local validation delta**, not an unmodified Bloom release claim. The actual runner's installed Bloom version and generated rules must be archived because apt Bloom may differ from this inspected commit.

## 2. Runner and source boundary

Use a new disposable Ubuntu22.04 amd64 GitHub Actions runner with an official signed ROS2 Humble apt source already configured. Review scripts first; set `TRILAYER_DEB_VALIDATION_REVIEWED=yes` only when deliberately running the draft. The runner must have no installed TriLayer package and no colcon/source overlay. Do not use the failed rootfs environment or a developer's workstation.

Inputs: an inspected source-only 12-package checkout. Copy each package into a new packaging workspace outside the checkout. Source remains unchanged. No workflow is added automatically. Keep all generated Debian metadata, patches, control/changelog, checksums, `.deb` metadata, CTest logs and runtime evidence as private job artifacts by default.

## 3. Transparent local rosdep rules

`prepare_graph.py` reads current package.xml dependencies, fails on unresolved conditional metadata/cycles, records the graph, and creates:

- 12 local keys mapping each `trilayer_*` to `ros-humble-trilayer-*` on Jammy;
- 4 provisional external keys: `python3-setuptools-scm`, `python3-pybind11`, `rustc-1.85`, `cargo-1.85`.

The 16-rule file is labeled **CI_ONLY_LOCAL_ROSDEP.yaml** and installed only in the disposable runner's rosdep configuration. `rosdep install --ignore-src` installs external dependencies, not unpublished peer binaries. Bloom then resolves peer dependencies into real expected Debian names. Earlier peers are built and installed in graph order. Do not use `--skip-keys`, empty mappings, `dpkg --force-depends`, or `dpkg-buildpackage -d` to hide missing dependencies.

These overrides do not constitute upstream acceptance. Existing `cargo-latest` could avoid two Rust rules after a separately validated1.91 transition; the draft preserves the tested1.85 line and does not perform that migration.

## 4. Current dependency order

Generated from present XML, including build/test/runtime inter-package edges:

1. trilayer_clarabel_vendor
2. trilayer_delta_codec
3. trilayer_l1_core
4. trilayer_master_snapshot
5. trilayer_native_helpers
6. trilayer_osqp_vendor
7. trilayer_costmap_bridge
8. trilayer_l3_core
9. trilayer_p6_runtime
10. trilayer_planner_service
11. trilayer_nav2_planner
12. trilayer_global_planner

This is one valid topological order, not a hard-coded substitute for the graph. `graph-snapshot/GRAPH.json` contains the current graph; execution regenerates it and fails if the package count or dependency structure is invalid.

## 5. Per-package Debian build/install gate

`runner-draft.sh` sketches the following steps for each package:

1. Assert **this package itself is not installed**. Already installed dependencies are allowed and must be owned by the previously built local debs or official distribution packages. A reused runner with an old own-package install is rejected.
2. Generate Debian metadata with `bloom-generate rosdebian ... -i 0localvalidation1`. The local increment distinguishes these test binaries; it is not an official release version. Binary-only validation does not produce/validate an official `.dsc` or bloom release repository.
3. Save original rules/control. Apply the strictly bounded test-failure patch, force BUILD_TESTING, and supply only the required already-installed solver origin to numerical tests. No numerical algorithm or solver source is changed.
4. Run `dpkg-checkbuilddeps`; reject failure. Set `DEB_BUILD_OPTIONS=parallel=2`, not nocheck; clear build profiles; build using `dpkg-buildpackage -us -uc -b -rfakeroot`.
5. Tests must run **before installation of the current package**. Vendors use their own newly compiled build-tree Python extensions. L3/P6/Nav2/master use their CMake-defined private CTest staging fixtures for their own files; that is not a system installation. Do not point own-package tests at `/opt/ros/humble` before its binary has passed.
6. Select the expected binary by `dpkg-deb -f ... Package`, preserve its metadata and contents, then install that exact file with apt. Only now can later packages use it.

`patch_rules.py` sets each numerical test option separately: L3 `TRILAYER_L3_SOLVER_DEPS`, P6 `TRILAYER_P6_TEST_SOLVER_ROOT`, public entry `TRILAYER_ENTRY_TEST_SOLVER_ROOT`, service `TRILAYER_SERVICE_TEST_SOLVER_ROOT`, Nav2 `TRILAYER_NAV2_TEST_SOLVER_ROOT`. Without these explicit roots some real tests are not registered, so a small passing CTest subset is insufficient.

The solver origin is derived from **dpkg-query's installed file list**, not hard-coded to the previous colcon `local/lib` layout. Actual `.deb` builds may choose a different Python installation directory. Verify OSQP/QDLDL/Clarabel share the intended installed prefix and ownership.

## 6. Installed-deb P6/Nav2 smoke

After all 12 binaries are installed, `postinstall-smoke-draft.sh`:

- clears colcon/build/source environment overlays and sources only `/opt/ros/humble/setup.bash`;
- verifies imports for solver/runtime/service/L3 modules resolve under `/opt/ros/humble` and are owned by `ros-humble-trilayer-*` debs;
- copies only standalone test-driver scripts into an artifact directory, not runtime implementation modules;
- runs real P6 straight/obstacle requests and map/semantic persistent sessions on generated synthetic maps;
- runs the real worker plus real Nav2 planner_server `ComputePathToPose` contract, including same-snapshot PID reuse and changed-snapshot rebuild;
- uses short job-owned TMPDIR, localhost-only DDS domains and bounded subprocess cleanup. Tests run sequentially on a dedicated runner, never connect to a robot or external ROS graph.

Do not run physical-robot/J6M claims from these smoke results. Preserve logs and worker cleanup evidence, and fail on any timeout/assertion/nonzero exit. Installed module ownership avoids silently testing the source checkout or a prior wheel.

## 7. Evidence and limitations

Required evidence: source commit/diff; generated graph/local rules; all external package versions; Bloom version/help and original/modified rules; generated control/changelog; per-package build/CTest logs and counts; every `.deb` SHA256/info/contents; installed package list; import ownership; P6/Nav2 runtime and cleanup reports.

This sequential runner accumulates dependencies. It exercises the real Debian package lifecycle and own-package preinstall tests, but **does not prove each Build-Depends list is minimal/complete in an independently minimal buildfarm container**. Installing all external workspace dependencies can mask undeclared dependencies; dpkg-checkbuilddeps proves satisfaction, not completeness. A subsequent stronger check should use a fresh sbuild/buildfarm-like environment per package, with only declared Build-Depends plus previously built peer debs. This draft does not implement or claim that tier.

Official apt publication still needs accepted external rosdep rules, proper bloom/release repository and rosdistro entry, successful official binary jobs and repository sync. No local override or local `.deb` closes those gates.

## 8. What was checked during preparation

Only source/document inspection, XML graph generation, Python syntax, shell syntax and synthetic generated-rules patch assertions are performed for this plan. No apt/rootfs operations, Debian builds, binary installs, runtime smoke, workflow edits or pushes are performed. All executable scripts remain guarded review drafts. Review generated output and actual Bloom behavior before enabling any runner job.

## Second review: version, serialization, failure artifacts

The audited Bloom commit accepts `-i` as a string (argparse has no integer
converter), passes `args.debian_inc` unchanged to substitutions, and formats
`DebianInc` as `-{0}`. The changelog template concatenates upstream version,
DebianInc and distribution. Thus version 0.1.0 produces
`0.1.0-0localvalidation1jammy`. `dpkg --validate-version` accepts this syntax.
The draft additionally checks every actually generated changelog at runtime;
this source audit is not a claim that Bloom was executed during preparation.

Strict test replacement is now
`CTEST_PARALLEL_LEVEL=1 dh_auto_test --max-parallel=1`; the driver also exports
`CTEST_PARALLEL_LEVEL=1`. Compilation remains parallel=2. Existing test timeout
budgets are unchanged. Package builds and post-install smoke calls remain
sequential. This prevents the Bloom/debhelper test runner from concurrently
using the fixed localhost ROS domains.

Copied smoke-driver closure was inspected:
- `test_service_request.py`: installed P6 API and ament package resource share;
  NumPy/Pillow/YAML and solver imports, no sibling test import.
- `test_public_session.py`: installed P6 API/session, NumPy/Pillow, no sibling
  file dependency.
- `run_real_worker_smoke.py`: installed planner service process/CLI and sibling
  `planner_server_smoke.py`, which is copied alongside it.
- `planner_server_smoke.py`: installed ROS Python messages/rclpy/tf2/YAML and
  `/opt/ros/humble/lib/nav2_planner/planner_server`; generated request/map files
  from the service's temporary output. It requires no source fixture file.
All four scripts are copied; package resources/executables must come from
installed dependency debs, not copied checkout modules. This is a static
closure audit, not post-install success evidence.

The driver EXIT trap preserves the original exit status and current stage,
runs the bounded diagnostics collector on success or failure, and records the
installed-package list. `evidence/EXIT_STATUS.txt` and `driver.log` are the
first files to inspect. `evidence/diagnostics/` includes CTest last-test and
failed-test logs, ament result XML, CMake diagnostics and smoke temporary
JSON/log/YAML/TXT, with a manifest listing skipped large files (2 MiB per file,
64 MiB copied total). Raw build tree/TMPDIR remain on the ephemeral runner but
are not copied wholesale. The collector does not copy sockets, binaries,
source archives or credentials. An external reviewed workflow should use an
`always()` upload step for the known absolute `<output>/evidence` path; absence
of an artifact or a nonzero exit is not a passing validation. `<output>/debs`
may be retained separately only as experimental binaries, not apt releases.
The guard/platform/output-creation checks run before the trap: failures there
are visible only in the Actions job log, with no artifact promised.

## Native dual-architecture follow-up

The workflow now validates native Ubuntu 22.04 amd64 and arm64 independently, based on the consolidated supported source revision. It uses increment `0localvalidation2`, distinct from the earlier amd64-only preview. It still uses explicit local rosdep rules and strict local test rules, not an official apt release. No cross-compiled or cached amd64 binary is reused on ARM.
