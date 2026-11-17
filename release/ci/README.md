# Independent Humble CI: native amd64/arm64 runs verified; revised matrix pending

This is the **locally prepared dual-architecture revision**, not a completed run
of this new revision, a release, or buildfarm approval. This revision task did
not push or execute a remote workflow. Prior successful exact-commit runs are
recorded below. The original template-preparation records are retained as
historical evidence; no host apt settings or clean-rootfs workspace changed here.

## Current native architecture evidence

Native Ubuntu 22.04/Humble amd64 and aarch64 source builds plus all 47 synthetic CTests passed on GitHub Actions. ARM evidence: [run 44069917303](https://github.com/Doribelove/pudu_robot_ws/actions/runs/44069917303), commit `322401744b079e11ced8bf54f69785147d0b6d5a`; the runner reported `uname=aarch64`, `dpkg=arm64`. This is native CI coverage, not Jetson/Orin, Horizon J6M/BPU, or real-robot validation. The new dual-architecture workflow revision still requires its own subsequent CI.

AMD64 evidence: [run 44068616506](https://github.com/Doribelove/pudu_robot_ws/actions/runs/44068616506), commit `6efcf157ff618d01028f0725377bd51d49fafbb2`. Exact run/artifact identifiers and boundaries are in `ARCHITECTURE_VALIDATION.json`. Historical preparation evidence below is retained, not rewritten as a successful run of this new revision.

## Required repository layout

Use a dedicated source repository or dedicated root containing exactly:

```
.github/workflows/trilayer-humble.yml  # copy the template here when approved
release/ci/                           # these scripts and CI-only rules
src/trilayer_clarabel_vendor/         # includes filtered source archive + notices
src/trilayer_osqp_vendor/             # includes OSQP/QDLDL source sdists + notices
src/trilayer_native_helpers/
src/trilayer_delta_codec/
src/trilayer_costmap_bridge/
src/trilayer_master_snapshot/
src/trilayer_l1_core/
src/trilayer_l3_core/
src/trilayer_p6_runtime/
src/trilayer_global_planner/
src/trilayer_planner_service/
src/trilayer_nav2_planner/
```

This is **not** a drop-in workflow for every historical package in the original
large robot workspace. Keep the twelve independent package directories intact,
including source manifests, license notices and source tarballs. The current
Clarabel archive is around 17 MiB and fits an ordinary Git blob; this workflow
does not request Git LFS or submodule credentials. Never replace source archives
with old binary wheels, installed `.so` files or private input maps.

To enable later, copy `trilayer-humble.yml` to the `.github/workflows/` path above
and commit it through the authorized publishing workflow. Do not copy the local
validation evidence as a success badge. Add `.ci_ws/` to repository ignore rules.

## What the job does

1. A matrix of fresh GitHub-hosted **ubuntu-22.04 x64** and **ubuntu-22.04-arm arm64** runners checks out the repository with
   read-only contents permission and `persist-credentials: false`. No secrets,
   external credentials or `pull_request_target` execution are used.
2. ROS Tooling's pinned `setup-ros` action installs official Humble packages, with
   testing repositories and Connext disabled.
3. `install-deps.sh` enables official Ubuntu universe and installs manifest
   dependencies using rosdep. It refuses to run outside GitHub Actions Linux.
4. Four **CI-only** local rosdep rules map the current missing manifest keys to
   official Jammy apt packages: python3-setuptools-scm, python3-pybind11,
   rustc-1.85 and cargo-1.85. They are written only into the disposable runner's
   `/etc/ros/rosdep/sources.list.d/10-trilayer-ci.list`. This is NOT upstream
   acceptance, a system requirement on the user's computer, or proof of ROS
   buildfarm readiness. If they are upstreamed later, review/remove the local
   override rather than silently keeping it forever.
5. `check-layout.py` verifies exactly twelve source packages, rejects checked-in
   `.so`/`.whl`/`.a`/`.o` runtime artifacts, and checks solver source archive hashes.
6. OSQP/QDLDL and Clarabel are compiled from the checked-in sources into a fresh
   **merged install**. The original vendor scripts enforce offline source
   dependency resolution. Rust/Cargo come from official versioned Ubuntu apt
   packages; there is no `curl | sh`, rustup download or pip binary fallback.
7. The other ten packages build against that solver underlay. The discovered
   merged site-packages path enables actual P6, CLI, socket-worker and Nav2
   integration tests, not merely import tests. All packages are reconfigured once
   all prefixes exist, then tested serially to prevent synthetic DDS collisions.
8. The job always uploads only CI-owned logs, CTest evidence and source-build
   provenance for fourteen days. It never uploads home directories, credentials,
   private maps or an entire installed environment. A failing command preserves
   its failure exit status; evidence collection cannot turn it green.

The job does use network access for official Actions, apt and rosdep setup.
"Offline" here refers specifically to the solver source compilation closure,
not to the whole GitHub job. Runtime inputs are generated synthetic test data.
The job's working files live under repository-local `.ci_ws/`, with a bounded
90-minute job and individual test/process bounds from the packages.

## Pinned official upstream sources

Full immutable commit pins were resolved with read-only `git ls-remote --tags`
and the corresponding official `action.yml` was retrieved and hashed. Metadata
and exact hashes are in `upstream-evidence/ACTION_PINS.json`.

- [actions/checkout v6.1.0](https://github.com/actions/checkout/tree/d23441a48e516b6c34aea4fa41551a30e30af803)
- [actions/upload-artifact v6.0.0](https://github.com/actions/upload-artifact/tree/b7c566a772e6b6bfb58ed0dc250532a479d7789f)
- [ROS Tooling setup-ros 0.7.19](https://github.com/ros-tooling/setup-ros/tree/649ef6bcd696da05bc27ceb3fab69d810c0daeab)

The inspected action metadata uses Node 24. The ROS action explicitly lists
Humble and exposes the three inputs used by this template. Pins are reviewed
inputs, not a promise that upstream actions themselves never need updates.
[GitHub's runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners)
currently lists `ubuntu-22.04` x64 and `ubuntu-22.04-arm` for public repositories.
Native ARM testing is now independently recorded above. The matrix uses these
two official runner labels, fail-fast=false, and architecture-qualified artifact
names so results do not collide. This does not validate board-specific hardware.

For the local-rule rationale and official Ubuntu package references see
`../rosdep/README.md` when included in the release preparation bundle. Those
proposals are independently unsubmitted; this CI file does not submit them.

## Validation status and remaining gates

Performed locally without installation or CI execution:

- `bash -n` on both shell scripts;
- Python compile checks on both helper scripts;
- YAML parse, supported trigger/read-only permission and exact action-pin checks;
- twelve-package source layout and source-archive hash checks via a disposable
  validation harness that read the source tree without changing it.

See `LOCAL_VALIDATION.json` and `upstream-evidence/LOCAL_SOURCE_LAYOUT.json`.
No GitHub runner job, privileged dependency setup, source build or test suite was
executed by this CI-preparation task. Those statements describe the original preparation task. Subsequent exact
remote runs are recorded above; this newly revised matrix must still pass its
own run before being described as green. Broader license review,
upstream rosdep acceptance, bloom/rosdistro release work and board-specific runtime/hardware
validation remain separate gates.

## Short test temporary paths

`build-and-test.sh` creates a private mode-0700 `/tmp/tri-ci-XXXXXXXX`
directory with `mktemp -d` and exports it as `TMPDIR`. This avoids coupling
Unix socket length to GitHub checkout paths: the service CLI rejects paths of
104 bytes or more, stricter than the Linux AF_UNIX limit. The existing service
test's longest child is `lowspace`, and Nav2 uses `worker`. A simulated long
checkout produces 228/236-byte paths with the previous scheme, versus 64/72
bytes with the short directory. See `upstream-evidence/TMPDIR_VALIDATION.json`;
this is a static path check, not a ROS integration run.

The exact temporary directory is recorded in `evidence/tmpdir.txt`. The exit
collector copies selected log/result files from that job-owned directory into
`evidence/collected/test-tmp`, with the same 32 MiB per-file cap. It does not
read sockets or symlinks, collect maps/binaries, or delete temporary files:
cancellation can leave children alive, so cleanup belongs to disposable-runner
teardown, not an EXIT trap deleting a running worker's state. Existing test
process cleanup remains the tests' responsibility.
