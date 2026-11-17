#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
case "$(uname -m)" in x86_64|aarch64) ;; *) echo 'Supported CI architectures: Linux x86_64/aarch64 only.' >&2; exit 2 ;; esac
[[ "$(uname -s)" == Linux ]] || exit 2
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Stock ROS Humble is required.' >&2; exit 2; }
work="$root/.ci_ws"
mkdir -p "$work/evidence"
# Unix sockets must stay below the service's 104-byte path limit, independently
# of checkout length. mktemp creates this job-owned directory with mode 0700.
short_tmp="$(mktemp -d /tmp/tri-ci-XXXXXXXX)"
printf '%s\n' "$short_tmp" > "$work/evidence/tmpdir.txt"
# This isolated CI workspace never consults old /home/robot workspaces, wheels or maps.
export PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export TMPDIR="$short_tmp" ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=178
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
exec > >(tee "$work/evidence/build-test.log") 2>&1
collect() {
  status=$?
  trap - EXIT
  /usr/bin/python3 "$root/release/ci/collect-evidence.py" "$work" "$status" "$short_tmp" || true
  # Do not rm/rmdir here: cancellation may leave a live child using these files.
  # The disposable GitHub runner owns eventual cleanup after the job ends.
  exit "$status"
}
trap collect EXIT
/usr/bin/python3 "$root/release/ci/check-layout.py" "$root"
/usr/bin/python3 "$root/release/ci/test_architecture_support.py"
df -h "$work"
uname -a
dpkg --print-architecture
/usr/bin/python3 --version
/usr/bin/rustc-1.85 --version
/usr/bin/cargo-1.85 --version
# Stage 1 compiles all solver extensions from checked-in hash-verified source archives.
# Both vendor build scripts enforce offline dependency resolution. No binary pip install.
colcon --log-base "$work/log" build --base-paths "$root/src" \
  --build-base "$work/build" --install-base "$work/install" --merge-install \
  --executor sequential --packages-select trilayer_osqp_vendor trilayer_clarabel_vendor \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DTRILAYER_CLARABEL_CARGO=/usr/bin/cargo-1.85 \
  -DTRILAYER_CLARABEL_RUSTC=/usr/bin/rustc-1.85 -DTRILAYER_CLARABEL_JOBS=2
set +u
source "$work/install/local_setup.bash"
set -u
# Merged install gives all three source-built solvers one explicit origin directory.
solver_root="$(/usr/bin/python3 - "$work/install" <<'PY'
from pathlib import Path
import sys,osqp,qdldl,clarabel
prefix=Path(sys.argv[1]).resolve()
root=Path(osqp.__file__).resolve().parent.parent
assert root.is_relative_to(prefix),root
assert Path(qdldl.__file__).resolve().parent==root
assert Path(clarabel.__file__).resolve().parent.parent==root
print(root)
PY
)"
printf '%s\n' "$solver_root" > "$work/evidence/solver-root.txt"
# Stage 2 builds the other ten packages. The solver underlay is only the fresh install above.
colcon --log-base "$work/log" build --base-paths "$root/src" \
  --build-base "$work/build" --install-base "$work/install" --merge-install \
  --executor sequential --packages-skip trilayer_osqp_vendor trilayer_clarabel_vendor \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DTRILAYER_L3_SOLVER_DEPS="$solver_root" \
  -DTRILAYER_P6_TEST_SOLVER_ROOT="$solver_root" \
  -DTRILAYER_ENTRY_TEST_SOLVER_ROOT="$solver_root" \
  -DTRILAYER_SERVICE_TEST_SOLVER_ROOT="$solver_root" \
  -DTRILAYER_NAV2_TEST_SOLVER_ROOT="$solver_root"
set +u
source "$work/install/local_setup.bash"
set -u
# Reconfigure after every prefix exists, so configure-time test environments are complete.
colcon --log-base "$work/log" build --base-paths "$root/src" \
  --build-base "$work/build" --install-base "$work/install" --merge-install \
  --executor sequential --packages-skip trilayer_osqp_vendor trilayer_clarabel_vendor \
  --cmake-force-configure --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON
# Sequential package/test execution prevents collisions between fixed synthetic DDS domains.
colcon --log-base "$work/log" test --base-paths "$root/src" \
  --build-base "$work/build" --install-base "$work/install" --merge-install \
  --executor sequential --event-handlers console_direct+ --return-code-on-test-failure \
  --ctest-args --output-on-failure -j1
colcon test-result --test-result-base "$work/build" --verbose
