#!/usr/bin/env bash
set -euo pipefail
# Deliberately refuse privileged dependency setup on a developer's computer.
[[ "${GITHUB_ACTIONS:-}" == true && "${RUNNER_OS:-}" == Linux ]] || {
  echo 'This dependency installer is only for a disposable GitHub Actions Linux runner.' >&2; exit 2;
}
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && ( "$(uname -m)" == x86_64 || ( "$(uname -m)" == aarch64 && "${TRILAYER_EXPERIMENTAL_ARM64:-}" == 1 ) ) ]] || exit 2
mkdir -p "$root/.ci_ws/evidence"
exec > >(tee "$root/.ci_ws/evidence/dependency-install.log") 2>&1
sudo apt-get update
sudo apt-get install -y --no-install-recommends software-properties-common
sudo add-apt-repository -y universe
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git python3-rosdep python3-colcon-common-extensions \
  python3-dev python3-yaml python3-setuptools python3-wheel
# setup-ros may already have initialized rosdep. Never overwrite its defaults.
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
# These four mappings are a local CI workaround, NOT evidence of upstream acceptance.
printf 'yaml file://%s/release/ci/rosdep-ci.yaml\n' "$root" | \
  sudo tee /etc/ros/rosdep/sources.list.d/10-trilayer-ci.list
rosdep update --rosdistro humble
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install --from-paths "$root/src" --ignore-src --rosdistro humble -y
rosdep check --from-paths "$root/src" --ignore-src --rosdistro humble
# Fail early if the required official Jammy toolchain is unavailable; no curl/rustup fallback.
/usr/bin/rustc-1.85 --version
/usr/bin/cargo-1.85 --version
apt-cache policy rustc-1.85 cargo-1.85 python3-setuptools-scm python3-pybind11 \
  > "$root/.ci_ws/evidence/apt-policy.txt"
dpkg-query -W -f='${binary:Package}\t${Version}\n' \
  'ros-humble-*' rustc-1.85 cargo-1.85 python3-setuptools-scm python3-pybind11 \
  > "$root/.ci_ws/evidence/installed-dependencies.tsv"
cp "$root/release/ci/rosdep-ci.yaml" "$root/.ci_ws/evidence/CI_ONLY_NOT_UPSTREAM_rosdep.yaml"
