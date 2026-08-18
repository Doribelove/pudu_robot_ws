#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"
readonly SCURM_WS="${SCURM_REFERENCE_WS}"
readonly SCURM_REPO="${SCURM_WS}/src/SCURM_SentryNavigation"
readonly MANIFEST="${PUDU_WS}/dependencies/scurm_sentry.repos"
readonly PATCH_FILES=(
  "${PUDU_WS}/dependencies/patches/scurm-rm-decision-missing-sources.patch"
  "${PUDU_WS}/dependencies/patches/scurm-icp-source-frame.patch"
  "${PUDU_WS}/dependencies/patches/scurm-icp-dependencies.patch"
  "${PUDU_WS}/dependencies/patches/scurm-rm-decision-dependencies.patch"
)

mkdir -p "${SCURM_WS}/src"

vcs import --recursive --skip-existing "${SCURM_WS}/src" < "${MANIFEST}"

for patch_file in "${PATCH_FILES[@]}"; do
  if git -C "${SCURM_REPO}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
    echo "SCURM build patch is already applied: ${patch_file##*/}"
  elif git -C "${SCURM_REPO}" apply --check "${patch_file}"; then
    git -C "${SCURM_REPO}" apply "${patch_file}"
  else
    echo "Error: the pinned SCURM patch no longer applies cleanly: ${patch_file}" >&2
    exit 1
  fi
done

set +u
source /opt/ros/humble/setup.bash
source "${NAV2_REFERENCE_WS}/install/setup.bash"
set -u

cd "${SCURM_WS}"
colcon_args=(
  build
  --symlink-install
  --packages-ignore pcl_conversions
)
if [[ "${PUDU_CLEAN_CMAKE_CACHE:-false}" == "true" ]]; then
  colcon_args+=(--cmake-clean-cache)
fi
colcon_args+=(--cmake-args -DCMAKE_BUILD_TYPE=Release)
colcon "${colcon_args[@]}"

echo "SCURM reference workspace built successfully: ${SCURM_WS}"
