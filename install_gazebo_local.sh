#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"
readonly LOCAL_DEPS_ROOT="${NAV2_LOCAL_DEPS_ROOT:-${NAV2_REFERENCE_WS}/local_deps}"
readonly DEB_DIR="${LOCAL_DEPS_ROOT}/debs"
readonly ROOTFS="${LOCAL_DEPS_ROOT}/rootfs"
readonly PACKAGES=(
  gazebo
  ros-humble-gazebo-dev
  ros-humble-gazebo-ros
  ros-humble-gazebo-plugins
  ros-humble-gazebo-ros-pkgs
)

mkdir -p "${DEB_DIR}" "${ROOTFS}"

for package_name in "${PACKAGES[@]}"; do
  candidate_version="$(LC_ALL=C apt-cache policy "${package_name}" | awk '/Candidate:/ {print $2; exit}')"
  if [[ -z "${candidate_version}" || "${candidate_version}" == "(none)" ]]; then
    echo "Error: no apt candidate is available for ${package_name}." >&2
    exit 1
  fi

  deb_path="${DEB_DIR}/${package_name}_${candidate_version}_amd64.deb"
  if [[ ! -f "${deb_path}" ]]; then
    (
      cd "${DEB_DIR}"
      apt download "${package_name}=${candidate_version}"
    )
  fi

  if [[ ! -f "${deb_path}" ]]; then
    echo "Error: apt downloaded ${package_name}, but ${deb_path} was not found." >&2
    exit 1
  fi
  dpkg-deb -x "${deb_path}" "${ROOTFS}"
done

test -x "${ROOTFS}/usr/bin/gazebo"
test -x "${ROOTFS}/usr/bin/gzserver"
test -f "${ROOTFS}/opt/ros/humble/share/gazebo_ros/package.xml"
test -f "${ROOTFS}/opt/ros/humble/lib/libgazebo_ros_diff_drive.so"

set +u
source /opt/ros/humble/setup.bash
source "${ROOTFS}/opt/ros/humble/local_setup.bash"
set -u

if ldd "${ROOTFS}/usr/bin/gzserver" | grep -q 'not found'; then
  echo "Error: the local Gazebo executable still has unresolved libraries:" >&2
  ldd "${ROOTFS}/usr/bin/gzserver" | grep 'not found' >&2
  exit 1
fi
if ldd "${ROOTFS}/opt/ros/humble/lib/libgazebo_ros_diff_drive.so" | grep -q 'not found'; then
  echo "Error: gazebo_ros plugins still have unresolved libraries:" >&2
  ldd "${ROOTFS}/opt/ros/humble/lib/libgazebo_ros_diff_drive.so" | grep 'not found' >&2
  exit 1
fi

echo "Gazebo 11 and gazebo_ros are installed locally under ${ROOTFS}."
