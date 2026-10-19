#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
# Run on Ubuntu 22.04 amd64 with the official ROS 2 Humble apt repository.
if [[ "$(dpkg --print-architecture)" != amd64 ]]; then
  echo 'This public release supports Ubuntu 22.04 amd64.' >&2; exit 2
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'Use Ubuntu 22.04 or the pinned ROS container in CI.' >&2; exit 2
fi
apt-get update
apt-get install -y --no-install-recommends \
  bubblewrap python3-venv ca-certificates \
  ros-humble-nav2-bringup=1.1.20-1jammy.20260916.005516 \
  ros-humble-nav2-smac-planner=1.1.20-1jammy.20260915.232226 \
  ros-humble-nav2-costmap-2d=1.1.20-1jammy.20260915.225814
