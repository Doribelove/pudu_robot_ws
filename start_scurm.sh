#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"
readonly SCURM_WS="${SCURM_REFERENCE_WS}"

mode="check"
start_livox="true"
extra_args=()

for arg in "$@"; do
  case "${arg}" in
    -h|--help)
      cat <<'EOF'
用法：./start_scurm.sh [--check|--mapping|--localization] [launch 参数]

  --check         检查 SCURM underlay、核心节点和 PUDU 适配插件（默认）
  --mapping       启动 Livox MID360 + FAST-LIO2 建图
  --localization  启动 Livox + ICP 初始配准 + FAST-LIO2 先验地图定位
  --no-livox      不启动 Livox 驱动，便于回放 rosbag 或接已有驱动

示例：
  ./start_scurm.sh --mapping
  ./start_scurm.sh --mapping --no-livox
  ./start_scurm.sh --localization map_path:=/absolute/path/map.pcd
EOF
      exit 0
      ;;
    --check) mode="check" ;;
    --mapping) mode="mapping" ;;
    --localization) mode="localization" ;;
    --no-livox) start_livox="false" ;;
    *) extra_args+=("${arg}") ;;
  esac
done

set +u
source "${PUDU_WS}/setup_scurm_runtime.bash"
set -u

if [[ "${mode}" == "check" ]]; then
  required_packages=(
    fast_lio icp_relocalization livox_ros_driver2 behavior_ext_plugins
    costmap_intensity pudu_nav2_plugins
  )
  for package_name in "${required_packages[@]}"; do
    ros2 pkg prefix "${package_name}" >/dev/null
    echo "OK package: ${package_name}"
  done
  ros2 pkg executables fast_lio | grep -q 'fastlio_mapping'
  ros2 pkg executables icp_relocalization | grep -q 'icp_node'
  test -f "${SCURM_WS}/install/sentry_bringup/share/sentry_bringup/maps/GlobalMap.pcd"
  echo "OK SCURM source/build: ${SCURM_WS}"
  echo "OK PUDU recovery plugin: pudu_nav2_plugins/BackUpToFreeSpace"
  exit 0
fi

params_name="fast_lio_mapping_param.yaml"
if [[ "${mode}" == "localization" ]]; then
  params_name="fast_lio_relocalization_param.yaml"
fi
readonly PARAMS_FILE="${SCURM_WS}/install/sentry_bringup/share/sentry_bringup/params/${params_name}"

exec ros2 launch pudu_nav2_bringup scurm_lio.launch.py \
  mode:="${mode}" \
  start_livox:="${start_livox}" \
  params_file:="${PARAMS_FILE}" \
  "${extra_args[@]}"
