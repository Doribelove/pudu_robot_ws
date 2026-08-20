#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}"
readonly PID_FILE="${RUNTIME_DIR}/arena.pid"

robot_name="jackal"
move_test=false

show_help() {
  cat <<'EOF'
用法：./verify_arena4_baseline.sh [--robot NAME] [--move]

  --robot NAME  要检查的 Gazebo/ROS 机器人名称（默认：jackal）
  --move        在 guided 且无活动目标时经速度平滑器直行约 2 秒，并验证 Gazebo 位移

默认检查只读：Gazebo 模型、Nav2 生命周期、定位 TF、代价地图坐标系、
里程计和速度桥接。
EOF
}

while (( $# > 0 )); do
  case "$1" in
    -h|--help)
      show_help
      exit 0
      ;;
    --robot)
      [[ $# -ge 2 ]] || {
        echo "错误：--robot 缺少名称。" >&2
        exit 2
      }
      robot_name="$2"
      shift 2
      ;;
    robot:=*)
      robot_name="${1#robot:=}"
      shift
      ;;
    --move)
      move_test=true
      shift
      ;;
    *)
      echo "错误：未知参数：$1" >&2
      show_help >&2
      exit 2
      ;;
  esac
done

[[ "${robot_name}" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "错误：机器人名称包含非法字符：${robot_name}" >&2
  exit 2
}

if [[ ! -r "${PID_FILE}" ]]; then
  echo "错误：Arena4 未由 ./start_arena4.sh 启动。" >&2
  exit 1
fi

read -r arena_session < "${PID_FILE}" || true
if [[ ! "${arena_session:-}" =~ ^[0-9]+$ ]] || \
   [[ -z "$(ps -s "${arena_session}" -o pid= 2>/dev/null)" ]]; then
  echo "错误：Arena4 会话不存在，请重新运行 ./start_arena4.sh。" >&2
  exit 1
fi

set +u
source "${PUDU_WS}/setup_arena4_runtime.bash" >/dev/null
set -u

model_pose() {
  local model_state
  model_state="$(timeout 10s gz model -m "${robot_name}" -p 2>/dev/null)" || return 1
  awk '
    /Pose \[ XYZ/ {
      getline
      gsub(/\[/, "")
      gsub(/\]/, "")
      x=$1
      y=$2
      getline
      gsub(/\[/, "")
      gsub(/\]/, "")
      print x, y, $3
      exit
    }
  ' <<< "${model_state}"
}

require_topic_endpoint() {
  local topic="$1"
  local endpoint_label="$2"
  local endpoint_count
  local topic_info

  for _ in {1..5}; do
    topic_info="$(timeout 6s ros2 topic info "${topic}" 2>/dev/null || true)"
    endpoint_count="$(awk -F': ' -v label="${endpoint_label}" \
      '$1 == label {print $2}' <<< "${topic_info}")"
    if [[ "${endpoint_count}" =~ ^[0-9]+$ ]] && (( endpoint_count > 0 )); then
      return 0
    fi
    sleep 0.5
  done
  echo "错误：${topic} 的 ${endpoint_label} 为 ${endpoint_count:-未知}。" >&2
  return 1
}

read -r before_x before_y before_yaw < <(model_pose) || {
  echo "错误：Gazebo 中找不到模型 ${robot_name}。" >&2
  exit 1
}
echo "[通过] Gazebo 模型 ${robot_name}：x=${before_x}, y=${before_y}"

for node in controller_server planner_server bt_navigator; do
  state=""
  for _ in {1..5}; do
    state="$(timeout 6s ros2 lifecycle get \
      "/task_generator_node/${robot_name}/${node}" 2>/dev/null || true)"
    [[ "${state}" == *"active [3]"* ]] && break
    sleep 0.5
  done
  if [[ "${state}" != *"active [3]"* ]]; then
    echo "错误：${node} 未激活：${state}" >&2
    exit 1
  fi
done
echo "[通过] Nav2：controller/planner/bt_navigator 均为 active"

tf_static_info="$(timeout 8s ros2 topic info /tf_static -v 2>/dev/null || true)"
if [[ -z "${tf_static_info}" ]]; then
  echo "错误：无法读取 /tf_static 发布者信息。" >&2
  exit 1
fi
if grep -q '^Node name: map_to_odomframe_publisher$' <<< "${tf_static_info}"; then
  echo "错误：发现旧版 map_to_odomframe_publisher，TF 发布权可能冲突。" >&2
  exit 1
fi
if ! timeout 8s ros2 node list 2>/dev/null | \
    grep -qx "/task_generator_node/${robot_name}/amcl"; then
  echo "错误：找不到 ${robot_name} 的 AMCL 定位节点。" >&2
  exit 1
fi
if [[ "${robot_name}" == "jackal" ]]; then
  if ! grep -q '^Node name: map_to_odom_truth$' <<< "${tf_static_info}"; then
    echo "错误：找不到 Jackal 的 Gazebo 真值 map -> odom 发布者。" >&2
    exit 1
  fi
  amcl_tf_broadcast="$(timeout 8s ros2 param get \
    "/task_generator_node/${robot_name}/amcl" tf_broadcast 2>/dev/null || true)"
  if [[ "${amcl_tf_broadcast}" != "Boolean value is: False" ]]; then
    echo "错误：Jackal AMCL 仍在广播 TF：${amcl_tf_broadcast:-未知}。" >&2
    exit 1
  fi
fi

global_costmap_frame="$(timeout 10s ros2 topic echo \
  "/task_generator_node/${robot_name}/global_costmap/costmap" \
  nav_msgs/msg/OccupancyGrid --once --field header.frame_id 2>/dev/null | \
  awk 'NF && $0 != "---" {print; exit}' || true)"
local_costmap_frame="$(timeout 10s ros2 topic echo \
  "/task_generator_node/${robot_name}/local_costmap/costmap" \
  nav_msgs/msg/OccupancyGrid --once --field header.frame_id 2>/dev/null | \
  awk 'NF && $0 != "---" {print; exit}' || true)"
if [[ "${global_costmap_frame}" != "map" || \
      "${local_costmap_frame}" != "${robot_name}/odom" ]]; then
  echo "错误：代价地图坐标系异常：global=${global_costmap_frame:-未知}, local=${local_costmap_frame:-未知}。" >&2
  exit 1
fi
if [[ "${robot_name}" == "jackal" ]]; then
  echo "[通过] 定位：Gazebo 真值 TF 生效、AMCL TF 已禁用，global/map 与 local/odom 配置正确"
else
  echo "[通过] 定位：global/map 与 local/odom 配置正确"
fi

amcl_pose="$(timeout 10s ros2 topic echo \
  "/task_generator_node/${robot_name}/amcl_pose" \
  geometry_msgs/msg/PoseWithCovarianceStamped --once --field pose.pose 2>/dev/null || true)"
read -r amcl_x amcl_y amcl_qz amcl_qw < <(
  awk '
    $1 == "x:" && ++x_count == 1 {x=$2}
    $1 == "y:" && ++y_count == 1 {y=$2}
    $1 == "z:" && ++z_count == 2 {qz=$2}
    $1 == "w:" {qw=$2}
    END {if (x != "" && y != "" && qz != "" && qw != "") print x, y, qz, qw}
  ' <<< "${amcl_pose}"
)
[[ -n "${amcl_x:-}" ]] || {
  echo "错误：AMCL 尚未发布有效初始位姿。" >&2
  exit 1
}
read -r localization_error yaw_error < <(
  awk -v gx="${before_x}" -v gy="${before_y}" -v gyaw="${before_yaw}" \
      -v ax="${amcl_x}" -v ay="${amcl_y}" -v qz="${amcl_qz}" -v qw="${amcl_qw}" '
    BEGIN {
      pi=atan2(0,-1)
      ayaw=atan2(2*qw*qz, 1-2*qz*qz)
      dyaw=ayaw-gyaw
      while (dyaw > pi) dyaw-=2*pi
      while (dyaw < -pi) dyaw+=2*pi
      printf "%.4f %.4f\n", sqrt((ax-gx)^2+(ay-gy)^2), sqrt(dyaw*dyaw)
    }
  '
)
echo "[诊断] AMCL/Gazebo 位置误差 ${localization_error} m，航向误差 ${yaw_error} rad（仿真真值 TF 不使用该诊断值）"

map_base_tf="$(timeout 8s ros2 run tf2_ros tf2_echo \
  map "${robot_name}/base_link" 2>/dev/null || true)"
read -r tf_x tf_y tf_yaw < <(
  awk '
    /Translation:/ {
      gsub(/[\[\],]/, "")
      x=$3
      y=$4
    }
    /RPY \(radian\)/ {
      gsub(/[\[\],]/, "")
      print x, y, $8
      exit
    }
  ' <<< "${map_base_tf}"
)
[[ -n "${tf_x:-}" ]] || {
  echo "错误：无法读取 RViz 使用的 map -> ${robot_name}/base_link TF。" >&2
  exit 1
}
read -r tf_position_error tf_yaw_error < <(
  awk -v gx="${before_x}" -v gy="${before_y}" -v gyaw="${before_yaw}" \
      -v tx="${tf_x}" -v ty="${tf_y}" -v tyaw="${tf_yaw}" '
    BEGIN {
      pi=atan2(0,-1)
      dyaw=tyaw-gyaw
      while (dyaw > pi) dyaw-=2*pi
      while (dyaw < -pi) dyaw+=2*pi
      printf "%.4f %.4f\n", sqrt((tx-gx)^2+(ty-gy)^2), sqrt(dyaw*dyaw)
    }
  '
)
if ! awk -v p="${tf_position_error}" -v a="${tf_yaw_error}" \
    'BEGIN {exit !(p <= 0.05 && a <= 0.0873)}'; then
  echo "错误：RViz TF 与 Gazebo 位姿不一致：位置误差 ${tf_position_error} m，航向误差 ${tf_yaw_error} rad。" >&2
  exit 1
fi
echo "[通过] RViz/Gazebo：map -> base_link 位置误差 ${tf_position_error} m，航向误差 ${tf_yaw_error} rad"

require_topic_endpoint "/task_generator_node/${robot_name}/odom" "Publisher count"
require_topic_endpoint "/task_generator_node/${robot_name}/cmd_vel" "Subscription count"
timeout 10s ros2 topic echo "/task_generator_node/${robot_name}/odom" \
  nav_msgs/msg/Odometry --once >/dev/null
echo "[通过] 桥接：里程计有数据，Gazebo 速度订阅存在"

if [[ "${move_test}" == true ]]; then
  echo "[运动测试] 仅应在 guided 且没有活动导航目标时使用。"
  require_topic_endpoint "/task_generator_node/${robot_name}/cmd_vel_nav" "Subscription count"
  timeout 2s ros2 topic pub -r 10 \
    "/task_generator_node/${robot_name}/cmd_vel_nav" \
    geometry_msgs/msg/Twist \
    '{linear: {x: 0.2}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
  timeout 3s ros2 topic pub --once \
    "/task_generator_node/${robot_name}/cmd_vel_nav" \
    geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  sleep 0.5

  read -r after_x after_y _ < <(model_pose) || {
    echo "错误：运动后无法读取 Gazebo 模型位姿。" >&2
    exit 1
  }
  displacement="$(awk \
    -v x0="${before_x}" -v y0="${before_y}" \
    -v x1="${after_x}" -v y1="${after_y}" \
    'BEGIN {dx=x1-x0; dy=y1-y0; printf "%.3f", sqrt(dx*dx + dy*dy)}')"
  awk -v d="${displacement}" 'BEGIN {exit !(d >= 0.05)}' || {
    echo "错误：运动测试位移仅 ${displacement} m。" >&2
    exit 1
  }
  echo "[通过] 运动：Gazebo 位移 ${displacement} m"
fi

echo "Arena4 基线检查通过。"
