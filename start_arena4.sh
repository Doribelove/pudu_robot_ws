#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"

readonly RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}"
readonly PID_FILE="${RUNTIME_DIR}/arena.pid"
readonly LOG_FILE="${RUNTIME_DIR}/arena.log"
readonly DOMAIN_FILE="${RUNTIME_DIR}/ros_domain_id"
readonly PARTITION_FILE="${RUNTIME_DIR}/gazebo_partition"

session_pids() {
  local session_id="$1"
  ps -s "${session_id}" -o pid= 2>/dev/null | tr -d ' '
}

session_is_alive() {
  [[ -n "$(session_pids "$1")" ]]
}

session_commands() {
  ps -s "$1" -o args= 2>/dev/null || true
}

signal_session_members() {
  local signal_name="$1"
  local session_id="$2"
  local member_pid

  while read -r member_pid; do
    [[ -n "${member_pid}" ]] && kill "-${signal_name}" "${member_pid}" 2>/dev/null || true
  done < <(session_pids "${session_id}")
}

stale_arena_sessions() {
  local task_generator_executable="${ARENA4_WS}/install/task_generator/lib/task_generator/task_generator_node"
  local session_id

  while read -r session_id; do
    [[ "${session_id}" =~ ^[0-9]+$ ]] || continue
    if ! grep -Eq '[r]os2 launch arena_bringup arena.launch.py|gz sim ' <<< "$(session_commands "${session_id}")"; then
      echo "${session_id}"
    fi
  done < <(
    ps -eo sid=,comm=,args= | awk -v target="${task_generator_executable}" \
      '($2 ~ /python|task_generator/) && index($0, target) {print $1}' | sort -nu
  )
}

terminate_session() {
  local session_id="$1"

  signal_session_members TERM "${session_id}"
  for _ in {1..50}; do
    session_is_alive "${session_id}" || return 0
    sleep 0.1
  done
  signal_session_members KILL "${session_id}"
}

nav2_is_active() {
  local robot_name="$1"
  local ros_domain_id="$2"

  timeout 15s bash -c '
    set -e
    arena_ws="$1"
    ros_domain_id="$2"
    robot_name="$3"
    unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
    cd "${arena_ws}"
    set +u
    source ./arena.bash >/dev/null
    set -u
    export ROS_DOMAIN_ID="${ros_domain_id}"

    for node in controller_server planner_server bt_navigator; do
      state="$(ros2 lifecycle get --no-daemon --spin-time 2 \
        "/task_generator_node/${robot_name}/${node}" 2>/dev/null)"
      [[ "${state}" == *"active [3]"* ]] || exit 1
    done
  ' _ "${ARENA4_WS}" "${ros_domain_id}" "${robot_name}"
}

localization_is_ready() {
  local robot_name="$1"
  local ros_domain_id="$2"

  timeout 15s bash -c '
    set -e
    arena_ws="$1"
    ros_domain_id="$2"
    robot_name="$3"
    unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
    cd "${arena_ws}"
    set +u
    source ./arena.bash >/dev/null
    set -u
    export ROS_DOMAIN_ID="${ros_domain_id}"
    ros2 topic echo \
      --no-daemon --spin-time 2 \
      "/task_generator_node/${robot_name}/amcl_pose" \
      geometry_msgs/msg/PoseWithCovarianceStamped --once >/dev/null

    timeout 5s ros2 run tf2_ros tf2_echo \
      map "${robot_name}/base_link" 2>/dev/null | grep -q "Translation:"
  ' _ "${ARENA4_WS}" "${ros_domain_id}" "${robot_name}"
}

cleanup_stale_arena_sessions() {
  local stale_session
  while read -r stale_session; do
    [[ -n "${stale_session}" ]] || continue
    echo "清理失去 launch/Gazebo 父进程的 Arena4 残留会话：${stale_session}"
    terminate_session "${stale_session}"
  done < <(stale_arena_sessions)
}

show_status() {
  local managed_pid=""
  if [[ -f "${PID_FILE}" ]]; then
    read -r managed_pid < "${PID_FILE}" || true
  fi

  if [[ "${managed_pid}" =~ ^[0-9]+$ ]] && session_is_alive "${managed_pid}"; then
    if grep -Eq '[r]os2 launch arena_bringup arena.launch.py|gz sim ' <<< "$(session_commands "${managed_pid}")"; then
      echo "Arena4 由 PUDU 统一入口运行中（会话：${managed_pid}）。"
      if [[ -r "${DOMAIN_FILE}" ]]; then
        echo "ROS Domain：$(<"${DOMAIN_FILE}")"
      fi
      echo "日志：${LOG_FILE}"
      return 0
    fi
    echo "Arena4 主进程已退出，但会话 ${managed_pid} 仍有残留；下次启动会自动清理。"
    return 0
  fi

  mapfile -t existing_launches < <(pgrep -f '[r]os2 launch arena_bringup arena.launch.py' 2>/dev/null || true)
  if (( ${#existing_launches[@]} > 0 )); then
    echo "Arena4 正在运行，但不是由本脚本启动："
    ps -o pid=,ppid=,sid=,stat=,etime=,args= -p "$(IFS=,; echo "${existing_launches[*]}")"
    return 0
  fi

  echo "Arena4 未运行。"
}

check_environment() {
  local package_count

  [[ -d "${ARENA4_WS}/src" ]] || {
    echo "错误：缺少 Arena4 源码目录：${ARENA4_WS}/src" >&2
    return 1
  }
  [[ -f "${ARENA4_WS}/arena.bash" ]] || {
    echo "错误：缺少 Arena4 环境脚本：${ARENA4_WS}/arena.bash" >&2
    return 1
  }
  [[ -f "${ARENA4_WS}/install/local_setup.bash" ]] || {
    echo "错误：Arena4 尚未构建：${ARENA4_WS}/install/local_setup.bash 不存在" >&2
    return 1
  }

  (
    unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
    cd "${ARENA4_WS}"
    set +u
    source ./arena.bash >/dev/null
    set -u
    command -v gz >/dev/null
    ros2 pkg prefix arena_bringup >/dev/null
    ros2 pkg prefix task_generator >/dev/null
  )

  package_count="$(find "${ARENA4_WS}/src" -name package.xml -type f | wc -l)"
  echo "Arena4 环境检查通过。"
  echo "工作区：${ARENA4_WS}"
  echo "源码包：${package_count}"
  echo "隔离环境：运行时独立 ROS Domain，RMW=Fast DDS，Gazebo Harmonic"
  show_status
}

show_help=false
check_only=false
status_only=false
launch_args=()
for arg in "$@"; do
  case "${arg}" in
    -h|--help)
      show_help=true
      ;;
    --check)
      check_only=true
      ;;
    --status)
      status_only=true
      ;;
    --headless)
      launch_args+=("headless:=2")
      ;;
    --scenario)
      launch_args+=("tm_robots:=scenario" "tm_obstacles:=scenario")
      ;;
    *)
      launch_args+=("${arg}")
      ;;
  esac
done

if [[ "${show_help}" == true ]]; then
  cat <<'EOF'
用法：./start_arena4.sh [--check|--status] [--headless] [--scenario] [Arena launch 参数]

  --check       检查 Arena4 源码、install、Gazebo 和 ROS 包
  --status      查看运行状态
  --headless    不启动 Gazebo GUI 和 RViz（等价于 headless:=2）
  默认界面     只启动 RViz，Gazebo 仍在后台运行（等价于 headless:=1）
  --scenario    使用固定机器人/障碍场景，便于复现实验

示例：
  ./start_arena4.sh                 # 默认 Jackal + TEB，只显示 RViz
  ./start_arena4.sh --scenario
  ./start_arena4.sh --headless --scenario
  ./start_arena4.sh robot:=turtlebot world:=hospital \
    local_planner:=dwb global_planner:=navfn tm_robots:=explore
  ./start_arena4.sh --scenario robot:=turtlebot world:=hospital \
    local_planner:=mppi global_planner:=smac_2d

常用可选项：
  world:=factory|hospital|ignc|map_empty|house17|generated
  robot:=jackal|turtlebot|boxer|dingo|husky|ridgeback|...
  local_planner:=dwb|mppi|teb|regulated_pure_pursuit|rotation_shim|...
  global_planner:=navfn|smac_2d|smac_hybrid|smac_state_lattice|theta_star
  tm_robots:=guided|explore|random|scenario
  tm_obstacles:=parametrized|random|scenario|environment

默认组合：robot:=jackal local_planner:=teb global_planner:=navfn

说明：factory/hospital/ignc 未指定 tm_obstacles 时，默认使用各自
      scenarios/default.json；显式传入 tm_obstacles:=random 可覆盖。

查看 Arena 原生完整参数：
  source ./setup_arena4_runtime.bash
  ros2 launch arena_bringup arena.launch.py --show-args

终止由本入口启动的实例：./stop_arena4.sh
检查当前运行基线：./verify_arena4_baseline.sh [--robot NAME] [--move]
EOF
  exit 0
fi

if [[ "${check_only}" == true ]]; then
  check_environment
  exit 0
fi

if [[ "${status_only}" == true ]]; then
  show_status
  exit 0
fi

check_environment >/dev/null
mkdir -p "${RUNTIME_DIR}"
chmod 700 "${RUNTIME_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  read -r existing_pid < "${PID_FILE}" || true
  if [[ "${existing_pid:-}" =~ ^[0-9]+$ ]] && session_is_alive "${existing_pid}"; then
    if grep -Eq '[r]os2 launch arena_bringup arena.launch.py|gz sim ' <<< "$(session_commands "${existing_pid}")"; then
      echo "Arena4 已由统一入口运行（会话：${existing_pid}）。"
      echo "日志：${LOG_FILE}"
      exit 0
    fi
    if grep -q 'task_generator_node' <<< "$(session_commands "${existing_pid}")"; then
      echo "检测到上次 Arena4 的孤立 task_generator，会先清理再启动。"
      terminate_session "${existing_pid}"
    else
      echo "错误：PID 文件中的会话 ${existing_pid} 已不属于 Arena4，拒绝覆盖。" >&2
      exit 1
    fi
  fi
  rm -f "${PID_FILE}"
fi

cleanup_stale_arena_sessions

mapfile -t unmanaged_launches < <(pgrep -f '[r]os2 launch arena_bringup arena.launch.py' 2>/dev/null || true)
if (( ${#unmanaged_launches[@]} > 0 )); then
  echo "Arena4 已在其他终端运行，本次不重复启动："
  ps -o pid=,ppid=,sid=,stat=,etime=,args= -p "$(IFS=,; echo "${unmanaged_launches[*]}")"
  echo "请在原终端按 Ctrl+C 停止；当前实例不会被 ./stop_arena4.sh 误杀。"
  exit 0
fi

has_sim_argument=false
has_headless_argument=false
has_robot_argument=false
has_local_planner_argument=false
camera_target="jackal"
gazebo_gui=true
guided_mode=false
for arg in "${launch_args[@]}"; do
  [[ "${arg}" == sim:=* ]] && has_sim_argument=true
  [[ "${arg}" == headless:=* ]] && has_headless_argument=true
  [[ "${arg}" == robot:=* ]] && has_robot_argument=true
  [[ "${arg}" == local_planner:=* ]] && has_local_planner_argument=true
  case "${arg}" in
    robot:=*)
      camera_target="${arg#robot:=}"
      ;;
    headless:=1|headless:=2)
      gazebo_gui=false
      ;;
    tm_robots:=guided)
      guided_mode=true
      ;;
  esac
done
if [[ "${has_robot_argument}" == false ]]; then
  launch_args=("robot:=jackal" "${launch_args[@]}")
fi
if [[ "${has_local_planner_argument}" == false ]]; then
  launch_args=("local_planner:=teb" "${launch_args[@]}")
fi
if [[ "${has_sim_argument}" == false ]]; then
  launch_args=("sim:=gazebo" "${launch_args[@]}")
fi

# Keep Gazebo running as the simulator, but show only RViz by default.  This
# avoids the heavy Gazebo GUI while preserving the RViz goal and costmap view.
# Explicit headless:=0/1/2 remains authoritative for callers that need another mode.
if [[ "${has_headless_argument}" == false ]]; then
  launch_args=("headless:=1" "${launch_args[@]}")
fi

# Reusing a fixed Fast DDS domain immediately after an unclean shutdown can
# expose stale lifecycle/service endpoints with the same Arena node names.
# Each managed run gets a separate domain; setup_arena4_runtime.bash reads the
# file below so ROS CLI commands automatically join the running instance.
arena_domain_id="${ARENA4_ROS_DOMAIN_ID:-$((20 + ($$ % 200)))}"
if [[ ! "${arena_domain_id}" =~ ^[0-9]+$ ]] || (( arena_domain_id > 232 )); then
  echo "错误：ARENA4_ROS_DOMAIN_ID 必须是 0 到 232 的整数。" >&2
  exit 1
fi
echo "${arena_domain_id}" > "${DOMAIN_FILE}"

nohup setsid bash -c '
  set -e
  arena_ws="$1"
  ros_domain_id="$2"
  shift 2
  unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
  cd "${arena_ws}"
  set +u
  source ./arena.bash
  # nav2_teb_controller is linked against the workspace-local g2o build.
  # The launch child sources arena.bash directly (rather than the outer
  # setup_arena4_runtime.bash), so propagate the runtime library prefix here.
  if [[ -d "${ARENA4_G2O_PREFIX:-}" ]]; then
    export CMAKE_PREFIX_PATH="${ARENA4_G2O_PREFIX}:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${ARENA4_G2O_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  fi
  export ROS_DOMAIN_ID="${ros_domain_id}"
  exec ros2 launch arena_bringup arena.launch.py "$@"
' _ "${ARENA4_WS}" "${arena_domain_id}" "${launch_args[@]}" > "${LOG_FILE}" 2>&1 &
arena_pid=$!
echo "${arena_pid}" > "${PID_FILE}"
echo "arena4_${UID}_${arena_pid}" > "${PARTITION_FILE}"

echo "Arena4 正在启动（会话：${arena_pid}，最长约 90 秒）……"
echo "启动日志：${LOG_FILE}"

arena_ready=false
model_ready=false
nav2_ready=false
localization_ready=false
for startup_poll in {1..180}; do
  if ! session_is_alive "${arena_pid}"; then
    break
  fi
  if grep -Eq 'Caught exception|PackageNotFoundError|No executable found|Traceback \(most recent call last\)|Failed to load a world|The supplied world name' "${LOG_FILE}"; then
    break
  fi
  if grep -q 'Task Reset!' "${LOG_FILE}" && \
      grep -q 'gz sim' < <(session_commands "${arena_pid}"); then
    gazebo_partition="$(<"${PARTITION_FILE}")"
    if GZ_PARTITION="${gazebo_partition}" timeout 10s gz model \
        -m "${camera_target}" -p 2>/dev/null | \
        grep -q -- "- Name: ${camera_target}"; then
      model_ready=true
    fi
    if [[ "${model_ready}" == true ]] && \
       nav2_is_active "${camera_target}" "${arena_domain_id}"; then
      nav2_ready=true
      if localization_is_ready "${camera_target}" "${arena_domain_id}"; then
        localization_ready=true
        arena_ready=true
        break
      fi
    fi
  fi
  if (( startup_poll % 20 == 0 )); then
    echo "仍在启动：Gazebo 模型=${model_ready}，Nav2=${nav2_ready}，定位/TF=${localization_ready}"
  fi
  sleep 0.5
done

if [[ "${arena_ready}" != true ]]; then
  if session_is_alive "${arena_pid}"; then
    terminate_session "${arena_pid}"
  fi
  rm -f "${PID_FILE}" "${DOMAIN_FILE}" "${PARTITION_FILE}"
  echo "Arena4 启动失败，最近日志如下：" >&2
  [[ "${model_ready}" == true ]] || \
    echo "诊断：Gazebo 中未发现机器人模型 ${camera_target}。" >&2
  [[ "${nav2_ready}" == true ]] || \
    echo "诊断：${camera_target} 的 Nav2 核心节点未全部进入 active 状态。" >&2
  [[ "${localization_ready}" == true ]] || \
    echo "诊断：${camera_target} 的 AMCL 初始位姿或 map -> base_link TF 尚未就绪。" >&2
  tail -n 80 "${LOG_FILE}" >&2 || true
  exit 1
fi

# Keep the Gazebo camera attached to the robot. This makes the model visible
# immediately in large worlds and keeps it in view after navigation starts.
camera_focused=false
camera_following=false
if [[ "${gazebo_gui}" == true ]]; then
  gazebo_partition="$(<"${PARTITION_FILE}")"
  follow_reply="$(
    GZ_PARTITION="${gazebo_partition}" gz service \
      -s /gui/follow \
      --reqtype gz.msgs.StringMsg \
      --reptype gz.msgs.Boolean \
      --timeout 1000 \
      --req "data: \"${camera_target}\"" \
      2>/dev/null || true
  )"
  if grep -q 'data: true' <<< "${follow_reply}"; then
    offset_reply="$(
      GZ_PARTITION="${gazebo_partition}" gz service \
        -s /gui/follow/offset \
        --reqtype gz.msgs.Vector3d \
        --reptype gz.msgs.Boolean \
        --timeout 1000 \
        --req 'x: -6, y: 0, z: 3' \
        2>/dev/null || true
    )"
    if grep -q 'data: true' <<< "${offset_reply}"; then
      camera_focused=true
      camera_following=true
    fi
  fi

  # Older GUI versions may not provide follow services. Fall back to moving
  # the camera to the current robot position in that case.
  for _ in {1..20}; do
    [[ "${camera_focused}" == true ]] && break
    model_state="$(
      GZ_PARTITION="${gazebo_partition}" gz model \
        -m "${camera_target}" -p 2>/dev/null || true
    )"
    robot_xy="$(
      awk '
        /Pose \[ XYZ/ {
          getline
          gsub(/\[/, "")
          gsub(/\]/, "")
          print $1, $2
          exit
        }
      ' <<< "${model_state}"
    )"
    read -r robot_x robot_y <<< "${robot_xy}"
    if [[ ! "${robot_x:-}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
       [[ ! "${robot_y:-}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
      sleep 0.25
      continue
    fi

    # Put the camera six metres behind and above the robot.  A fixed camera
    # orientation then looks along +X toward the model while preserving its
    # actual world Y coordinate.
    camera_x="$(awk -v x="${robot_x}" 'BEGIN {printf "%.6f", x - 6.0}')"
    camera_reply="$(
      GZ_PARTITION="${gazebo_partition}" gz service \
        -s /gui/move_to/pose \
        --reqtype gz.msgs.GUICamera \
        --reptype gz.msgs.Boolean \
        --timeout 500 \
        --req "pose: {position: {x: ${camera_x}, y: ${robot_y}, z: 6}, orientation: {y: 0.24740396, w: 0.96891242}}" \
        2>/dev/null || true
    )"
    if grep -q 'data: true' <<< "${camera_reply}"; then
      camera_focused=true
      break
    fi
    sleep 0.25
  done
fi

echo "Arena4 已启动（会话：${arena_pid}）。"
echo "环境：ROS Domain ${arena_domain_id} / Gazebo Harmonic；不会改变 PUDU 普通模式。"
if [[ "${camera_focused}" == true ]]; then
  if [[ "${camera_following}" == true ]]; then
    echo "Gazebo 相机正在跟随机器人：${camera_target}"
  else
    echo "Gazebo 相机已定位到机器人：${camera_target}"
  fi
fi
if [[ "${guided_mode}" == true ]]; then
  echo "guided 模式已就绪：车辆会等待 RViz/Nav2 目标，不会在启动后自动运动。"
fi
echo "日志：${LOG_FILE}"
echo "停止：./stop_arena4.sh"
