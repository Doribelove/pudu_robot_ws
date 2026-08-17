#!/usr/bin/env bash

set -euo pipefail

readonly LINOROBOT_WS="/home/robot/linorobot_sim_ws"
readonly LINOROBOT_SETUP="${LINOROBOT_WS}/setup_sim.bash"
readonly EXPLORATION_SETUP="/home/robot/exploration_reference_ws/install/local_setup.bash"
readonly PUDU_SETUP="/home/robot/pudu_robot_ws/install/local_setup.bash"
readonly RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/linorobot2-${UID}"
readonly GAZEBO_PID_FILE="${RUNTIME_DIR}/simulation.pid"
readonly NAV_PID_FILE="${RUNTIME_DIR}/navigation.pid"
readonly EXPLORE_PID_FILE="${RUNTIME_DIR}/exploration.pid"
readonly GAZEBO_LOG_FILE="${RUNTIME_DIR}/gazebo.log"
readonly NAV_LOG_FILE="${RUNTIME_DIR}/navigation.log"
readonly EXPLORE_LOG_FILE="${RUNTIME_DIR}/exploration.log"

session_pids() {
  local session_id="$1"
  ps -s "${session_id}" -o pid= 2>/dev/null | tr -d ' '
}

session_is_alive() {
  [[ -n "$(session_pids "$1")" ]]
}

stop_session() {
  local session_id="$1"
  local member_pid

  if kill -0 "${session_id}" 2>/dev/null; then
    kill -INT "${session_id}" 2>/dev/null || true
  fi

  for _ in {1..20}; do
    session_is_alive "${session_id}" || return 0
    sleep 0.2
  done

  while read -r member_pid; do
    [[ -n "${member_pid}" ]] && kill -TERM "${member_pid}" 2>/dev/null || true
  done < <(session_pids "${session_id}")

  for _ in {1..25}; do
    session_is_alive "${session_id}" || return 0
    sleep 0.2
  done

  while read -r member_pid; do
    [[ -n "${member_pid}" ]] && kill -KILL "${member_pid}" 2>/dev/null || true
  done < <(session_pids "${session_id}")
}

mode="navigation"
explore_enabled=false
show_help=false
gazebo_args=()
for arg in "$@"; do
  case "${arg}" in
    -h|--help)
      show_help=true
      ;;
    --slam)
      mode="slam"
      ;;
    --navigation)
      mode="navigation"
      ;;
    --explore)
      explore_enabled=true
      ;;
    *)
      if [[ "${arg}" == world:=* ]]; then
        world_path="${arg#world:=}"
        if [[ ! -f "${world_path}" ]]; then
          echo "错误：Gazebo world 文件不存在或不是普通文件：${world_path}" >&2
          exit 1
        fi
      fi
      gazebo_args+=("${arg}")
      ;;
  esac
done

if [[ "${show_help}" == true ]]; then
  cat <<'EOF'
用法：./start_linorobot.sh [--navigation|--slam|--explore] [Gazebo launch 参数]

  --navigation  使用已有地图启动 Nav2（默认）
  --slam        启动 SLAM Toolbox 在线建图
  --explore     启动 SLAM、Nav2 和前沿自动探索建图

示例：
  ./start_linorobot.sh --explore
  ./start_linorobot.sh --explore world:=/absolute/path/to/scene.world

终止：./stop_linorobot.sh
EOF
  exit 0
fi

if [[ "${explore_enabled}" == true ]]; then
  mode="slam"
fi

# Keep this simulation isolated from unrelated ROS experiments and stale DDS
# discovery entries in the default domain. Override when integration requires it.
export ROS_DOMAIN_ID="${LINOROBOT_ROS_DOMAIN_ID:-42}"

if [[ ! -f "${LINOROBOT_SETUP}" ]]; then
  echo "错误：找不到 Linorobot2 环境脚本：${LINOROBOT_SETUP}" >&2
  exit 1
fi

mkdir -p "${RUNTIME_DIR}"
chmod 700 "${RUNTIME_DIR}"

for pid_file in "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}" "${EXPLORE_PID_FILE}"; do
  if [[ ! -f "${pid_file}" ]]; then
    continue
  fi

  read -r existing_pid < "${pid_file}" || true
  if [[ "${existing_pid:-}" =~ ^[0-9]+$ ]] && session_is_alive "${existing_pid}"; then
    echo "Linorobot2 仿真已经在运行（会话：${existing_pid}）。"
    echo "如需切换到地图/SLAM 界面，请先运行 ./stop_linorobot.sh，再重新启动。"
    exit 0
  fi
  rm -f "${pid_file}"
done

# 同一 Gazebo Master 不能同时绑定默认端口。提前报错，避免只启动 RViz。
mapfile -t existing_gzservers < <(pgrep -x gzserver 2>/dev/null || true)
if (( ${#existing_gzservers[@]} > 0 )); then
  echo "错误：检测到已有 gzserver，Gazebo 默认端口会冲突：" >&2
  ps -o pid=,ppid=,sid=,stat=,args= -p "$(IFS=,; echo "${existing_gzservers[*]}")" >&2 || true
  echo "请先关闭旧 Gazebo，再重新运行本命令。" >&2
  exit 1
fi

mapfile -t unmanaged_launches < <(pgrep -f '[r]os2 launch linorobot2_gazebo gazebo.launch.py' 2>/dev/null || true)
if (( ${#unmanaged_launches[@]} > 0 )); then
  echo "错误：检测到未被当前脚本记录的 Linorobot2 Gazebo launch：" >&2
  ps -o pid=,ppid=,sid=,stat=,args= -p "$(IFS=,; echo "${unmanaged_launches[*]}")" >&2 || true
  echo "请先在其终端按 Ctrl+C 关闭，再重新运行本命令。" >&2
  exit 1
fi

# setup_sim.bash 会加载 ROS 2 Humble、Nav2、Linorobot2，并默认使用 2wd 底盘。
# ROS 2 Humble 的环境脚本不完全兼容 nounset，因此加载期间临时关闭。
set +u
source "${LINOROBOT_SETUP}"
if [[ "${explore_enabled}" == true ]]; then
  if [[ ! -f "${EXPLORATION_SETUP}" ]]; then
    echo "错误：未构建探索算法 underlay：${EXPLORATION_SETUP}" >&2
    exit 1
  fi
  if [[ ! -f "${PUDU_SETUP}" ]]; then
    echo "错误：未构建 PUDU 工作区：${PUDU_SETUP}" >&2
    exit 1
  fi
  source "${EXPLORATION_SETUP}"
  source "${PUDU_SETUP}"
fi
set -u

# Gazebo 和 Nav2/SLAM 各自成为独立进程组，停止脚本可完整清理其子进程。
nohup setsid ros2 launch linorobot2_gazebo gazebo.launch.py \
  "${gazebo_args[@]}" > "${GAZEBO_LOG_FILE}" 2>&1 &
gazebo_pid=$!
echo "${gazebo_pid}" > "${GAZEBO_PID_FILE}"

gazebo_ready=false
for _ in {1..60}; do
  if ! session_is_alive "${gazebo_pid}"; then
    break
  fi
  if grep -Eq "Unable to start server|process has died.*gazebo|EXCEPTION:" "${GAZEBO_LOG_FILE}"; then
    break
  fi
  if grep -Eq "SpawnEntity: Successfully spawned entity" "${GAZEBO_LOG_FILE}"; then
    gazebo_ready=true
    break
  fi
  sleep 0.5
done

if [[ "${gazebo_ready}" != true ]]; then
  stop_session "${gazebo_pid}"
  rm -f "${GAZEBO_PID_FILE}"
  echo "Gazebo 启动失败，最近的日志如下：" >&2
  tail -n 50 "${GAZEBO_LOG_FILE}" >&2 || true
  exit 1
fi

nohup setsid ros2 launch linorobot2_navigation "${mode}.launch.py" \
  sim:=true rviz:=true > "${NAV_LOG_FILE}" 2>&1 &
nav_pid=$!
echo "${nav_pid}" > "${NAV_PID_FILE}"

nav_ready=false
for _ in {1..30}; do
  if ! session_is_alive "${nav_pid}"; then
    break
  fi
  if grep -Eq "\[rviz2-[0-9]+\]: process started" "${NAV_LOG_FILE}"; then
    nav_ready=true
    break
  fi
  sleep 0.5
done

if [[ "${nav_ready}" != true ]]; then
  stop_session "${nav_pid}"
  stop_session "${gazebo_pid}"
  rm -f "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}"
  echo "Nav2/RViz 启动失败，最近的日志如下：" >&2
  tail -n 40 "${NAV_LOG_FILE}" >&2 || true
  exit 1
fi

if [[ "${explore_enabled}" == true ]]; then
  # explore_lite 启动后会立即规划，因此先等待 Nav2 生命周期节点激活，
  # 并确认 SLAM 已生成第一张非空地图。
  mapping_ready=false
  for _ in {1..60}; do
    if ! session_is_alive "${nav_pid}"; then
      break
    fi
    if grep -Eq "Managed nodes are active" "${NAV_LOG_FILE}" && \
       grep -Eq "StaticLayer: Resizing costmap to" "${NAV_LOG_FILE}"; then
      mapping_ready=true
      break
    fi
    sleep 0.5
  done

  if [[ "${mapping_ready}" != true ]]; then
    stop_session "${nav_pid}"
    stop_session "${gazebo_pid}"
    rm -f "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}"
    echo "自动探索启动失败：SLAM/Nav2 未在 30 秒内生成可用地图。" >&2
    tail -n 50 "${NAV_LOG_FILE}" >&2 || true
    exit 1
  fi

  nohup setsid ros2 launch pudu_coverage_planner autonomous_mapping.launch.py \
    use_sim_time:=true > "${EXPLORE_LOG_FILE}" 2>&1 &
  explore_pid=$!
  echo "${explore_pid}" > "${EXPLORE_PID_FILE}"

  explore_ready=false
  for _ in {1..60}; do
    if ! session_is_alive "${explore_pid}"; then
      break
    fi
    if grep -Eq "Connected to move_base nav2 server" "${EXPLORE_LOG_FILE}"; then
      explore_ready=true
      break
    fi
    sleep 0.5
  done

  if [[ "${explore_ready}" != true ]]; then
    stop_session "${explore_pid}"
    stop_session "${nav_pid}"
    stop_session "${gazebo_pid}"
    rm -f "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}" "${EXPLORE_PID_FILE}"
    echo "自动探索节点启动失败，最近的日志如下：" >&2
    tail -n 60 "${EXPLORE_LOG_FILE}" >&2 || true
    exit 1
  fi
fi

if [[ "${explore_enabled}" == true ]]; then
  mode_text="前沿覆盖自动建图"
elif [[ "${mode}" == "slam" ]]; then
  mode_text="SLAM 在线建图"
else
  mode_text="Nav2 地图导航"
fi

echo "Linorobot2 Gazebo + RViz 已启动（${mode_text}，底盘：${LINOROBOT2_BASE}）。"
echo "ROS Domain：${ROS_DOMAIN_ID}（其他终端请 source ./setup_linorobot_runtime.bash）"
echo "Gazebo 日志：${GAZEBO_LOG_FILE}"
echo "导航日志：${NAV_LOG_FILE}"
if [[ "${explore_enabled}" == true ]]; then
  echo "探索日志：${EXPLORE_LOG_FILE}"
fi
echo "终止：$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/stop_linorobot.sh"
