#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"

readonly RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/arena4-${UID}"
readonly PID_FILE="${RUNTIME_DIR}/arena.pid"
readonly LOG_FILE="${RUNTIME_DIR}/arena.log"

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

show_status() {
  local managed_pid=""
  if [[ -f "${PID_FILE}" ]]; then
    read -r managed_pid < "${PID_FILE}" || true
  fi

  if [[ "${managed_pid}" =~ ^[0-9]+$ ]] && session_is_alive "${managed_pid}"; then
    echo "Arena4 由 PUDU 统一入口运行中（会话：${managed_pid}）。"
    echo "日志：${LOG_FILE}"
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
  echo "隔离环境：ROS_DOMAIN_ID=1，RMW=Fast DDS，Gazebo Harmonic"
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
  --scenario    使用固定机器人/障碍场景，便于复现实验

示例：
  ./start_arena4.sh
  ./start_arena4.sh --scenario
  ./start_arena4.sh --headless --scenario
  ./start_arena4.sh robot:=turtlebot world:=hospital

查看 Arena 原生完整参数：
  source ./setup_arena4_runtime.bash
  ros2 launch arena_bringup arena.launch.py --show-args

终止由本入口启动的实例：./stop_arena4.sh
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
    if grep -Eq 'arena_bringup|arena.launch.py|task_generator|gz sim' <<< "$(session_commands "${existing_pid}")"; then
      echo "Arena4 已由统一入口运行（会话：${existing_pid}）。"
      echo "日志：${LOG_FILE}"
      exit 0
    fi
    echo "错误：PID 文件中的会话 ${existing_pid} 已不属于 Arena4，拒绝覆盖。" >&2
    exit 1
  fi
  rm -f "${PID_FILE}"
fi

mapfile -t unmanaged_launches < <(pgrep -f '[r]os2 launch arena_bringup arena.launch.py' 2>/dev/null || true)
if (( ${#unmanaged_launches[@]} > 0 )); then
  echo "Arena4 已在其他终端运行，本次不重复启动："
  ps -o pid=,ppid=,sid=,stat=,etime=,args= -p "$(IFS=,; echo "${unmanaged_launches[*]}")"
  echo "请在原终端按 Ctrl+C 停止；当前实例不会被 ./stop_arena4.sh 误杀。"
  exit 0
fi

has_sim_argument=false
for arg in "${launch_args[@]}"; do
  [[ "${arg}" == sim:=* ]] && has_sim_argument=true
done
if [[ "${has_sim_argument}" == false ]]; then
  launch_args=("sim:=gazebo" "${launch_args[@]}")
fi

nohup setsid bash -c '
  set -e
  arena_ws="$1"
  shift
  unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
  cd "${arena_ws}"
  set +u
  source ./arena.bash
  exec ros2 launch arena_bringup arena.launch.py "$@"
' _ "${ARENA4_WS}" "${launch_args[@]}" > "${LOG_FILE}" 2>&1 &
arena_pid=$!
echo "${arena_pid}" > "${PID_FILE}"

arena_ready=false
for _ in {1..90}; do
  if ! session_is_alive "${arena_pid}"; then
    break
  fi
  if grep -Eq 'Caught exception|PackageNotFoundError|No executable found|Traceback \(most recent call last\)' "${LOG_FILE}"; then
    break
  fi
  if grep -q 'Spawning task_generator with namespace:' "${LOG_FILE}" && \
      grep -q 'gz sim' < <(session_commands "${arena_pid}"); then
    arena_ready=true
    break
  fi
  sleep 0.5
done

if [[ "${arena_ready}" != true ]]; then
  if session_is_alive "${arena_pid}"; then
    while read -r member_pid; do
      [[ -n "${member_pid}" ]] && kill -TERM "${member_pid}" 2>/dev/null || true
    done < <(session_pids "${arena_pid}")
  fi
  rm -f "${PID_FILE}"
  echo "Arena4 启动失败，最近日志如下：" >&2
  tail -n 80 "${LOG_FILE}" >&2 || true
  exit 1
fi

echo "Arena4 已启动（会话：${arena_pid}）。"
echo "环境：ROS Domain 1 / Gazebo Harmonic；不会改变 PUDU 普通模式。"
echo "日志：${LOG_FILE}"
echo "停止：./stop_arena4.sh"
