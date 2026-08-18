#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"
readonly LINOROBOT_WS="${LINOROBOT_SIM_WS}"
readonly LINOROBOT_SETUP="${LINOROBOT_WS}/setup_sim.bash"
readonly LOCAL_GAZEBO_ROOT="${NAV2_REFERENCE_WS}/local_deps/rootfs"
readonly EXPLORATION_SETUP="${EXPLORATION_REFERENCE_WS}/install/local_setup.bash"
readonly SCURM_SETUP="${SCURM_REFERENCE_WS}/install/local_setup.bash"
readonly PUDU_SETUP="${PUDU_ROBOT_WS}/install/local_setup.bash"
readonly COVERAGE_RVIZ_CONFIG="${PUDU_ROBOT_WS}/install/pudu_coverage_planner/share/pudu_coverage_planner/rviz/area_coverage.rviz"
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
coverage_enabled=false
scurm_navigation=false
scurm_lio=false
scurm_lio_mapping=false
fast_sim=false
show_help=false
gazebo_args=()
scurm_pipeline_args=()
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
    --coverage)
      coverage_enabled=true
      ;;
    --scurm-nav)
      scurm_navigation=true
      ;;
    --scurm-lio)
      scurm_lio=true
      ;;
    --scurm-lio-map)
      scurm_lio=true
      scurm_lio_mapping=true
      ;;
    --fast-sim)
      fast_sim=true
      ;;
    *)
      if [[ "${arg}" == world:=* ]]; then
        world_path="${arg#world:=}"
        if [[ ! -f "${world_path}" ]]; then
          echo "错误：Gazebo world 文件不存在或不是普通文件：${world_path}" >&2
          exit 1
        fi
      fi
      case "${arg}" in
        map_path:=*|map_output_path:=*|navigation:=*|rviz:=*)
          scurm_pipeline_args+=("${arg}")
          ;;
        spawn_x:=*)
          gazebo_args+=("${arg}")
          scurm_pipeline_args+=("initial_x:=${arg#spawn_x:=}")
          ;;
        spawn_y:=*)
          gazebo_args+=("${arg}")
          scurm_pipeline_args+=("initial_y:=${arg#spawn_y:=}")
          ;;
        spawn_yaw:=*)
          gazebo_args+=("${arg}")
          scurm_pipeline_args+=("initial_yaw:=${arg#spawn_yaw:=}")
          ;;
        *)
          gazebo_args+=("${arg}")
          ;;
      esac
      ;;
  esac
done

if [[ "${show_help}" == true ]]; then
  cat <<'EOF'
用法：./start_linorobot.sh [--navigation|--slam|--explore|--scurm-nav|--scurm-lio|--scurm-lio-map] [--coverage] [--fast-sim] [launch 参数]

  --navigation  使用已有地图启动 Nav2（默认）
  --slam        启动 SLAM Toolbox 在线建图
  --explore     启动 SLAM、Nav2 和前沿自动探索建图
  --scurm-nav   使用可选的 Theta* + Constrained Smoother + MPPI + 自适应脱困
  --scurm-lio   使用 Gazebo 3D 雷达 + ICP + FAST-LIO 定位，并启动无 AMCL 导航
  --scurm-lio-map 使用 Gazebo 3D 雷达运行 FAST-LIO 建图（不启动 Nav2）
  --coverage    按 /map 自动规划可达自由区全覆盖路线（不自动执行）
  --fast-sim    仅配合 SCURM LIO 使用，将 Gazebo 目标时间倍率设为 1.5×

示例：
  ./start_linorobot.sh --explore
  ./start_linorobot.sh --scurm-nav
  ./start_linorobot.sh --scurm-lio
  ./start_linorobot.sh --scurm-lio --coverage
  ./start_linorobot.sh --scurm-lio --fast-sim
  ./start_linorobot.sh --scurm-lio-map map_output_path:=/tmp/my_map.pcd
  ./start_linorobot.sh --explore world:=/absolute/path/to/scene.world

终止：./stop_linorobot.sh
EOF
  exit 0
fi

if [[ "${explore_enabled}" == true ]]; then
  mode="slam"
fi

if [[ "${scurm_navigation}" == true && "${mode}" != "navigation" ]]; then
  echo "错误：--scurm-nav 当前只用于已有地图导航，不能与 --slam/--explore 同时使用。" >&2
  exit 1
fi

if [[ "${scurm_navigation}" == true && "${scurm_lio}" == true ]]; then
  echo "错误：--scurm-nav 和 --scurm-lio/--scurm-lio-map 是不同定位模式，不能同时使用。" >&2
  exit 1
fi

if [[ "${scurm_lio}" == true && ( "${mode}" != "navigation" || "${explore_enabled}" == true ) ]]; then
  echo "错误：SCURM LIO 仿真模式不能与 --slam/--explore 同时使用。" >&2
  exit 1
fi

if [[ "${coverage_enabled}" == true && ( "${mode}" != "navigation" || "${explore_enabled}" == true || "${scurm_lio_mapping}" == true ) ]]; then
  echo "错误：--coverage 需要已有地图和已启动的 Nav2，不能与 --slam/--explore/--scurm-lio-map 同时使用。" >&2
  exit 1
fi

if [[ "${fast_sim}" == true && "${scurm_lio}" != true ]]; then
  echo "错误：--fast-sim 目前只用于 --scurm-lio 或 --scurm-lio-map。" >&2
  exit 1
fi

if [[ "${fast_sim}" == true ]]; then
  # The 3D ray sensor is the limiting Gazebo workload. Keep all 16 vertical
  # channels and the 10 Hz simulation-time rate, reducing only horizontal
  # samples for this opt-in profile. Normal mode remains at 360 samples.
  gazebo_args+=("lidar_horizontal_samples:=120")
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
# Gazebo itself is installed without sudo beside the local ROS underlay. ROS
# package setup covers gazebo_ros; its Ubuntu executable lives under usr/bin.
if [[ -d "${LOCAL_GAZEBO_ROOT}/usr/bin" ]]; then
  export PATH="${LOCAL_GAZEBO_ROOT}/usr/bin:${PATH}"
fi
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
elif [[ "${scurm_navigation}" == true || "${scurm_lio}" == true || "${coverage_enabled}" == true ]]; then
  if [[ ! -f "${PUDU_SETUP}" ]]; then
    echo "错误：未构建 PUDU 工作区：${PUDU_SETUP}" >&2
    exit 1
  fi
  if [[ "${scurm_lio}" == true ]]; then
    if [[ ! -f "${SCURM_SETUP}" ]]; then
      echo "错误：未构建 SCURM underlay：${SCURM_SETUP}" >&2
      exit 1
    fi
    source "${SCURM_SETUP}"
  fi
  source "${PUDU_SETUP}"
fi
set -u

if [[ "${scurm_lio}" == true ]]; then
  # This world only needs Linorobot's playground model. TurtleBot3's model
  # root contains the grouping directory turtlebot3_autorace_2020, which is
  # not itself a Gazebo model and therefore produces a misleading
  # "Missing model.config" GUI error when Gazebo scans the directory.
  turtlebot_model_root="${NAV2_REFERENCE_WS}/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models"
  filtered_model_path=""
  IFS=':' read -ra gazebo_model_paths <<< "${GAZEBO_MODEL_PATH:-}"
  for gazebo_model_path in "${gazebo_model_paths[@]}"; do
    if [[ -z "${gazebo_model_path}" || "${gazebo_model_path}" == "${turtlebot_model_root}" ]]; then
      continue
    fi
    filtered_model_path+="${filtered_model_path:+:}${gazebo_model_path}"
  done
  export GAZEBO_MODEL_PATH="${filtered_model_path}"
  unset turtlebot_model_root filtered_model_path gazebo_model_paths gazebo_model_path
fi

missing_gazebo_dependencies=()
command -v gazebo >/dev/null 2>&1 || missing_gazebo_dependencies+=("gazebo")
ros2 pkg prefix gazebo_ros >/dev/null 2>&1 || missing_gazebo_dependencies+=("gazebo_ros")
ros2 pkg prefix gazebo_plugins >/dev/null 2>&1 || missing_gazebo_dependencies+=("gazebo_plugins")
if [[ "${fast_sim}" == true ]]; then
  command -v gz >/dev/null 2>&1 || missing_gazebo_dependencies+=("gz")
fi
if (( ${#missing_gazebo_dependencies[@]} > 0 )); then
  echo "错误：Gazebo 仿真依赖不完整：${missing_gazebo_dependencies[*]}" >&2
  echo "请运行 ${PUDU_ROBOT_WS}/install_gazebo_local.sh 后重试（无需 sudo）。" >&2
  exit 1
fi

# Gazebo 和 Nav2/SLAM 各自成为独立进程组，停止脚本可完整清理其子进程。
if [[ "${scurm_lio}" == true ]]; then
  nohup setsid ros2 launch pudu_nav2_bringup scurm_sim_gazebo.launch.py \
    "${gazebo_args[@]}" > "${GAZEBO_LOG_FILE}" 2>&1 &
else
  nohup setsid ros2 launch linorobot2_gazebo gazebo.launch.py \
    "${gazebo_args[@]}" > "${GAZEBO_LOG_FILE}" 2>&1 &
fi
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

if [[ "${fast_sim}" == true ]]; then
  # Target RTF = max_step_size * real_time_update_rate = 0.002 * 750 = 1.5.
  # This changes only the current Gazebo process; the source world and normal
  # startup profile remain at their original 1.0x settings.
  if ! gz physics --step-size 0.002 --update-rate 750 >> "${GAZEBO_LOG_FILE}" 2>&1; then
    stop_session "${gazebo_pid}"
    rm -f "${GAZEBO_PID_FILE}"
    echo "Gazebo 1.5× 物理配置应用失败，最近的日志如下：" >&2
    tail -n 50 "${GAZEBO_LOG_FILE}" >&2 || true
    exit 1
  fi
fi

if [[ "${scurm_lio_mapping}" == true ]]; then
  nohup setsid ros2 launch pudu_nav2_bringup scurm_sim_pipeline.launch.py \
    mode:=mapping \
    params_file:="${PUDU_ROBOT_WS}/install/pudu_nav2_bringup/share/pudu_nav2_bringup/config/scurm_fast_lio_sim_mapping.yaml" \
    navigation:=false rviz:=false "${scurm_pipeline_args[@]}" > "${NAV_LOG_FILE}" 2>&1 &
elif [[ "${scurm_lio}" == true ]]; then
  scurm_rviz_args=()
  if [[ "${coverage_enabled}" == true ]]; then
    scurm_rviz_args+=("rviz_config:=${COVERAGE_RVIZ_CONFIG}")
  fi
  nohup setsid ros2 launch pudu_nav2_bringup scurm_sim_pipeline.launch.py \
    mode:=localization navigation:=true rviz:=true \
    "${scurm_rviz_args[@]}" "${scurm_pipeline_args[@]}" > "${NAV_LOG_FILE}" 2>&1 &
elif [[ "${scurm_navigation}" == true ]]; then
  scurm_rviz_args=()
  if [[ "${coverage_enabled}" == true ]]; then
    scurm_rviz_args+=("rviz_config:=${COVERAGE_RVIZ_CONFIG}")
  fi
  nohup setsid ros2 launch pudu_nav2_bringup scurm_navigation.launch.py \
    use_sim_time:=true rviz:=true \
    "${scurm_rviz_args[@]}" > "${NAV_LOG_FILE}" 2>&1 &
else
  navigation_rviz=true
  if [[ "${coverage_enabled}" == true ]]; then
    navigation_rviz=false
  fi
  nohup setsid ros2 launch linorobot2_navigation "${mode}.launch.py" \
    sim:=true rviz:="${navigation_rviz}" > "${NAV_LOG_FILE}" 2>&1 &
fi
nav_pid=$!
echo "${nav_pid}" > "${NAV_PID_FILE}"

nav_ready=false
for _ in {1..30}; do
  if ! session_is_alive "${nav_pid}"; then
    break
  fi
  if grep -Eq "\[rviz2-[0-9]+\]: process started|\[fastlio_mapping-[0-9]+\]: process started|Managed nodes are active" "${NAV_LOG_FILE}"; then
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

if [[ "${scurm_navigation}" == true || "${scurm_lio}" == true ]]; then
  scurm_ready=false
  for _ in {1..60}; do
    if ! session_is_alive "${nav_pid}"; then
      break
    fi
    if grep -Eq "(FATAL|Failed to bring up all requested nodes)" "${NAV_LOG_FILE}"; then
      break
    fi
    if [[ "${scurm_lio_mapping}" == true ]] && grep -Eq "Node init finished" "${NAV_LOG_FILE}"; then
      scurm_ready=true
      break
    fi
    if [[ "${scurm_lio_mapping}" != true ]] && grep -Eq "lifecycle_manager_navigation.*Managed nodes are active" "${NAV_LOG_FILE}"; then
      scurm_ready=true
      break
    fi
    sleep 0.5
  done

  if [[ "${scurm_ready}" != true ]]; then
    stop_session "${nav_pid}"
    stop_session "${gazebo_pid}"
    rm -f "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}"
    echo "SCURM 仿真模式启动失败，最近的日志如下：" >&2
    tail -n 70 "${NAV_LOG_FILE}" >&2 || true
    exit 1
  fi
fi

if [[ "${coverage_enabled}" == true ]]; then
  # Coverage consumes a latched known map and active Nav2 actions. Waiting here
  # keeps the optional node from racing controller_server during lifecycle bringup.
  coverage_nav_ready=false
  for _ in {1..60}; do
    if ! session_is_alive "${nav_pid}"; then
      break
    fi
    if grep -Eq "lifecycle_manager_navigation.*Managed nodes are active|Managed nodes are active" "${NAV_LOG_FILE}"; then
      coverage_nav_ready=true
      break
    fi
    sleep 0.5
  done

  if [[ "${coverage_nav_ready}" != true ]]; then
    stop_session "${nav_pid}"
    stop_session "${gazebo_pid}"
    rm -f "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}"
    echo "区域全覆盖启动失败：Nav2 未在 30 秒内进入 active 状态。" >&2
    tail -n 70 "${NAV_LOG_FILE}" >&2 || true
    exit 1
  fi

  coverage_launch_args=("use_sim_time:=true")
  if [[ "${scurm_navigation}" == true || "${scurm_lio}" == true ]]; then
    coverage_launch_args+=("goal_checker_id:=coverage_goal_checker")
  else
    coverage_launch_args+=("rviz:=true")
  fi
  nohup setsid ros2 launch pudu_coverage_planner area_coverage.launch.py \
    "${coverage_launch_args[@]}" > "${EXPLORE_LOG_FILE}" 2>&1 &
  coverage_pid=$!
  echo "${coverage_pid}" > "${EXPLORE_PID_FILE}"

  coverage_ready=false
  for _ in {1..40}; do
    if ! session_is_alive "${coverage_pid}"; then
      break
    fi
    if grep -Eq "Area coverage ready" "${EXPLORE_LOG_FILE}"; then
      coverage_ready=true
      break
    fi
    sleep 0.25
  done

  if [[ "${coverage_ready}" != true ]]; then
    stop_session "${coverage_pid}"
    stop_session "${nav_pid}"
    stop_session "${gazebo_pid}"
    rm -f "${GAZEBO_PID_FILE}" "${NAV_PID_FILE}" "${EXPLORE_PID_FILE}"
    echo "区域全覆盖节点启动失败，最近的日志如下：" >&2
    tail -n 60 "${EXPLORE_LOG_FILE}" >&2 || true
    exit 1
  fi
fi

if [[ "${scurm_lio_mapping}" == true ]]; then
  mode_text="SCURM 3D FAST-LIO 建图"
elif [[ "${scurm_lio}" == true ]]; then
  mode_text="SCURM 3D ICP + FAST-LIO 定位导航"
elif [[ "${scurm_navigation}" == true ]]; then
  mode_text="SCURM 风格差速导航"
elif [[ "${explore_enabled}" == true ]]; then
  mode_text="前沿覆盖自动建图"
elif [[ "${mode}" == "slam" ]]; then
  mode_text="SLAM 在线建图"
else
  mode_text="Nav2 地图导航"
fi
if [[ "${coverage_enabled}" == true ]]; then
  mode_text="${mode_text} + 可选区域全覆盖"
fi

echo "Linorobot2 仿真已启动（${mode_text}，底盘：${LINOROBOT2_BASE}）。"
if [[ "${fast_sim}" == true ]]; then
  echo "Gazebo 时间倍率：目标 1.5×（物理步长 0.002 s，更新率 750 Hz）"
  echo "高速雷达采样：120×16 @ 10 Hz（普通模式为 360×16 @ 10 Hz）"
else
  echo "Gazebo 时间倍率：普通模式（世界文件默认设置）"
fi
echo "ROS Domain：${ROS_DOMAIN_ID}（其他终端请 source ./setup_linorobot_runtime.bash）"
echo "Gazebo 日志：${GAZEBO_LOG_FILE}"
echo "导航日志：${NAV_LOG_FILE}"
if [[ "${explore_enabled}" == true ]]; then
  echo "探索日志：${EXPLORE_LOG_FILE}"
elif [[ "${coverage_enabled}" == true ]]; then
  echo "覆盖规划日志：${EXPLORE_LOG_FILE}"
  echo "已按 /map 自动规划可达区域；确认路线后启动：ros2 service call /coverage/start std_srvs/srv/Trigger '{}'"
fi
echo "终止：$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/stop_linorobot.sh"
