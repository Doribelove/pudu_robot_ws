# Source from Bash. Select V13_PROFILE=p6 or V13_PROFILE=p7-r1.
_v13_l2_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_v13_l2_profile="${V13_PROFILE:-p6}"
case "$_v13_l2_profile" in
  p6|p7-r1) ;;
  *) printf 'Unknown V13_PROFILE: %s\n' "$_v13_l2_profile" >&2; return 2 ;;
esac
source "${V13_RUNTIME_ENV:-/home/robot/workspaces/3ydl3_v13_p5_recovery_schedule_20261027/env.bash}" || return
export V13_L2_DELIVERY_ROOT="$_v13_l2_root"
export V13_PROFILE="$_v13_l2_profile"
export V13_P6_HINT_MODE=local
export V13_P5_HEADING_REFERENCE=0
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$_v13_l2_root/profiles/p6/src:$_v13_l2_root/base/src:$_v13_l2_root/base/runtime/base/src:$_v13_l2_root/base/runtime:$_v13_l2_root/base/deps:$PYTHONPATH"
if [[ "$_v13_l2_profile" == p7-r1 ]]; then
  export PYTHONPATH="$_v13_l2_root/src:$PYTHONPATH"
  export V13_P7_OBSTACLE_L1=1
else
  export V13_P7_OBSTACLE_L1=0
fi
unset _v13_l2_root _v13_l2_profile
