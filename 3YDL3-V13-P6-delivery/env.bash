# Source from Bash to select the delivered P6 / H-I runtime.
_p6_delivery_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export V13_PROFILE=p6
export V13_CLOSEOUT_PACKAGE="$_p6_delivery_root/runtime/3YDL3-V13-L2"
source "$V13_CLOSEOUT_PACKAGE/env.bash" || return
export V13_P7_OBSTACLE_L1=0
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$_p6_delivery_root/tools:$PYTHONPATH"
unset _p6_delivery_root
