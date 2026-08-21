#!/usr/bin/env bash

set -euo pipefail

readonly PUDU_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${PUDU_WS}/stack_paths.bash"
readonly ARENA_PATCHES=(
  "${PUDU_WS}/dependencies/patches/arena4-jackal-baseline.patch"
  "${PUDU_WS}/dependencies/patches/arena4-evaluation.patch"
)

if [[ ! -d "${ARENA4_WS}/src" || ! -x "${ARENA4_WS}/colcon_build" ]]; then
  echo "错误：Arena4 工作区不完整：${ARENA4_WS}" >&2
  exit 1
fi

mapfile -t arena_launches < <(pgrep -f '[r]os2 launch arena_bringup arena.launch.py' 2>/dev/null || true)
if (( ${#arena_launches[@]} > 0 )); then
  echo "错误：Arena4 正在运行，拒绝同时重编其 install 空间。" >&2
  ps -o pid=,ppid=,sid=,stat=,args= -p "$(IFS=,; echo "${arena_launches[*]}")" >&2 || true
  echo "请先在原终端按 Ctrl+C，或对统一入口启动的实例运行 ./stop_arena4.sh。" >&2
  exit 1
fi

# Apply the patch idempotently.  Arena4 is intentionally kept outside the
# parent repository, so its files may contain compatible local edits that make
# `patch` return 1 after skipping already-applied hunks.  Never let patch enter
# its interactive "Assume -R?" prompt during a build.
for arena_patch in "${ARENA_PATCHES[@]}"; do
  patch_output=""
  patch_rc=0
  patch_output=$(patch --directory="${ARENA4_WS}" --strip=1 --forward --batch --dry-run -N \
    < "${arena_patch}" 2>&1) || patch_rc=$?
  if grep -qE "FAILED|malformed|can.t find file|Only garbage" <<<"${patch_output}"; then
    printf '%s\n' "${patch_output}" >&2
    echo "错误：Arena4 补丁与当前源码存在未解决冲突：${arena_patch}" >&2
    exit 1
  fi

  if (( patch_rc == 0 )); then
    apply_output=$(patch --directory="${ARENA4_WS}" --strip=1 --forward --batch -N \
      < "${arena_patch}" 2>&1) || {
      printf '%s\n' "${apply_output}" >&2
      echo "错误：Arena4 补丁应用失败：${arena_patch}" >&2
      exit 1
    }
    printf '%s\n' "${apply_output}"
    echo "Arena4 patch applied: ${arena_patch##*/}"
  else
    echo "Arena4 patch already applied (or compatible hunks skipped): ${arena_patch##*/}"
  fi
done

echo "==> 独立增量编译 Arena4: ${ARENA4_WS}"
(
  unset ARENA_SOURCED ARENA_WS_DIR INSTALLED
  cd "${ARENA4_WS}"
  set +u
  source ./arena.bash
  set -u
  if [[ -d "${ARENA4_G2O_PREFIX}" ]]; then
    export CMAKE_PREFIX_PATH="${ARENA4_G2O_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  fi
  ./colcon_build "$@"
)
