#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace_dir/env.bash"
exec python3 "$workspace_dir/tools/run_v7.py" "$@"
