#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/env.bash"
cmake -S "$workspace_dir/native/nav2_stack" -B "$workspace_dir/build/nav2_stack" -DCMAKE_BUILD_TYPE=Release
cmake --build "$workspace_dir/build/nav2_stack" -j2
cp "$workspace_dir/build/nav2_stack/curve_nav2_stack" "$workspace_dir/native/curve_nav2_stack"
