#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
for name in v13_dstar v13_segment_cells v13_extract_path; do
  g++ -std=c++17 -O3 -fPIC -shared -ffp-contract=off -fno-fast-math -Wall -Wextra -o "lib${name}.so" "${name}.cpp"
done
