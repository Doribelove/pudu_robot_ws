#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O3 -fPIC -shared -ffp-contract=off -fno-fast-math -Wall -Wextra -o libv13_extract_path.so v13_extract_path.cpp
