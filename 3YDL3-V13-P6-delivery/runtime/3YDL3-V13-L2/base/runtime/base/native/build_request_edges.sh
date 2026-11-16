#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -O3 -fPIC -shared -ffp-contract=off -fno-fast-math -o librequest_edges.so request_edges.cpp
