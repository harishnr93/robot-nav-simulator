#!/usr/bin/env bash
# Configure and build the C++ code into build/.
# Usage: scripts/build.sh [Release|Debug]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${1:-Release}"

cmake -S "${ROOT_DIR}" -B "${ROOT_DIR}/build" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${ROOT_DIR}/build" -j
echo "Built ${ROOT_DIR}/build/robot_simulator (${BUILD_TYPE})"
