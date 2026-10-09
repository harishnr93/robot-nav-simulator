#!/usr/bin/env bash
# Build (if needed) and run the full CTest suite.
# Usage: scripts/test.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "${ROOT_DIR}/build/CMakeCache.txt" ]]; then
    "${ROOT_DIR}/scripts/build.sh"
else
    cmake --build "${ROOT_DIR}/build" -j
fi
ctest --test-dir "${ROOT_DIR}/build" --output-on-failure
