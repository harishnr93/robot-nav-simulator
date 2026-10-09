#!/usr/bin/env bash
# Plan a path with the C++ simulator, then render the static figures.
# Usage: scripts/run.sh [MAP START_X START_Y GOAL_X GOAL_Y]
# Example: scripts/run.sh maps/corridor.csv 1 1 18 18
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP="${1:-maps/obstacles.csv}"
START_X="${2:-1}"
START_Y="${3:-1}"
GOAL_X="${4:-18}"
GOAL_Y="${5:-18}"
OUTPUT_DIR="${ROOT_DIR}/results"

if [[ -x "${ROOT_DIR}/.venv/bin/python" ]]; then
    PYTHON="${ROOT_DIR}/.venv/bin/python"
else
    PYTHON="python3"
fi

if [[ ! -x "${ROOT_DIR}/build/robot_simulator" ]]; then
    "${ROOT_DIR}/scripts/build.sh"
fi

# A failed plan (exit code 3 or 4) still writes results, so visualize it anyway.
status=0
"${ROOT_DIR}/build/robot_simulator" --map "${MAP}" \
    --start "${START_X}" "${START_Y}" --goal "${GOAL_X}" "${GOAL_Y}" \
    --output "${OUTPUT_DIR}" || status=$?
if [[ ${status} -ne 0 && ${status} -ne 3 && ${status} -ne 4 ]]; then
    exit "${status}"
fi

"${PYTHON}" "${ROOT_DIR}/python/visualize_map.py" --map "${MAP}" --results-dir "${OUTPUT_DIR}"
exit "${status}"
