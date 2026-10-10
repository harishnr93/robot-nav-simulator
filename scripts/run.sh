#!/usr/bin/env bash
# Full demo: plan, simulate, save static figures and results/navigation.gif.
# Usage: scripts/run.sh [MAP START_X START_Y GOAL_X GOAL_Y] [extra run_demo.py options]
# Example: scripts/run.sh maps/maze.csv 1 1 19 19 --speed 1.0
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP="${1:-maps/obstacles.csv}"
START_X="${2:-1}"
START_Y="${3:-1}"
GOAL_X="${4:-18}"
GOAL_Y="${5:-18}"
shift $(( $# < 5 ? $# : 5 ))

if [[ -x "${ROOT_DIR}/.venv/bin/python" ]]; then
    PYTHON="${ROOT_DIR}/.venv/bin/python"
else
    PYTHON="python3"
fi

if [[ ! -x "${ROOT_DIR}/build/robot_simulator" ]]; then
    "${ROOT_DIR}/scripts/build.sh"
fi

exec "${PYTHON}" "${ROOT_DIR}/python/run_demo.py" --map "${MAP}" \
    --start "${START_X}" "${START_Y}" --goal "${GOAL_X}" "${GOAL_Y}" --save-gif "$@"
