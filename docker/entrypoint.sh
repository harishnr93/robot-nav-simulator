#!/usr/bin/env bash
# Source ROS 2 and the robot_nav_sim_ros install, then run the command
# (default: the ROS 2 launch file). Examples, from the repository root:
#   docker compose run --rm sim
#   docker run --rm robot-nav-sim robot_simulator --map maps/obstacles.csv --start 1 1 --goal 18 18
#   docker run --rm robot-nav-sim test
set -e
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${WS}/install_ros2/setup.bash"
mkdir -p "${MPLCONFIGDIR}" "${ROS_HOME}" 2>/dev/null || true
if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
    mkdir -p "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}" 2>/dev/null || true
fi

case "${1:-}" in
    test)   # every test: the standalone CTest suite, then the ROS 2 package's tests
        shift
        exec "${WS}/docker/run_tests.sh" "$@" ;;
    demo)   # one-command demo into results/ (mount it to keep the outputs)
        shift
        exec python "${WS}/python/run_demo.py" --save-gif "$@" ;;
esac
exec "$@"
