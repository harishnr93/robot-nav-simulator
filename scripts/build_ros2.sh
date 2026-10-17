#!/usr/bin/env bash
# Build (and optionally test) the ROS 2 package ros2/robot_nav_sim_ros with colcon.
# Uses build_ros2/, install_ros2/ and log_ros2/ so it never touches the standalone build/.
# Usage: scripts/build_ros2.sh [--test]   (source your ROS 2 setup.bash first)
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "ROS 2 is not sourced: run 'source /opt/ros/<distro>/setup.bash' first" >&2
    exit 1
fi

cd "${ROOT_DIR}"
COMMON=(--base-paths ros2 --build-base build_ros2 --install-base install_ros2)
colcon --log-base log_ros2 build "${COMMON[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
if [[ "${1:-}" == "--test" ]]; then
    colcon --log-base log_ros2 test "${COMMON[@]}"
    colcon test-result --test-result-base build_ros2 --verbose
fi
echo "Built into ${ROOT_DIR}/install_ros2: source install_ros2/setup.bash"
