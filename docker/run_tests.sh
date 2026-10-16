#!/usr/bin/env bash
# All tests inside the image: 56 CTest entries (C++, CLI checks, Python suite) and the
# 21 ROS 2 tests. Exits non-zero if any fails.
set -euo pipefail
cd "${WS}"
ctest --test-dir build --output-on-failure -j"$(nproc)"
colcon --log-base /tmp/log_ros2 test --base-paths ros2 \
    --build-base build_ros2 --install-base install_ros2 --event-handlers console_cohesion+
colcon test-result --test-result-base build_ros2 --verbose
