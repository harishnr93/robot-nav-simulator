# robot-nav-simulator: the standalone simulator, the Python tools and the ROS 2 package
# (with RViz2) in one image. See docs/docker.md.
#
#   docker build -t robot-nav-sim .                              # everything
#   docker build -t robot-nav-sim --build-arg INSTALL_RVIZ=false .   # headless, smaller
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG INSTALL_RVIZ=true
ENV DEBIAN_FRONTEND=noninteractive \
    WS=/ws \
    MPLBACKEND=Agg \
    MPLCONFIGDIR=/tmp/matplotlib \
    ROS_HOME=/tmp/ros \
    PYTHONDONTWRITEBYTECODE=1

# System packages, then the ROS 2 package's dependencies from its package.xml (rosdep).
# Only package.xml is copied here so that source edits don't invalidate this layer.
COPY ros2/robot_nav_sim_ros/package.xml /tmp/deps/robot_nav_sim_ros/package.xml
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential cmake python3-pip python3-venv \
        python3-colcon-common-extensions python3-rosdep ffmpeg \
    && SKIP="" && if [ "${INSTALL_RVIZ}" != "true" ]; then SKIP="--skip-keys rviz2"; fi \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/deps --ignore-src -y --rosdistro "${ROS_DISTRO}" ${SKIP} \
    && rm -rf /var/lib/apt/lists/* /tmp/deps

# Python environment for the visualization tools. --system-site-packages keeps rclpy and
# the other ROS Python packages importable, so one interpreter serves both. NumPy stays
# below 2: Humble's message packages are compiled against the NumPy 1.x ABI.
WORKDIR ${WS}
COPY requirements.txt .
RUN python3 -m venv --system-site-packages .venv \
    && .venv/bin/pip install --no-cache-dir -r requirements.txt "numpy<2"

# Standalone simulator (build/) and ROS 2 package (build_ros2/, install_ros2/), exactly
# as on the host: scripts/build.sh and scripts/build_ros2.sh.
COPY . .
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon --log-base log_ros2 build --base-paths ros2 \
        --build-base build_ros2 --install-base install_ros2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf log_ros2

ENV PATH=${WS}/.venv/bin:${WS}/build:${PATH}
ENTRYPOINT ["/ws/docker/entrypoint.sh"]
CMD ["ros2", "launch", "robot_nav_sim_ros", "sim.launch.py"]
