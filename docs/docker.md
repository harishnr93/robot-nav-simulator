# Docker

One image holds everything:

- the standalone C++ simulator (`build/robot_simulator` and the experiments);
- the Python visualization tools, with their virtual environment;
- the ROS 2 package with RViz2.

The image is based on `ros:humble-ros-base` (Ubuntu 22.04). It is built exactly as on a
host: `cmake` into `build/`, colcon into `build_ros2/` and `install_ros2/`.

## Build

```bash
docker compose build                       # or: docker build -t robot-nav-sim:latest .
docker build -t robot-nav-sim --build-arg INSTALL_RVIZ=false .   # headless, without RViz2/Qt
```

| Build argument | Default | Meaning |
|---|---|---|
| `ROS_DISTRO` | `humble` | base image `ros:<distro>-ros-base` |
| `INSTALL_RVIZ` | `true` | install RViz2 (skip it for servers and CI) |

Dependencies come from `requirements.txt` and from the ROS 2 package's `package.xml`
(via `rosdep`). Both layers are cached, so a source edit rebuilds only the C++ code.
`.dockerignore` keeps host builds (`build/`, `build_ros2/`, `install_ros2/`, `.venv/`)
and `results/` out of the image.

## Run

| Command | What it runs |
|---|---|
| `docker compose up sim` | map server, simulator and RViz2 (needs a display, see below) |
| `docker compose up headless` | the two ROS 2 nodes without RViz2 |
| `docker compose run --rm demo` | `run_demo.py --save-gif`: CSV results, figures and `navigation.gif` in `./results` |
| `docker compose run --rm test` | every test: the 63 CTest entries, then the 21 ROS 2 tests |
| `docker compose run --rm sim robot_simulator --map maps/obstacles.csv --start 1 1 --goal 18 18` | the standalone CLI |
| `docker compose run --rm sim bash` | a shell with ROS 2 and the package sourced |

Any command runs inside the image: the entrypoint sources ROS 2 and the package, and puts
`robot_simulator` and the Python environment on `PATH`. Launch arguments pass through:

```bash
docker compose run --rm sim ros2 launch robot_nav_sim_ros sim.launch.py \
    robot_model:=diff-drive lidar_beams:=36 ekf:=true
docker compose run --rm sim ros2 launch robot_nav_sim_ros sim.launch.py \
    dynamic_obstacles_file:=/ws/maps/obstacles_dynamic.csv robot_model:=diff-drive
```

Maps live in `/ws/maps`. To use your own, mount a directory:
`-v "$PWD/my_maps:/maps"` and `map_file:=/maps/x.csv`.

The `demo` service runs as `${UID}:${GID}` (default 1000:1000), so the files it writes
to `./results` belong to you. `UID` is not exported by bash, so use
`UID=$(id -u) GID=$(id -g) docker compose run --rm demo` if your IDs differ.

## RViz2 display

The `sim` service uses the host network (also for DDS discovery) and forwards
`DISPLAY` and `/tmp/.X11-unix`. RViz2 renders in software (`LIBGL_ALWAYS_SOFTWARE=1`),
so no GPU passthrough is needed.

| Host | Setup |
|---|---|
| Windows 11 / WSL2 (WSLg) | nothing: `DISPLAY` is already set inside WSL |
| WSL2 with an X server on Windows (VcXsrv, X410) | allow connections from WSL in the X server; keep the `DISPLAY` WSL sets |
| Linux desktop (X11 or XWayland) | `xhost +local:docker` once per session |
| no display (server, CI) | `docker compose up headless`; view from another machine's RViz2 on the same `ROS_DOMAIN_ID` |

The host network means a host RViz2, or any `ros2 topic echo` on the host, sees the
container's topics directly, provided the host and the container use the same ROS 2
distribution and `ROS_DOMAIN_ID`.

## Checked by CI

The `docker` job in `.github/workflows/ci.yml` builds the image (headless) and runs
`test` in it. It also runs the CLI, and the `demo` with a mounted `results/`, checking
that the GIF was written.
