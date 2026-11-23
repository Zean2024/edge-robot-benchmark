# Benchmark image: ROS 2 Jazzy + Cartographer + SLAM Toolbox + robot_localization.
#
# The resource budget (1 CPU core, 1 GB RAM, no swap) is applied at run time
# with `docker run --cpus=1 --memory=1g --memory-swap=1g`, which is enforced by
# the same kernel cgroup v2 controllers that the host-side systemd scope used
# during development (see 08_decision_log.md, D10/D14).
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-cartographer \
        ros-jazzy-cartographer-ros \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-rosbag2 \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-tf2-ros \
        git \
        procps \
    && rm -rf /var/lib/apt/lists/*

COPY . /repo
WORKDIR /repo

CMD ["/bin/bash"]
