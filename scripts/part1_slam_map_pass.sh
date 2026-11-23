#!/usr/bin/env bash
# Capture occupancy grids for the map-quality section (Chapter 4.3).
# Runs one clean pass per configuration on a single sequence and stores a
# PGM/YAML pair. No metrics are recorded here; this is a figure-only pass.
set -uo pipefail
source /opt/ros/jazzy/setup.bash

REPO="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/.." && pwd)"
DATASET="${1:-parking1}"
DATASETS_ROOT="${PART1_DATA_ROOT:-$HOME/data/M2DGR-plus}"
OUT_ROOT="$REPO/results/part1_slam/maps"
mkdir -p "$OUT_ROOT"
cd "$REPO" || exit 1

for EXPERIMENT in slam_toolbox_no_imu slam_toolbox_imu cartographer_no_imu cartographer_imu; do
    OUT="$OUT_ROOT/${EXPERIMENT}_${DATASET}"
    rm -rf "$OUT"; mkdir -p "$OUT"
    echo "===== $EXPERIMENT / $DATASET ====="
    setsid ros2 launch "./launch/part1_slam/${EXPERIMENT}.launch.py" > "$OUT/launch.log" 2>&1 &
    LPID=$!
    sleep 10
    timeout 400 ros2 bag play "$DATASETS_ROOT/$DATASET" --clock --rate 1.0 \
        --topics /scan /tf /odom /imu --remap /tf:=/tf_bag > "$OUT/bag.log" 2>&1
    sleep 4

    if [[ "$EXPERIMENT" == cartographer* ]]; then
        timeout 20 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
            "{filename: '$OUT/map.pbstream'}" > "$OUT/write_state.log" 2>&1 || true
        timeout 60 ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
            -pbstream_filename "$OUT/map.pbstream" \
            -map_filestem "$OUT/map" -resolution 0.05 > "$OUT/convert.log" 2>&1 || true
    else
        timeout 40 python3 src/part1_slam/scripts/map_saver.py --ros-args \
            -p output_stem:="$OUT/map" -p timeout_s:=30.0 > "$OUT/save.log" 2>&1 || true
    fi

    kill -INT "$LPID" 2>/dev/null
    sleep 2
    pkill -f 'async_slam_toolbox_node|cartographer_node|ekf_node|tf_filter.py|map_saver|ros2 launch' 2>/dev/null
    sleep 1
    ls -la "$OUT"
done
