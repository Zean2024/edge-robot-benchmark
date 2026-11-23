#!/usr/bin/env bash
# Stress pass at twice the recorded sensor rate. Under the one-core budget the
# pipeline cannot slow the player down, so the observable is how much of the
# sensor stream and of the trajectory survives when the demand doubles.
set -uo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

REPO=/mnt/e/Data/01-Study/LUT/BachelorThesis/repos/edge-robot-benchmark
cd "$REPO" || exit 1

for DATASET in anomaly_ros2 parking1; do
  for EXPERIMENT in slam_toolbox_no_imu slam_toolbox_imu cartographer_no_imu cartographer_imu; do
    echo "===== $EXPERIMENT / $DATASET @2x ====="
    PART1_PLAY_RATE=2.0 bash scripts/part1_slam_run_limited.sh "$EXPERIMENT" "$DATASET" x2 || echo "FAILED"
    sleep 2
  done
done

~/.venvs/part1/bin/python scripts/part1_slam_aggregate.py >/dev/null 2>&1 || true
echo "rate pass finished $(date --iso-8601=seconds)"
