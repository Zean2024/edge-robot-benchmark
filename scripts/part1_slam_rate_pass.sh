#!/usr/bin/env bash
# Stress pass at twice the recorded sensor rate. Under the one-core budget the
# pipeline cannot slow the player down, so the observable is how much of the
# sensor stream and of the trajectory survives when the demand doubles.
#
# Environment:
#   PART1_RUNNER       capture wrapper (default scripts/part1_slam_run_limited.sh;
#                      use docker/run_experiment.sh for the container)
#   PART1_DATASETS     dataset list (default "anomaly_ros2 parking1")
#   PART1_EXPERIMENTS  configuration list (default the four groups)
set -uo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

HERE="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
cd "$REPO" || exit 1

read -r -a DATASETS <<< "${PART1_DATASETS:-anomaly_ros2 parking1}"
read -r -a EXPERIMENTS <<< "${PART1_EXPERIMENTS:-slam_toolbox_no_imu slam_toolbox_imu cartographer_no_imu cartographer_imu}"
RUNNER="${PART1_RUNNER:-scripts/part1_slam_run_limited.sh}"

for DATASET in "${DATASETS[@]}"; do
  for EXPERIMENT in "${EXPERIMENTS[@]}"; do
    echo "===== $EXPERIMENT / $DATASET @2x ====="
    PART1_PLAY_RATE=2.0 bash "$RUNNER" "$EXPERIMENT" "$DATASET" x2 || echo "FAILED"
    sleep 2
  done
done

~/.venvs/part1/bin/python scripts/part1_slam_aggregate.py >/dev/null 2>&1 || true
echo "rate pass finished $(date --iso-8601=seconds)"
