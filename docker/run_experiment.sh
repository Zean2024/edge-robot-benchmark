#!/usr/bin/env bash
# Run one capture inside a Docker container with the 1 CPU / 1 GB budget.
#
#   usage: run_experiment.sh EXPERIMENT DATASET [RUN_TAG]
#
# Environment:
#   PART1_IMAGE         image name (default part1:jazzy)
#   PART1_DATA_ROOT     bag directory on the host (default ~/data/M2DGR-plus)
#   PART1_RESULTS_ROOT  results directory on the host
#   PART1_PLAY_RATE     playback rate (default 1.0)
set -euo pipefail

HERE="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
IMAGE="${PART1_IMAGE:-part1:jazzy}"
DATA_ROOT="${PART1_DATA_ROOT:-$HOME/data/M2DGR-plus}"
RESULTS_ROOT="${PART1_RESULTS_ROOT:-$REPO/results/part1_slam/runs}"

exec docker run --rm \
    --network host \
    --cpus=1 \
    --memory=1g \
    --memory-swap=1g \
    -v "$REPO":/repo \
    -v "$DATA_ROOT":/data:ro \
    -v "$RESULTS_ROOT":/results \
    -e PART1_PLAY_RATE="${PART1_PLAY_RATE:-1.0}" \
    -e PART1_DATA_ROOT=/data \
    -e PART1_RESULTS_ROOT=/results \
    "$IMAGE" \
    bash /repo/scripts/part1_slam_capture_run.sh "$@"
