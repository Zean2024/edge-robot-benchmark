#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# part1_slam_run_all.sh - formal experiment driver
#
#   Runs 4 configurations x 2 datasets x N repeats at real-time playback inside
#   the 1 CPU / 1 GB cgroup scope, evaluates each run, then optionally performs
#   one unthrottled pass per configuration for throughput (RTF) measurement.
#
# Environment:
#   PART1_REPEATS   number of real-time repeats per cell (default 3)
#   PART1_SKIP_RTF  set to 1 to skip the unthrottled throughput pass
#   PART1_DATASETS  override the dataset list, e.g. "anomaly_ros2"
#   PART1_EXPERIMENTS override the configuration list
# ------------------------------------------------------------------------------
set -uo pipefail

HERE="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
cd "$REPO" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
set -u

REPEATS="${PART1_REPEATS:-3}"
SKIP_RTF="${PART1_SKIP_RTF:-0}"
RESULTS="${PART1_RESULTS_ROOT:-$REPO/results/part1_slam/runs}"
EVAL_PY="$HOME/.venvs/part1/bin/python"

# Execution mechanism: cgroup scope on the host (default) or Docker container.
if [[ "${PART1_EXEC:-cgroup}" == docker ]]; then
    RUNNER="$REPO/docker/run_experiment.sh"
else
    RUNNER="$HERE/part1_slam_run_limited.sh"
fi

if [[ -n "${PART1_EXPERIMENTS:-}" ]]; then
    read -r -a EXPERIMENTS <<< "$PART1_EXPERIMENTS"
else
    EXPERIMENTS=(slam_toolbox_no_imu slam_toolbox_imu cartographer_no_imu cartographer_imu)
fi
if [[ -n "${PART1_DATASETS:-}" ]]; then
    read -r -a DATASETS <<< "$PART1_DATASETS"
else
    DATASETS=(anomaly_ros2 parking1)
fi

groundtruth_for() {
    case "$1" in
        anomaly_ros2) echo "datasets/groundtruth/M2DGR-plus/anomaly.tum" ;;
        parking1)     echo "datasets/groundtruth/M2DGR-plus/parking1.tum" ;;
        *)            echo "datasets/groundtruth/M2DGR-plus/$1.tum" ;;
    esac
}

latest_run() {
    ls -dt "$RESULTS/$1/$2"/run_* 2>/dev/null | head -1
}

run_once() {
    local experiment="$1" dataset="$2" tag="$3" rate="$4"
    echo "=================================================================="
    echo "[$(date +%H:%M:%S)] $experiment / $dataset / $tag (rate $rate)"
    echo "=================================================================="
    PART1_PLAY_RATE="$rate" bash "$RUNNER" \
        "$experiment" "$dataset" "$tag" || {
        echo "WARNING: capture failed for $experiment/$dataset/$tag"
        return 1
    }
    local run_dir
    run_dir="$(latest_run "$experiment" "$dataset")"
    if [[ -z "$run_dir" ]]; then
        echo "WARNING: no run directory produced"
        return 1
    fi
    timeout 180 "$EVAL_PY" "$HERE/part1_slam_evaluate.py" \
        --gt "$(groundtruth_for "$dataset")" \
        --est "$run_dir/trajectory.tum" \
        --run-dir "$run_dir" \
        --max-diff 0.12 2>&1 | tail -2
    if [[ ! -f "$run_dir/metrics.json" ]]; then
        echo "WARNING: evaluation produced no metrics.json for $run_dir"
    fi
}

echo "Configurations: ${EXPERIMENTS[*]}"
echo "Datasets:       ${DATASETS[*]}"
echo "Repeats:        $REPEATS"
echo "Results root:   $RESULTS"

# ---------------------------------------------------------------- real-time
for dataset in "${DATASETS[@]}"; do
    for experiment in "${EXPERIMENTS[@]}"; do
        for repeat in $(seq 1 "$REPEATS"); do
            run_once "$experiment" "$dataset" "r$repeat" 1.0
            sleep 2
        done
    done
done

# ------------------------------------------------------- unthrottled (RTF) pass
if [[ "$SKIP_RTF" != "1" ]]; then
    echo
    echo "===== unthrottled throughput pass ====="
    for dataset in "${DATASETS[@]}"; do
        for experiment in "${EXPERIMENTS[@]}"; do
            run_once "$experiment" "$dataset" "rtf" 1000
            sleep 2
        done
    done
fi

# --------------------------------------------------------------------- summary
echo
echo "===== aggregation ====="
"$EVAL_PY" "$HERE/part1_slam_aggregate.py" \
    --runs-root "$RESULTS" \
    --out-root "$REPO/results/part1_slam/evaluation" 2>&1 | tail -5

echo
echo "All runs finished at $(date --iso-8601=seconds)"
