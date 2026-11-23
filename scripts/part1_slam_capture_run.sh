#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# part1_slam_capture_run.sh - one controlled SLAM run under a resource budget
#
#   usage: part1_slam_capture_run.sh EXPERIMENT DATASET [RUN_TAG]
#
#   EXPERIMENT : slam_toolbox_no_imu | slam_toolbox_imu
#                cartographer_no_imu | cartographer_imu
#   DATASET    : anomaly | parking1 | ... (a bag directory under PART1_DATA_ROOT)
#   RUN_TAG    : optional suffix, e.g. r1
#
# Environment:
#   PART1_DATA_ROOT     default $HOME/data/M2DGR-plus
#   PART1_RESULTS_ROOT  default <repo>/results/part1_slam/runs
#   PART1_PLAY_RATE     default 1.0 (use 1000 for the unthrottled RTF pass)
#   PART1_MAX_WALL      optional hard wall-clock cap in seconds
#
# Every run directory is self-describing: command, git revision, config hashes,
# bag info, launch log, bag log, resource samples, TF filter counters,
# trajectory, and a summary.json. No number in the thesis may come from
# anywhere else.
# ------------------------------------------------------------------------------
set -uo pipefail

SCRIPT_PATH="$(realpath "${BASH_SOURCE[0]}")"
REPO="$(cd "$(dirname "$SCRIPT_PATH")/.." && pwd)"

if [[ $# -lt 2 ]]; then
    sed -n '2,20p' "$SCRIPT_PATH"
    exit 2
fi

EXPERIMENT="$1"
DATASET="$2"
RUN_TAG="${3:-}"

DATA_ROOT="${PART1_DATA_ROOT:-$HOME/data/M2DGR-plus}"
RESULTS_ROOT="${PART1_RESULTS_ROOT:-$REPO/results/part1_slam/runs}"
PLAY_RATE="${PART1_PLAY_RATE:-1.0}"
MAX_WALL="${PART1_MAX_WALL:-0}"

BAG="$DATA_ROOT/$DATASET"
LAUNCH="$REPO/launch/part1_slam/${EXPERIMENT}.launch.py"
RECORDER="$REPO/src/part1_slam/scripts/trajectory_recorder.py"
SAMPLER="$REPO/src/part1_slam/scripts/resource_sampler.py"

if [[ ! -f "$LAUNCH" ]]; then
    echo "ERROR: missing launch file $LAUNCH" >&2
    exit 1
fi
if [[ ! -d "$BAG" ]]; then
    echo "ERROR: missing bag directory $BAG" >&2
    exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
SUFFIX=""
[[ -n "$RUN_TAG" ]] && SUFFIX="_${RUN_TAG}"
RUN_DIR="$RESULTS_ROOT/$EXPERIMENT/$DATASET/run_${STAMP}${SUFFIX}"
mkdir -p "$RUN_DIR"
RUN_DIR="$(realpath "$RUN_DIR")"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$REPO" || exit 1

LAUNCH_PID=""
BAG_PID=""
RECORDER_PID=""
SAMPLER_PID=""
CLEANING=0

stop_group() {
    local pid="${1:-}"
    [[ -n "$pid" ]] || return 0
    kill -s INT -- "-$pid" 2>/dev/null || kill -s INT "$pid" 2>/dev/null || true
}

kill_group() {
    local pid="${1:-}"
    [[ -n "$pid" ]] || return 0
    kill -s TERM -- "-$pid" 2>/dev/null || kill -s TERM "$pid" 2>/dev/null || true
}

cleanup() {
    local rc=$?
    [[ "$CLEANING" -eq 1 ]] && return 0
    CLEANING=1
    trap - EXIT INT TERM

    stop_group "${RECORDER_PID:-}"
    kill_group "${SAMPLER_PID:-}"
    kill_group "${BAG_PID:-}"
    kill_group "${LAUNCH_PID:-}"
    sleep 2
    for pid in "${RECORDER_PID:-}" "${SAMPLER_PID:-}" "${BAG_PID:-}" "${LAUNCH_PID:-}"; do
        [[ -n "$pid" ]] || continue
        kill -s KILL -- "-$pid" 2>/dev/null || true
    done
    exit "$rc"
}
trap cleanup EXIT INT TERM

echo "=========================================================="
echo " run:        $EXPERIMENT / $DATASET"
echo " run dir:    $RUN_DIR"
echo " bag:        $BAG"
echo " play rate:  $PLAY_RATE"
echo "=========================================================="

# ---------------------------------------------------------------- provenance
{
    echo "experiment=$EXPERIMENT"
    echo "dataset=$DATASET"
    echo "play_rate=$PLAY_RATE"
    echo "run_dir=$RUN_DIR"
    echo "date=$(date --iso-8601=seconds)"
    echo "host=$(uname -a)"
    echo "repo=$REPO"
    echo "git_rev=$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo unknown)"
    echo "git_describe=$(git -C "$REPO" describe --always --dirty 2>/dev/null || echo unknown)"
    echo "command=$0 $*"
} > "$RUN_DIR/command.txt"
git -C "$REPO" status --porcelain > "$RUN_DIR/git_status.txt" 2>&1 || true

sha256sum \
    "$LAUNCH" \
    "$REPO/launch/part1_slam/"*.launch.py \
    "$REPO/configs/part1_slam/"*.lua \
    "$REPO/configs/part1_slam/"*.yaml \
    "$REPO/scripts/part1_slam_capture_run.sh" \
    "$REPO/src/part1_slam/scripts/tf_filter.py" \
    "$REPO/src/part1_slam/scripts/imu_covariance_filter.py" \
    "$REPO/src/part1_slam/scripts/trajectory_recorder.py" \
    2>/dev/null | sed "s#$REPO/##" > "$RUN_DIR/config_hashes.txt"

ros2 bag info "$BAG" > "$RUN_DIR/bag_info.txt" 2>&1 || true

# ------------------------------------------------------------------- launch
echo "Launching $EXPERIMENT ..."
setsid ros2 launch "./launch/part1_slam/${EXPERIMENT}.launch.py" \
    > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

SLAM_NODE="slam_toolbox"
[[ "$EXPERIMENT" == cartographer* ]] && SLAM_NODE="cartographer_node"

READY=0
for _ in $(seq 1 90); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "ERROR: launch died during startup" >&2
        break
    fi
    NODES="$(timeout 3 ros2 node list 2>/dev/null || true)"
    PUBLISHERS="$(timeout 3 ros2 topic info /scan_for_slam --verbose 2>/dev/null || true)"
    if grep -qx "/$SLAM_NODE" <<<"$NODES" \
        && grep -qx "/tf_filter" <<<"$NODES" \
        && grep -qx "/ekf_filter_node" <<<"$NODES" \
        && grep -q 'Node name: tf_filter' <<<"$PUBLISHERS"; then
        READY=1
        break
    fi
    sleep 1
done

if [[ "$READY" -ne 1 ]]; then
    echo "ERROR: stack did not become ready" >&2
    tail -40 "$RUN_DIR/launch.log" >&2
    exit 1
fi

if [[ "$SLAM_NODE" == "slam_toolbox" ]]; then
    for _ in $(seq 1 30); do
        STATE="$(timeout 3 ros2 lifecycle get /slam_toolbox 2>/dev/null || true)"
        grep -q 'active' <<<"$STATE" && break
        sleep 1
    done
    timeout 3 ros2 lifecycle get /slam_toolbox > "$RUN_DIR/lifecycle.txt" 2>&1 || true
fi

# ------------------------------------------------------------ instrumentation
echo "Starting trajectory recorder and resource sampler ..."
setsid python3 "$RECORDER" \
    --ros-args \
    -p use_sim_time:=true \
    -p output_file:="$RUN_DIR/trajectory.tum" \
    -p map_frame:=map \
    -p base_frame:=base_footprint \
    -p sample_period:=0.05 \
    > "$RUN_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!

setsid python3 "$SAMPLER" \
    --out "$RUN_DIR/resources.csv" \
    --interval 0.5 \
    --peak-file "$RUN_DIR/cgroup_memory_peak.txt" \
    > "$RUN_DIR/sampler.log" 2>&1 &
SAMPLER_PID=$!

sleep 1

# ------------------------------------------------------------------ playback
echo "Playing bag ..."
PLAY_START=$(date +%s.%N)
setsid ros2 bag play "$BAG" \
    --clock \
    --rate "$PLAY_RATE" \
    --topics /scan /tf /odom /imu \
    --remap /tf:=/tf_bag \
    > "$RUN_DIR/bag.log" 2>&1 &
BAG_PID=$!

BAG_DURATION="$(awk '/Duration:/ {gsub(/s$/,"",$2); print $2; exit}' "$RUN_DIR/bag_info.txt" 2>/dev/null)"
[[ -z "$BAG_DURATION" ]] && BAG_DURATION=0
if [[ "$MAX_WALL" -le 0 ]]; then
    MAX_WALL="$(python3 -c "print(int(3*float('${BAG_DURATION:-1}')+180))" 2>/dev/null || echo 600)"
fi

while kill -0 "$BAG_PID" 2>/dev/null; do
    NOW=$(date +%s)
    if (( NOW - ${PLAY_START%.*} > MAX_WALL )); then
        echo "WARNING: wall-clock cap reached, stopping playback" | tee "$RUN_DIR/time_limit.txt"
        kill_group "$BAG_PID"
        break
    fi
    sleep 1
done
wait "$BAG_PID" 2>/dev/null
BAG_RC=$?
PLAY_END=$(date +%s.%N)
echo "$BAG_RC" > "$RUN_DIR/bag_exit_code.txt"

WALL="$(python3 -c "print(round(float('${PLAY_END}')-float('${PLAY_START}'),3))")"
python3 - "$RUN_DIR" "$BAG_DURATION" "$WALL" "$PLAY_RATE" <<'PY'
import json, sys
run_dir, bag_duration, wall, rate = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
rtf = (bag_duration / wall) if wall > 0 else 0.0
data = {'bag_duration_s': bag_duration, 'wall_time_s': wall, 'play_rate': rate,
        'throughput_ratio': round(rtf, 4)}
open(f'{run_dir}/rtf.json', 'w').write(json.dumps(data, indent=2))
PY

# Give loop closure / final optimisation a moment to settle.
echo "Settling for final optimisation ..."
sleep 4

if [[ "$EXPERIMENT" == cartographer* ]]; then
    timeout 20 ros2 service call /write_state \
        cartographer_ros_msgs/srv/WriteState \
        "{filename: '$RUN_DIR/map.pbstream'}" \
        > "$RUN_DIR/write_state.log" 2>&1 || true
fi

# --------------------------------------------------------------- teardown
stop_group "$RECORDER_PID"
sleep 2
RECORDER_PID=""

grep 'TF filter stats' "$RUN_DIR/launch.log" | tail -1 > "$RUN_DIR/tf_filter_stats.log" 2>/dev/null || true
grep -E 'Tracking|loop closure|Loop closure|Optimization|Registering sensor|Adding' \
    "$RUN_DIR/launch.log" > "$RUN_DIR/slam_events.log" 2>/dev/null || true

kill_group "$LAUNCH_PID"
sleep 3
kill_group "$SAMPLER_PID"
sleep 1
SAMPLER_PID=""
LAUNCH_PID=""
BAG_PID=""

python3 "$REPO/scripts/part1_slam_summarize_run.py" "$RUN_DIR" \
    > "$RUN_DIR/summarize.log" 2>&1 || true

echo "Run finished. Artifacts in $RUN_DIR"
LC_ALL=C find "$RUN_DIR" -maxdepth 1 -type f -printf '%f\n' | sort
