#!/usr/bin/env bash
set -uo pipefail

cd "$HOME/edge-robot-benchmark" || exit 1

# ------------------------------------------------------------------------------
# 1. Parameter Parsing & Validation
# ------------------------------------------------------------------------------
EXPERIMENT="${1:-slam_toolbox_no_imu}"
DATASET="${2:-anomaly_ros2}"

LAUNCH_FILE="launch/part1_slam/${EXPERIMENT}.launch.py"
BAG_PATH="datasets/processed/M2DGR-plus/${DATASET}"

usage() {
    echo "Usage: $0 [EXPERIMENT] [DATASET]"
    echo "  EXPERIMENT: slam_toolbox_no_imu | slam_toolbox_imu | cartographer_no_imu | cartographer_imu (default: slam_toolbox_no_imu)"
    echo "  DATASET:    anomaly_ros2 | Parking_02_ros2 | ... (default: anomaly_ros2)"
}

if [[ ! -f "$LAUNCH_FILE" ]]; then
    echo "ERROR: Launch file not found: $LAUNCH_FILE"
    usage
    exit 1
fi

if [[ ! -d "$BAG_PATH" && ! -f "$BAG_PATH/metadata.yaml" ]]; then
    echo "ERROR: ROS bag dataset not found: $BAG_PATH"
    usage
    exit 1
fi

RUN_DIR="results/part1_slam/${EXPERIMENT}/${DATASET}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
RUN_DIR="$(realpath "$RUN_DIR")"

LAUNCH_PID=""
BAG_PID=""
STARTED_PID=""
MONITORS=()
CLEANING=0

# ------------------------------------------------------------------------------
# 2. Cleanup Stale Project Processes
# ------------------------------------------------------------------------------
clean_stale_project_processes() {
python3 - <<'PY'
import os
import signal
import time

patterns = [
    "ros2 launch launch/part1_slam/",
    "ros2 bag play datasets/processed/",
    "src/part1_slam/scripts/tf_filter.py",
    "src/part1_slam/scripts/imu_covariance_filter.py",
    "/opt/ros/jazzy/lib/robot_localization/ekf_node",
    "/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node",
    "/opt/ros/jazzy/lib/cartographer_ros/cartographer_node",
    "ros2 topic hz",
    "tf2_echo",
    "ros2 topic echo /map",
    "scripts/part1_slam/clock_rate_monitor.py",
]

def get_state(pid):
    try:
        content = open(f"/proc/{pid}/stat").read()
        return content.rsplit(")", 1)[1].strip().split()[0]
    except (OSError, IndexError):
        return None

matches = []
for entry in os.scandir("/proc"):
    if not entry.name.isdigit():
        continue
    pid = int(entry.name)
    if pid == os.getpid():
        continue
    try:
        raw = open(f"/proc/{pid}/cmdline", "rb").read()
        cmd = raw.replace(b"\0", b" ").decode(errors="replace").strip()
    except OSError:
        continue
    if cmd and any(pattern in cmd for pattern in patterns):
        matches.append((pid, cmd))

if matches:
    print("Cleaning leftover project processes:")
    for pid, cmd in matches:
        print(f"  PID {pid}: {cmd[:180]}")
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except PermissionError:
            print(f"  No permission to signal PID {pid}")

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if all(get_state(pid) in (None, "Z", "X") for pid, _ in matches):
            break
        time.sleep(0.1)

    for pid, _ in matches:
        if get_state(pid) not in (None, "Z", "X"):
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
else:
    print("No leftover processes from this diagnostic workflow.")
PY
}

process_group_alive() {
    local pgid="${1:-}"
    [[ -n "$pgid" ]] || return 1

    ps -eo pgid=,stat= 2>/dev/null |
        awk -v target="$pgid" \
        '$1 == target && $2 !~ /^[ZX]/ { found=1 }
         END { exit(found ? 0 : 1) }'
}

signal_group() {
    local sig="$1"
    local pid="$2"
    [[ -n "$pid" ]] || return 0

    if process_group_alive "$pid"; then
        kill -s "$sig" -- "-$pid" 2>/dev/null || true
    fi
}

start_group() {
    local logfile="$1"
    shift

    setsid bash -c 'trap - INT TERM; exec "$@"' _ "$@" \
        >"$logfile" 2>&1 &
    STARTED_PID=$!
}

cleanup() {
    local rc=$?

    if [[ "${CLEANING:-0}" -eq 1 ]]; then
        return 0
    fi
    CLEANING=1

    trap - EXIT INT TERM
    trap '' INT TERM

    echo
    echo "Stopping diagnostic processes (bounded cleanup)..."

    local -a all_pids=(
        "${MONITORS[@]}"
        "${BAG_PID:-}"
        "${LAUNCH_PID:-}"
    )
    local p alive i

    for p in "${MONITORS[@]}"; do
        [[ -n "$p" ]] && signal_group TERM "$p" || true
    done
    signal_group INT "${BAG_PID:-}"
    signal_group INT "${LAUNCH_PID:-}"

    for i in $(seq 1 20); do
        alive=0
        for p in "${all_pids[@]}"; do
            [[ -n "$p" ]] || continue
            if process_group_alive "$p"; then
                alive=1
            fi
        done
        [[ "$alive" -eq 0 ]] && break
        sleep 0.25
    done

    for p in "${all_pids[@]}"; do
        [[ -n "$p" ]] && signal_group TERM "$p" || true
    done
    sleep 0.3

    for p in "${all_pids[@]}"; do
        [[ -n "$p" ]] && signal_group KILL "$p" || true
    done
    sleep 0.3

    for p in "${all_pids[@]}"; do
        [[ -n "$p" ]] || continue
        if ! process_group_alive "$p"; then
            wait "$p" 2>/dev/null || true
        else
            echo "WARNING: process group $p is still alive after SIGKILL."
        fi
    done

    echo "Diagnostic cleanup finished."
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "=========================================================="
echo " Starting Benchmarking Run"
echo " Experiment: $EXPERIMENT"
echo " Dataset:    $DATASET"
echo " Launch:     $LAUNCH_FILE"
echo " Bag:        $BAG_PATH"
echo " Results:    $RUN_DIR"
echo "=========================================================="
echo "Checking for leftover processes from an earlier run..."
clean_stale_project_processes

start_group "$RUN_DIR/launch.log" ros2 launch "$LAUNCH_FILE"
LAUNCH_PID="$STARTED_PID"

echo "Waiting for SLAM node lifecycle and subscriptions..."
READY=0
IS_SLAM_TOOLBOX=0
if [[ "$EXPERIMENT" == slam_toolbox* ]]; then
    IS_SLAM_TOOLBOX=1
fi

for i in $(seq 1 60); do
    NODES="$(timeout 3s ros2 node list 2>/dev/null || true)"
    SCAN_INFO="$(timeout 3s ros2 topic info /scan --verbose 2>/dev/null || true)"
    FILTERED_SCAN_INFO="$(timeout 3s ros2 topic info /scan_for_slam --verbose 2>/dev/null || true)"
    TF_INFO="$(timeout 3s ros2 topic info /tf_bag --verbose 2>/dev/null || true)"

    NODE_READY=0
    if [[ "$IS_SLAM_TOOLBOX" -eq 1 ]]; then
        SLAM_STATE="$(timeout 3s ros2 lifecycle get /slam_toolbox 2>/dev/null || true)"
        if printf '%s\n' "$SLAM_STATE" | grep -Eqi 'active' && printf '%s\n' "$NODES" | grep -qx '/slam_toolbox'; then
            NODE_READY=1
        fi
    else
        if printf '%s\n' "$NODES" | grep -qx '/cartographer_node'; then
            NODE_READY=1
        fi
    fi

    if [[ "$NODE_READY" -eq 1 ]] &&
       printf '%s\n' "$NODES" | grep -qx '/ekf_filter_node' &&
       printf '%s\n' "$NODES" | grep -qx '/tf_filter' &&
       printf '%s\n' "$SCAN_INFO" | grep -q 'Node name: tf_filter' &&
       printf '%s\n' "$FILTERED_SCAN_INFO" | grep -q 'Node name: ' &&
       printf '%s\n' "$TF_INFO" | grep -q 'Node name: tf_filter'; then
        READY=1
        break
    fi
    sleep 1
done

if [[ "$READY" -ne 1 ]]; then
    echo "ERROR: required nodes/subscriptions did not become ready."
    ros2 node list 2>&1 || true
    if [[ "$IS_SLAM_TOOLBOX" -eq 1 ]]; then
        ros2 lifecycle get /slam_toolbox 2>&1 || true
    fi
    tail -n 80 "$RUN_DIR/launch.log"
    exit 1
fi

echo "SLAM node is active; starting rosbag player..."
sleep 2

start_group "$RUN_DIR/bag.log" \
    ros2 bag play "$BAG_PATH" \
    --clock --rate 0.5 --remap /tf:=/tf_bag
BAG_PID="$STARTED_PID"

# Start all monitors in separate groups while the bag is playing.
start_group "$RUN_DIR/scan_hz.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 topic hz --window 10 /scan
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/scan_filtered_hz.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 topic hz --window 10 /scan_for_slam
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/odom_hz.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 topic hz --window 10 /odometry/filtered
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/clock_hz.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 topic hz --window 10 /clock
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/map_hz.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 topic hz --window 5 /map
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/tf_odom_base.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 run tf2_ros tf2_echo \
    odom_combined base_footprint --ros-args -p use_sim_time:=true
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/tf_map_base.log" \
    timeout --signal=TERM --kill-after=2s 900 \
    ros2 run tf2_ros tf2_echo \
    map base_footprint --ros-args -p use_sim_time:=true
MONITORS+=("$STARTED_PID")

start_group "$RUN_DIR/map_header.log" \
    timeout --signal=TERM --kill-after=2s 60 \
    ros2 topic echo /map --once --field header
MONITORS+=("$STARTED_PID")

echo '"time","pid","cpu_percent","memory_percent","elapsed","command"' \
    > "$RUN_DIR/resources.csv"

echo "Monitoring rosbag playback, topics, TF and resources..."
SECONDS=0

while kill -0 "$BAG_PID" 2>/dev/null; do
    state="$(ps -o stat= -p "$BAG_PID" 2>/dev/null | tr -d ' ')"
    [[ -z "$state" || "$state" == Z* ]] && break

    now="$(date --iso-8601=seconds)"
    ps -eo pid=,pcpu=,pmem=,etime=,args= |
        grep -E \
        'async_slam_toolbox_node|cartographer_node|robot_localization/ekf_node|src/part1_slam/scripts/tf_filter.py|ros2 bag play' |
        grep -v grep |
        while read -r pid cpu mem elapsed command; do
            printf '"%s",%s,%s,%s,"%s","%s"\n' \
                "$now" "$pid" "$cpu" "$mem" "$elapsed" "$command" \
                >> "$RUN_DIR/resources.csv"
        done

    if (( SECONDS > 900 )); then
        echo "Monitoring limit reached; stopping run safely." \
            | tee "$RUN_DIR/time_limit.txt"
        exit 124
    fi
    sleep 2
done

# Reap the naturally completed bag process before cleanup.
wait "$BAG_PID"
BAG_RC=$?
printf '%s\n' "$BAG_RC" > "$RUN_DIR/bag_exit_code.txt"
BAG_PID=""

echo "Rosbag playback ended (exit code $BAG_RC)."
echo "Stopping monitors and SLAM; cleanup has a bounded timeout."
exit "$BAG_RC"