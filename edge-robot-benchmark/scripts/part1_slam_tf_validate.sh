#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
    source "$ROOT/install/setup.bash"
fi

usage() {
    echo "Usage: $0 {slam_toolbox_no_imu|slam_toolbox_imu|cartographer_no_imu|cartographer_imu}"
}

if [[ $# -ne 1 ]]; then
    usage
    exit 2
fi

EXPERIMENT="$1"

case "$EXPERIMENT" in
    slam_toolbox_no_imu)
        LAUNCH_FILE="launch/part1_slam/slam_toolbox_no_imu.launch.py"
        EXPECTED_NODE="/slam_toolbox"
        ;;
    slam_toolbox_imu)
        LAUNCH_FILE="launch/part1_slam/slam_toolbox_imu.launch.py"
        EXPECTED_NODE="/slam_toolbox"
        ;;
    cartographer_no_imu)
        LAUNCH_FILE="launch/part1_slam/cartographer_no_imu.launch.py"
        EXPECTED_NODE="/cartographer_node"
        ;;
    cartographer_imu)
        LAUNCH_FILE="launch/part1_slam/cartographer_imu.launch.py"
        EXPECTED_NODE="/cartographer_node"
        ;;
    *)
        usage
        exit 2
        ;;
esac

BAG="$ROOT/datasets/processed/M2DGR-plus/anomaly_ros2"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/results/part1_slam/tf_validation/${EXPERIMENT}_${STAMP}"

if [[ ! -f "$LAUNCH_FILE" ]]; then
    echo "ERROR: Launch file not found: $LAUNCH_FILE"
    exit 1
fi

if [[ ! -f "$BAG/metadata.yaml" ]]; then
    echo "ERROR: ROS bag metadata not found: $BAG/metadata.yaml"
    exit 1
fi

if [[ ! -f "$ROOT/src/part1_slam/scripts/tf_filter.py" ]]; then
    echo "ERROR: TF filter script not found."
    exit 1
fi

if pgrep -af '[r]os2 bag play' >/dev/null; then
    echo "ERROR: A rosbag player is already running."
    pgrep -af '[r]os2 bag play' || true
    echo "Stop the existing bag playback before running this validator."
    exit 2
fi

if pgrep -af '[r]os2 launch launch/part1_slam/' >/dev/null; then
    echo "ERROR: A Part 1 SLAM launch is already running."
    pgrep -af '[r]os2 launch launch/part1_slam/' || true
    echo "Stop the existing experiment before running this validator."
    exit 2
fi

STALE_REGEX="^python3 $ROOT/src/part1_slam/scripts/(tf_filter|imu_covariance_filter)[.]py$|^/opt/ros/jazzy/lib/(robot_localization/ekf_node|cartographer_ros/cartographer_node|cartographer_ros/cartographer_occupancy_grid_node|slam_toolbox/async_slam_toolbox_node|rosbag2_transport/rosbag2_player)([[:space:]]|$)"

if pgrep -f "$STALE_REGEX" >/dev/null; then
    echo "ERROR: Orphaned Part 1 ROS processes were detected."
    pgrep -af "$STALE_REGEX" || true
    echo "Clean up the old experiment before running this validator."
    exit 2
fi

mkdir -p "$OUT"

LAUNCH_PID=""
BAG_PID=""
EDGE_PID=""
EDGE_AUDIT_RC=0

stop_process() {
    local pid="$1"
    local attempt

    [[ -n "$pid" ]] || return 0

    if kill -0 -- "-$pid" 2>/dev/null; then
        kill -INT -- "-$pid" 2>/dev/null || true

        for attempt in 1 2 3 4 5 6 7 8; do
            kill -0 -- "-$pid" 2>/dev/null || break
            sleep 0.25
        done

        kill -TERM -- "-$pid" 2>/dev/null || true

        for attempt in 1 2 3 4; do
            kill -0 -- "-$pid" 2>/dev/null || break
            sleep 0.25
        done

        kill -KILL -- "-$pid" 2>/dev/null || true
    elif kill -0 "$pid" 2>/dev/null; then
        kill -INT "$pid" 2>/dev/null || true
        sleep 1
        kill -TERM "$pid" 2>/dev/null || true
    fi

    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local rc=$?
    trap - EXIT
    stop_process "$EDGE_PID"
    stop_process "$BAG_PID"
    stop_process "$LAUNCH_PID"
    echo "Processes stopped."
    echo "Report directory: $OUT"
    exit "$rc"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Experiment: $EXPERIMENT"
echo "Launch file: $LAUNCH_FILE"
echo "ROS bag: $BAG"
echo "Report directory: $OUT"

ros2 bag info "$BAG" > "$OUT/bag_info.txt" 2>&1 || true

echo "Launching the algorithm and TF filter..."
setsid ros2 launch "$LAUNCH_FILE" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

READY=0

for attempt in $(seq 1 25); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "ERROR: The launch process exited unexpectedly."
        tail -n 100 "$OUT/launch.log" || true
        exit 1
    fi

    NODES="$(timeout 3s ros2 node list 2>/dev/null || true)"

    if grep -Fxq "/tf_filter" <<< "$NODES" &&
       grep -Fxq "$EXPECTED_NODE" <<< "$NODES"; then
        READY=1
        break
    fi

    sleep 1
done

if [[ "$READY" -ne 1 ]]; then
    echo "ERROR: Required ROS nodes did not become ready."
    {
        echo "=== Node list ==="
        timeout 3s ros2 node list 2>&1 || true
        echo
        echo "=== Launch log ==="
        tail -n 100 "$OUT/launch.log" || true
    } > "$OUT/startup_failure.txt"
    cat "$OUT/startup_failure.txt"
    exit 1
fi

echo "Required nodes are ready."
if [[ ! -f "$ROOT/scripts/part1_slam_tf_edge_audit.py" ]]; then
    echo "ERROR: TF edge audit program not found."
    exit 1
fi

echo "Starting TF edge ownership audit..."
setsid python3 "$ROOT/scripts/part1_slam_tf_edge_audit.py" \
    --duration 25 \
    --output "$OUT/tf_edge_audit.txt" \
    > "$OUT/tf_edge_audit_monitor.log" 2>&1 &
EDGE_PID=$!
sleep 1

echo "Starting rosbag playback with /tf remapped to /tf_bag..."

printf '%s\n' \
    "ros2 bag play $BAG --clock --remap /tf:=/tf_bag" \
    > "$OUT/bag_command.txt"

setsid ros2 bag play "$BAG" --clock --remap /tf:=/tf_bag \
    > "$OUT/bag_play.log" 2>&1 &
BAG_PID=$!

wait_for_message() {
    local topic="$1"
    local output="$2"
    local qos="$3"

    local attempt

    for attempt in $(seq 1 10); do
        if [[ "$qos" == "best_effort" ]]; then
            if timeout --signal=INT 2s \
                ros2 topic echo "$topic" --once \
                --qos-reliability best_effort \
                > "$output" 2>&1; then
                return 0
            fi
        else
            if timeout --signal=INT 2s \
                ros2 topic echo "$topic" --once \
                > "$output" 2>&1; then
                return 0
            fi
        fi

        sleep 0.2
    done

    return 1
}

echo "Waiting for actual input messages..."

TF_INPUT_OK=0
SCAN_INPUT_OK=0

if wait_for_message "/tf_bag" "$OUT/ready_tf_bag.txt" "reliable"; then
    TF_INPUT_OK=1
    echo "PASS: Received a message on /tf_bag."
else
    echo "FAIL: No message received on /tf_bag."
fi

if wait_for_message "/scan" "$OUT/ready_scan.txt" "best_effort"; then
    SCAN_INPUT_OK=1
    echo "PASS: Received a message on /scan."
else
    echo "FAIL: No message received on /scan."
fi

TOPICS=(
    "/odom"
    "/tf_bag"
    "/tf"
    "/tf_static"
    "/scan"
    "/scan_for_slam"
    "/imu"
    "/imu_ekf"
)

for topic in "${TOPICS[@]}"; do
    safe_name="${topic#/}"
    safe_name="${safe_name//\//_}"

    timeout 5s ros2 topic info "$topic" --verbose \
        > "$OUT/topic_${safe_name}_info.txt" 2>&1 || true
done

timeout 5s ros2 node list > "$OUT/node_list.txt" 2>&1 || true

run_rate_probe() {
    local topic="$1"
    local output="$2"

    timeout --signal=INT 8s ros2 topic hz "$topic" \
        > "$output" 2>&1 || true
}

run_tf_probe() {
    local target="$1"
    local source="$2"
    local output="$3"

    timeout --signal=INT 10s \
        ros2 run tf2_ros tf2_echo "$target" "$source" \
        > "$output" 2>&1 || true
}

PROBE_PIDS=()

if [[ "$TF_INPUT_OK" -eq 1 && "$SCAN_INPUT_OK" -eq 1 ]]; then
    echo "Running topic-rate and TF lookup probes..."

    run_rate_probe "/odom" "$OUT/hz_odom.txt" &
    PROBE_PIDS+=("$!")

    run_rate_probe "/tf_bag" "$OUT/hz_tf_bag.txt" &
    PROBE_PIDS+=("$!")

    run_rate_probe "/tf" "$OUT/hz_tf.txt" &
    PROBE_PIDS+=("$!")

    run_rate_probe "/scan" "$OUT/hz_scan.txt" &
    PROBE_PIDS+=("$!")

    run_rate_probe "/scan_for_slam" "$OUT/hz_scan_for_slam.txt" &
    PROBE_PIDS+=("$!")

    run_tf_probe "odom_combined" "base_footprint" \
        "$OUT/tf_odom_to_base.txt" &
    PROBE_PIDS+=("$!")

    run_tf_probe "base_footprint" "laser" \
        "$OUT/tf_base_to_laser.txt" &
    PROBE_PIDS+=("$!")

    run_tf_probe "map" "base_footprint" \
        "$OUT/tf_map_to_base.txt" &
    PROBE_PIDS+=("$!")

    for pid in "${PROBE_PIDS[@]}"; do
        wait "$pid" || true
    done
else
    echo "Skipping TF and rate probes because required input messages are missing."
fi

if [[ -n "$EDGE_PID" ]]; then
    wait "$EDGE_PID" || EDGE_AUDIT_RC=$?
    EDGE_PID=""
fi

grep -Eni \
    '(\[WARN\]|\[ERROR\]|[[:space:]]W[0-9]{8}|[[:space:]]E[0-9]{8}).*(TF_OLD_DATA|Message Filter dropping|Detected jump back|Lookup would require|Could not transform|Failed to transform|Ignored subdivision of a LaserScan message|Failed to meet update rate)' \
    "$OUT/launch.log" "$OUT/bag_play.log" \
    > "$OUT/tf_warning_matches.txt" || true

grep -E 'TF filter stats:' "$OUT/launch.log" \
    > "$OUT/tf_filter_stats.txt" || true

check_file() {
    local file="$1"
    local pattern="$2"

    [[ -f "$file" ]] && grep -Eq "$pattern" "$file"
}

CHECKS_FAILED=0

check_status() {
    local label="$1"
    local result="$2"

    if [[ "$result" -eq 1 ]]; then
        printf 'PASS: %s\n' "$label"
    else
        printf 'FAIL: %s\n' "$label"
        CHECKS_FAILED=1
    fi
}

{
    echo "Experiment: $EXPERIMENT"
    echo "Launch file: $LAUNCH_FILE"
    echo "Bag: $BAG"
    echo "Bag TF remapping: /tf -> /tf_bag"
    echo
    echo "=== TF edge ownership audit ==="

    if [[ -s "$OUT/tf_edge_audit.txt" ]]; then
        cat "$OUT/tf_edge_audit.txt"
        if grep -q '^AUDIT_RESULT: FAIL' "$OUT/tf_edge_audit.txt"; then
            CHECKS_FAILED=1
        fi
    else
        echo "FAIL: TF edge audit report is missing."
        CHECKS_FAILED=1
    fi

    if [[ "$EDGE_AUDIT_RC" -ne 0 ]]; then
        echo "FAIL: TF edge auditor exited with status $EDGE_AUDIT_RC."
        CHECKS_FAILED=1
    fi

    echo
    echo "=== Input readiness ==="

    check_status "/tf_bag received messages" "$TF_INPUT_OK"
    check_status "/scan received messages" "$SCAN_INPUT_OK"

    echo
    echo "=== TF lookup results ==="

    ODOM_TF_OK=0
    LASER_TF_OK=0
    MAP_TF_OK=0

    check_file "$OUT/tf_odom_to_base.txt" 'Translation:' && ODOM_TF_OK=1
    check_file "$OUT/tf_base_to_laser.txt" 'Translation:' && LASER_TF_OK=1
    check_file "$OUT/tf_map_to_base.txt" 'Translation:' && MAP_TF_OK=1

    check_status "odom_combined -> base_footprint" "$ODOM_TF_OK"
    check_status "base_footprint -> laser" "$LASER_TF_OK"
    check_status "map -> base_footprint" "$MAP_TF_OK"

    echo
    echo "=== Topic-rate results ==="

    ODOM_RATE_OK=0
    TF_BAG_RATE_OK=0
    TF_RATE_OK=0
    SCAN_RATE_OK=0
    FILTERED_SCAN_RATE_OK=0

    check_file "$OUT/hz_odom.txt" 'average rate:' && ODOM_RATE_OK=1
    check_file "$OUT/hz_tf_bag.txt" 'average rate:' && TF_BAG_RATE_OK=1
    check_file "$OUT/hz_tf.txt" 'average rate:' && TF_RATE_OK=1
    check_file "$OUT/hz_scan.txt" 'average rate:' && SCAN_RATE_OK=1
    check_file "$OUT/hz_scan_for_slam.txt" 'average rate:' && FILTERED_SCAN_RATE_OK=1

    check_status "/odom rate available" "$ODOM_RATE_OK"
    check_status "/tf_bag rate available" "$TF_BAG_RATE_OK"
    check_status "/tf rate available" "$TF_RATE_OK"
    check_status "/scan rate available" "$SCAN_RATE_OK"
    check_status "/scan_for_slam rate available" "$FILTERED_SCAN_RATE_OK"

    echo
    echo "=== Runtime warning scan ==="

    if [[ -s "$OUT/tf_warning_matches.txt" ]]; then
        echo "WARN: Relevant warning/error signatures were found."
        cat "$OUT/tf_warning_matches.txt"
    else
        echo "PASS: No relevant TF, timestamp, or update-rate warning signatures were found in the captured logs."
    fi

    echo
    echo "=== TF filter statistics ==="

    if [[ -s "$OUT/tf_filter_stats.txt" ]]; then
        cat "$OUT/tf_filter_stats.txt"
    else
        echo "WARN: No TF filter statistics were captured."
    fi

    if [[ "$CHECKS_FAILED" -eq 0 ]]; then
        echo
        echo "OVERALL RESULT: PASS"
    else
        echo
        echo "OVERALL RESULT: FAIL"
    fi

    echo "Report directory: $OUT"
} | tee "$OUT/summary.txt"

if [[ "$CHECKS_FAILED" -ne 0 ]]; then
    exit 1
fi

exit 0
