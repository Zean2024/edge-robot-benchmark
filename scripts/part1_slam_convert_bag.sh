#!/usr/bin/env bash
# Convert an M2DGR-plus ROS 1 bag to a filtered ROS 2 bag.
#
#   usage: part1_slam_convert_bag.sh SOURCE.bag DESTINATION_DIR
#
# Only /scan, /tf, /odom and /imu are kept, because no evaluated configuration
# consumes the camera streams or the point cloud, and the images dominate the
# original bag size. The conversion preserves the recorded timestamps, so the
# reference trajectory and the replayed stream share one time base.
#
# Environment:
#   ROSBAGS_CONVERT   converter binary (default: part1 venv, then PATH)
set -euo pipefail

if [[ $# -ne 2 ]]; then
    sed -n '2,14p' "$0"
    exit 2
fi

SOURCE="$1"
DEST="$2"

if [[ ! -f "$SOURCE" ]]; then
    echo "ERROR: source bag not found: $SOURCE" >&2
    exit 1
fi
if [[ -e "$DEST" ]]; then
    echo "ERROR: destination already exists: $DEST" >&2
    exit 1
fi

CONVERTER="${ROSBAGS_CONVERT:-$HOME/.venvs/part1/bin/rosbags-convert}"
if [[ ! -x "$CONVERTER" ]]; then
    CONVERTER="$(command -v rosbags-convert || true)"
fi
if [[ -z "${CONVERTER:-}" ]]; then
    echo "ERROR: rosbags-convert not found (pip install rosbags)" >&2
    exit 1
fi

mkdir -p "$(dirname "$DEST")"
"$CONVERTER" \
    --src "$SOURCE" \
    --dst "$DEST" \
    --dst-storage sqlite3 \
    --dst-typestore ros2_jazzy \
    --include-topic /scan /tf /odom /imu

echo "converted $SOURCE -> $DEST"
