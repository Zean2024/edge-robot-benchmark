#!/usr/bin/env bash
# Run one capture inside a cgroup v2 scope that emulates the target device:
# one CPU core and one gigabyte of RAM. This mirrors `docker run --cpus=1
# --memory=1g` without requiring a Docker daemon on the machine.
set -uo pipefail

HERE="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
SCRIPT="$HERE/part1_slam_capture_run.sh"
CPU_QUOTA="${PART1_CPU_QUOTA:-100%}"
MEM_MAX="${PART1_MEM_MAX:-1G}"
UNIT="part1-$(date +%s)-$$"

exec systemd-run --user --scope --collect \
    --unit="$UNIT" \
    -p CPUQuota="$CPU_QUOTA" \
    -p MemoryMax="$MEM_MAX" \
    -p MemorySwapMax=0 \
    -- "$SCRIPT" "$@"
