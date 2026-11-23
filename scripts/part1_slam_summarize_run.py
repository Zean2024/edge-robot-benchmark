#!/usr/bin/env python3
"""Reduce one run directory to summary.json / summary.txt.

Inputs (produced by part1_slam_capture_run.sh):
    resources.csv          per-process CPU and RSS samples
    tf_filter_stats.log    final TF filter counters
    bag_info.txt           rosbag metadata
    rtf.json               bag duration, wall time, playback rate
    trajectory.tum         recorded trajectory

Outputs:
    summary.json           machine-readable, consumed by evaluate/aggregate
    summary.txt            human-readable, committed for traceability
"""

import csv
import json
import os
import re
import sys
from collections import defaultdict

COUNTER_KEYS = (
    'input_messages',
    'input_transforms',
    'dropped_odom_base',
    'dropped_dynamic_laser',
    'forwarded_transforms',
    'output_messages',
    'scans_received',
    'scans_forwarded',
    'scans_dropped_no_tf',
    'scans_dropped_overflow',
)

ALGORITHM_ROLES = ('slam', 'slam_extra', 'ekf', 'tf_filter', 'imu_filter')


def read_resources(path):
    """Return per-role samples grouped by sampling instant."""
    samples = defaultdict(lambda: defaultdict(float))
    if not os.path.exists(path):
        return samples
    with open(path, newline='', encoding='utf-8') as handle:
        for row in csv.DictReader(handle):
            try:
                elapsed = round(float(row['elapsed_s']), 1)
                cpu = float(row['cpu_pct'])
                rss = float(row['rss_kb'])
            except (KeyError, ValueError):
                continue
            role = row['role']
            samples[elapsed][role + '_cpu'] += cpu
            samples[elapsed][role + '_rss'] += rss
    return samples


def stats(values):
    if not values:
        return {'n': 0, 'mean': 0.0, 'max': 0.0, 'p95': 0.0}
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(0.95 * (len(ordered) - 1))))
    return {
        'n': len(values),
        'mean': round(sum(values) / len(values), 3),
        'max': round(max(values), 3),
        'p95': round(ordered[index], 3),
    }


def read_tf_stats(path):
    counters = {}
    if not os.path.exists(path):
        return counters
    text = open(path, encoding='utf-8', errors='replace').read()
    for key in COUNTER_KEYS:
        match = re.search(rf'{key}=(\d+)', text)
        if match:
            counters[key] = int(match.group(1))
    return counters


def read_bag_duration(path):
    if not os.path.exists(path):
        return 0.0
    match = re.search(r'Duration:\s+([0-9.]+)s', open(path, encoding='utf-8').read())
    return float(match.group(1)) if match else 0.0


def read_trajectory(path):
    info = {'samples': 0, 'first_stamp': 0.0, 'last_stamp': 0.0, 'duration_s': 0.0}
    if not os.path.exists(path):
        return info
    first = last = None
    count = 0
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                stamp = float(parts[0])
            except ValueError:
                continue
            if first is None:
                first = stamp
            last = stamp
            count += 1
    if count:
        info = {
            'samples': count,
            'first_stamp': first,
            'last_stamp': last,
            'duration_s': round(last - first, 3),
        }
    return info


def main():
    run_dir = sys.argv[1]
    samples = read_resources(os.path.join(run_dir, 'resources.csv'))
    tf_stats = read_tf_stats(os.path.join(run_dir, 'tf_filter_stats.log'))
    bag_duration = read_bag_duration(os.path.join(run_dir, 'bag_info.txt'))
    trajectory = read_trajectory(os.path.join(run_dir, 'trajectory.tum'))

    rtf = {}
    rtf_path = os.path.join(run_dir, 'rtf.json')
    if os.path.exists(rtf_path):
        rtf = json.load(open(rtf_path, encoding='utf-8'))

    roles = defaultdict(list)
    algorithm_cpu = []
    algorithm_rss = []
    total_cpu = []
    for _, instant in sorted(samples.items()):
        for key, value in instant.items():
            if key.endswith('_cpu'):
                roles[key[:-4]].append(value)
        algorithm_cpu.append(sum(instant.get(f'{role}_cpu', 0.0) for role in ALGORITHM_ROLES))
        algorithm_rss.append(sum(instant.get(f'{role}_rss', 0.0) for role in ALGORITHM_ROLES))
        total_cpu.append(sum(v for k, v in instant.items() if k.endswith('_cpu')))

    peak_memory_kb = 0
    peak_path = os.path.join(run_dir, 'cgroup_memory_peak.txt')
    if os.path.exists(peak_path):
        try:
            peak_bytes = int(open(peak_path, encoding='utf-8').read().strip())
            if peak_bytes > 0:
                peak_memory_kb = peak_bytes // 1024
        except (ValueError, OSError):
            peak_memory_kb = 0

    summary = {
        'run_dir': os.path.basename(run_dir),
        'bag_duration_s': bag_duration,
        'wall_time_s': rtf.get('wall_time_s', 0.0),
        'play_rate': rtf.get('play_rate', ''),
        'throughput_ratio': rtf.get('throughput_ratio', 0.0),
        'rtf': round(rtf.get('wall_time_s', 0.0) / bag_duration, 3) if bag_duration else 0.0,
        'algorithm_cpu_pct': stats(algorithm_cpu),
        'algorithm_memory_mb': {
            'mean': round(sum(algorithm_rss) / len(algorithm_rss) / 1024, 1) if algorithm_rss else 0.0,
            'max': round(max(algorithm_rss) / 1024, 1) if algorithm_rss else 0.0,
        },
        'total_cpu_pct': stats(total_cpu),
        'cgroup_peak_memory_mb': round(peak_memory_kb / 1024, 1),
        'per_role_cpu_pct': {role: stats(values) for role, values in roles.items()},
        'tf_filter': tf_stats,
        'trajectory': trajectory,
    }

    if tf_stats.get('scans_received'):
        summary['scan_forward_ratio'] = round(
            tf_stats.get('scans_forwarded', 0) / tf_stats['scans_received'], 4
        )
    else:
        summary['scan_forward_ratio'] = 0.0

    with open(os.path.join(run_dir, 'summary.json'), 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2)

    lines = [
        f"run:                 {summary['run_dir']}",
        f"bag duration [s]:    {bag_duration:.2f}",
        f"wall time [s]:       {summary['wall_time_s']}",
        f"play rate:           {summary['play_rate']}",
        f"throughput ratio:    {summary['throughput_ratio']}",
        f"algorithm CPU %:     mean {summary['algorithm_cpu_pct']['mean']}, "
        f"p95 {summary['algorithm_cpu_pct']['p95']}, max {summary['algorithm_cpu_pct']['max']}",
        f"algorithm RSS [MB]:  mean {summary['algorithm_memory_mb']['mean']}, "
        f"max {summary['algorithm_memory_mb']['max']}",
        f"cgroup peak [MB]:    {summary['cgroup_peak_memory_mb']}",
        f"scans forwarded:     {tf_stats.get('scans_forwarded', 0)}/"
        f"{tf_stats.get('scans_received', 0)} (ratio {summary['scan_forward_ratio']})",
        f"trajectory samples:  {trajectory['samples']} "
        f"({trajectory['duration_s']:.1f} s covered)",
    ]
    with open(os.path.join(run_dir, 'summary.txt'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
