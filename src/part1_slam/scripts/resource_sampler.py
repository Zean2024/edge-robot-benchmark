#!/usr/bin/env python3
"""Sample per-process CPU and RSS for a SLAM run.

ps reports the average CPU usage since process start, which hides the short
peaks that actually decide whether a one-core budget is sufficient. This
sampler reads /proc/<pid>/stat deltas instead and tags every sample with the
role of the process, so the aggregator can exclude tooling (bag player,
recorder) from the algorithmic overhead.
"""

import os
import signal
import sys
import time

CLK_TCK = os.sysconf('SC_CLK_TCK')
PAGE_SIZE = os.sysconf('SC_PAGE_SIZE')

ROLE_PATTERNS = (
    ('async_slam_toolbox_node', 'slam'),
    ('cartographer_occupancy_grid_node', 'slam_extra'),
    ('cartographer_node', 'slam'),
    ('ekf_node', 'ekf'),
    ('imu_covariance_filter.py', 'imu_filter'),
    ('tf_filter.py', 'tf_filter'),
    ('trajectory_recorder.py', 'recorder'),
    ('ros2 bag play', 'bag_player'),
    ('ros2 launch', 'launcher'),
    ('ros2 topic hz', 'monitor'),
    ('resource_sampler.py', 'sampler'),
)


def classify(cmdline):
    for needle, role in ROLE_PATTERNS:
        if needle in cmdline:
            return role
    return None


def read_process(pid):
    """Return (cpu_ticks, rss_kb, cmdline) or None if the process vanished."""
    try:
        with open(f'/proc/{pid}/stat', 'rb') as handle:
            stat = handle.read().decode('utf-8', 'replace')
        # The command name is in parentheses and may contain spaces.
        fields = stat.rsplit(')', 1)[1].split()
        utime, stime = int(fields[11]), int(fields[12])
        with open(f'/proc/{pid}/statm', 'rb') as handle:
            statm = handle.read().split()
        rss_pages = int(statm[1])
    except (OSError, IndexError, ValueError):
        return None

    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as handle:
            cmdline = handle.read().replace(b'\0', b' ').decode('utf-8', 'replace')
    except OSError:
        return None

    return (utime + stime, rss_pages * PAGE_SIZE // 1024, cmdline.strip())


def cgroup_dir():
    try:
        with open('/proc/self/cgroup', 'r', encoding='utf-8') as handle:
            for line in handle:
                if line.startswith('0::'):
                    return os.path.join('/sys/fs/cgroup', line.strip()[3:])
    except OSError:
        pass
    return None


def main():
    output = 'resources.csv'
    interval = 0.5
    peak_file = None
    argv = sys.argv[1:]
    for index, item in enumerate(argv):
        if item == '--out' and index + 1 < len(argv):
            output = argv[index + 1]
        elif item == '--interval' and index + 1 < len(argv):
            interval = float(argv[index + 1])
        elif item == '--peak-file' and index + 1 < len(argv):
            peak_file = argv[index + 1]

    stop = {'flag': False}

    def request_stop(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    previous = {}
    started = time.monotonic()

    with open(output, 'w', encoding='utf-8') as handle:
        handle.write('elapsed_s,iso_time,role,pid,cpu_pct,rss_kb,command\n')
        handle.flush()
        while not stop['flag']:
            now = time.monotonic()
            rows = []
            total = {}
            for entry in os.scandir('/proc'):
                if not entry.name.isdigit():
                    continue
                pid = int(entry.name)
                data = read_process(pid)
                if data is None:
                    continue
                ticks, rss_kb, cmdline = data
                role = classify(cmdline)
                if role is None:
                    continue
                last = previous.get(pid)
                if last is None:
                    previous[pid] = (ticks, now)
                    continue
                last_ticks, last_time = last
                elapsed = now - last_time
                if elapsed <= 0:
                    continue
                cpu_pct = (ticks - last_ticks) / CLK_TCK / elapsed * 100.0
                previous[pid] = (ticks, now)
                total[role] = total.get(role, 0.0) + cpu_pct
                rows.append((role, pid, cpu_pct, rss_kb, cmdline))

            iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())
            for role, pid, cpu_pct, rss_kb, cmdline in rows:
                handle.write(
                    f'{now - started:.2f},{iso},{role},{pid},'
                    f'{cpu_pct:.2f},{rss_kb},"{cmdline[:200]}"\n'
                )
            if rows:
                handle.flush()
            time.sleep(interval)

    if peak_file:
        path = cgroup_dir()
        peak = None
        if path:
            peak_path = os.path.join(path, 'memory.peak')
            for _ in range(3):
                try:
                    with open(peak_path, 'r', encoding='utf-8') as handle:
                        peak = int(handle.read().strip())
                    break
                except (OSError, ValueError) as exc:
                    print(f'cgroup peak read failed ({peak_path}): {exc}', file=sys.stderr)
                    time.sleep(0.2)
        with open(peak_file, 'w', encoding='utf-8') as handle:
            handle.write(f'{peak if peak is not None else -1}\n')


if __name__ == '__main__':
    main()
