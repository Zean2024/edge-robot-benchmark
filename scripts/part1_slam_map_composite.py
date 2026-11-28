#!/usr/bin/env python3
"""Build a 2x2 composite of the four occupancy grids for Chapter 4.3."""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_pgm(path):
    with open(path, 'rb') as handle:
        magic = handle.readline().strip()
        line = handle.readline()
        while line.startswith(b'#'):
            line = handle.readline()
        width, height = (int(v) for v in line.split())
        handle.readline()
        data = np.frombuffer(handle.read(), dtype=np.uint8)
    return data.reshape((height, width))


def to_image(grid, occupied_thresh=0.65, free_thresh=0.196):
    """Render a PGM grid with the ROS map_server convention.

    The value is the occupancy probability encoded as
    ``p = (255 - value) / 255``. Both writers follow this convention but use
    different unknown values (nav2 map_saver: 205; Cartographer: 128), so the
    palette is derived from the yaml thresholds instead of exact pixel values.
    """
    p = (255.0 - grid.astype(float)) / 255.0
    image = np.zeros((*grid.shape, 3), dtype=float)
    image[p > occupied_thresh] = 0.0      # occupied -> black
    image[p < free_thresh] = 1.0         # free -> white
    unknown = ~((p > occupied_thresh) | (p < free_thresh))
    image[unknown] = 0.78                # unknown -> light grey
    return image


def read_yaml_thresholds(pgm_path):
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    occupied, free = 0.65, 0.196
    try:
        for line in open(yaml_path, encoding='utf-8'):
            if line.startswith('occupied_thresh:'):
                occupied = float(line.split(':')[1])
            elif line.startswith('free_thresh:'):
                free = float(line.split(':')[1])
    except OSError:
        pass
    return occupied, free


def crop_mapped(grid, margin_cells=20):
    """Crop to the bounding box of the mapped cells plus a small margin.

    The saved grids cover different physical extents (the Cartographer grid
    includes the whole submap set), so cropping to the mapped area makes the
    four panels show the same environment at a comparable scale.
    """
    p = (255.0 - grid.astype(float)) / 255.0
    mapped = (p > 0.65) | (p < 0.196)
    rows = np.where(mapped.any(axis=1))[0]
    cols = np.where(mapped.any(axis=0))[0]
    if rows.size == 0 or cols.size == 0:
        return grid
    return grid[max(rows.min() - margin_cells, 0):min(rows.max() + margin_cells + 1, grid.shape[0]),
                max(cols.min() - margin_cells, 0):min(cols.max() + margin_cells + 1, grid.shape[1])]


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else 'results/part1_slam/maps'
    out = sys.argv[2] if len(sys.argv) > 2 else 'results/part1_slam/evaluation/figures/fig_map_composite.pdf'
    entries = [
        ('slam_toolbox_no_imu_parking1', 'SLAM Toolbox, LiDAR only'),
        ('slam_toolbox_imu_parking1', 'SLAM Toolbox, LiDAR + IMU'),
        ('cartographer_no_imu_parking1', 'Cartographer, LiDAR only'),
        ('cartographer_imu_parking1', 'Cartographer, LiDAR + IMU'),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.6))
    for axis, (name, title) in zip(axes.ravel(), entries):
        path = os.path.join(base, name, 'map.pgm')
        if not os.path.exists(path):
            axis.set_axis_off()
            axis.set_title(title + ' (missing)', fontsize=7)
            continue
        grid = crop_mapped(read_pgm(path))
        occupied, free = read_yaml_thresholds(path)
        axis.imshow(to_image(grid, occupied, free), origin='lower', interpolation='nearest')
        axis.set_title(title, fontsize=8)
        axis.set_xticks([])
        axis.set_yticks([])
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    print('wrote', out)


if __name__ == '__main__':
    main()
