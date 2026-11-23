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


def to_image(grid):
    image = np.zeros((*grid.shape, 3), dtype=float)
    image[grid == 0] = 0.0
    image[grid == 205] = 0.78
    image[grid == 254] = 1.0
    other = ~np.isin(grid, (0, 205, 254))
    image[other] = 0.5
    return image


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
        grid = read_pgm(path)
        axis.imshow(to_image(grid), origin='lower', interpolation='nearest')
        axis.set_title(title, fontsize=8)
        axis.set_xticks([])
        axis.set_yticks([])
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    print('wrote', out)


if __name__ == '__main__':
    main()
