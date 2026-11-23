#!/usr/bin/env python3
"""Render occupancy-grid PGM files as thesis-ready PDF figures.

Reads the P5 binary PGM written by map_saver.py (or by the Cartographer
pbstream converter), converts the grayscale grid to an image and saves a
cropped PDF with a metric scale bar.
"""

import argparse
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_pgm(path):
    with open(path, 'rb') as handle:
        magic = handle.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path}: not a binary PGM')
        line = handle.readline()
        while line.startswith(b'#'):
            line = handle.readline()
        width, height = (int(value) for value in line.split())
        maxval = int(handle.readline().strip())
        data = np.frombuffer(handle.read(), dtype=np.uint8)
    if maxval != 255:
        raise ValueError(f'{path}: unsupported maxval {maxval}')
    return data.reshape((height, width))


def read_yaml_resolution(yaml_path, default=0.05):
    try:
        for line in open(yaml_path, encoding='utf-8'):
            if line.startswith('resolution:'):
                return float(line.split(':')[1])
    except OSError:
        pass
    return default


def render(pgm_path, output_path):
    grid = read_pgm(pgm_path)
    resolution = read_yaml_resolution(
        os.path.splitext(pgm_path)[0] + '.yaml'
    )
    height, width = grid.shape
    image = np.zeros((height, width, 3), dtype=float)
    # 0 = occupied (black), 205 = unknown (grey), 254 = free (white).
    for value, colour in ((0, 0.0), (205, 0.75), (254, 1.0)):
        mask = grid == value
        image[mask] = colour
    image[~np.isin(grid, (0, 205, 254))] = 0.5

    fig, axis = plt.subplots(figsize=(5.2, 5.2 * height / max(width, 1)))
    extent = [0, width * resolution, 0, height * resolution]
    axis.imshow(image, extent=extent, origin='lower')
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_title(os.path.basename(os.path.dirname(pgm_path)), fontsize=8)
    scale = 10.0
    axis.plot([extent[1] * 0.05, extent[1] * 0.05 + scale],
              [extent[3] * 0.05, extent[3] * 0.05], 'k-', linewidth=2)
    axis.text(extent[1] * 0.05, extent[3] * 0.05 + 0.4, '10 m', fontsize=7)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
    print(f'{pgm_path} -> {output_path} ({width}x{height} cells, {resolution} m)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--maps-root', default='results/part1_slam/maps')
    parser.add_argument('--out-dir', default='results/part1_slam/evaluation/figures')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    for pgm in sorted(glob.glob(os.path.join(args.maps_root, '*', 'map.pgm'))):
        name = os.path.basename(os.path.dirname(pgm))
        render(pgm, os.path.join(args.out_dir, f'fig_map_{name}.pdf'))


if __name__ == '__main__':
    main()
