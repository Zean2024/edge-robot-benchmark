#!/usr/bin/env python3
"""Plot ground truth and estimated trajectories for the thesis.

For each dataset, one representative run per configuration is aligned with the
same SE(2) policy used by the evaluation and drawn over the ground truth.
"""

import argparse
import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

EXPERIMENTS = (
    'slam_toolbox_no_imu',
    'slam_toolbox_imu',
    'cartographer_no_imu',
    'cartographer_imu',
)
LABELS = {
    'slam_toolbox_no_imu': 'SLAM Toolbox (LiDAR only)',
    'slam_toolbox_imu': 'SLAM Toolbox (LiDAR + IMU)',
    'cartographer_no_imu': 'Cartographer (LiDAR only)',
    'cartographer_imu': 'Cartographer (LiDAR + IMU)',
}
DATASET_LABELS = {'anomaly_ros2': 'Anomaly', 'parking1': 'Parking'}


def planar(trajectory):
    positions = np.array(trajectory.positions_xyz, dtype=float)
    quats = np.array(trajectory.orientations_quat_wxyz, dtype=float)
    w, x, y, z = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    planar_quats = np.stack(
        [np.cos(yaw / 2.0), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2.0)],
        axis=1,
    )
    positions[:, 2] = 0.0
    return PoseTrajectory3D(
        positions_xyz=positions,
        orientations_quat_wxyz=planar_quats,
        timestamps=np.array(trajectory.timestamps, dtype=float),
    )


def pick_run(runs_root, experiment, dataset):
    pattern = os.path.join(runs_root, experiment, dataset, 'run_*')
    candidates = [
        path for path in sorted(glob.glob(pattern))
        if os.path.exists(os.path.join(path, 'metrics.json'))
    ]
    for path in candidates:
        summary_path = os.path.join(path, 'summary.json')
        if os.path.exists(summary_path):
            summary = json.load(open(summary_path, encoding='utf-8'))
            try:
                if float(summary.get('play_rate', 1.0)) > 10:
                    continue
            except (TypeError, ValueError):
                pass
        return path
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs-root', default='results/part1_slam/runs')
    parser.add_argument('--groundtruth-root', default='datasets/groundtruth/M2DGR-plus')
    parser.add_argument('--out-dir', default='results/part1_slam/evaluation/figures')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    for dataset in ('anomaly_ros2', 'parking1'):
        groundtruth_name = {'anomaly_ros2': 'anomaly', 'parking1': 'parking1'}.get(
            dataset, dataset
        )
        gt_path = os.path.join(args.groundtruth_root, f'{groundtruth_name}.tum')
        if not os.path.exists(gt_path):
            continue
        gt = file_interface.read_tum_trajectory_file(gt_path)
        gt_positions = np.array(gt.positions_xyz, dtype=float)

        fig, axis = plt.subplots(figsize=(5.4, 4.6))
        axis.plot(gt_positions[:, 0], gt_positions[:, 1], 'k--', linewidth=1.6,
                  label='Ground truth')

        for experiment in EXPERIMENTS:
            run_dir = pick_run(args.runs_root, experiment, dataset)
            if run_dir is None:
                continue
            estimate_path = os.path.join(run_dir, 'trajectory.tum')
            if not os.path.exists(estimate_path):
                continue
            estimate = file_interface.read_tum_trajectory_file(estimate_path)
            reference, aligned = sync.associate_trajectories(gt, estimate, max_diff=0.12)
            reference = planar(reference)
            aligned = planar(aligned)
            aligned.align(reference)
            positions = np.array(aligned.positions_xyz, dtype=float)
            axis.plot(positions[:, 0], positions[:, 1], linewidth=1.2,
                      label=LABELS[experiment])

        axis.set_aspect('equal', adjustable='datalim')
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.grid(alpha=0.3)
        axis.legend(fontsize=7, loc='best')
        axis.set_title(f'{DATASET_LABELS.get(dataset, dataset)} sequence', fontsize=9)
        fig.tight_layout()
        out_path = os.path.join(args.out_dir, f'fig_trajectory_{dataset}.pdf')
        fig.savefig(out_path, dpi=200)
        fig.savefig(out_path.replace('.pdf', '.png'), dpi=160)
        plt.close(fig)
        print(f'wrote {out_path}')


if __name__ == '__main__':
    main()
