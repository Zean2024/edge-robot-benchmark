#!/usr/bin/env python3
"""Evaluate one run against ground truth with evo (ATE + RPE).

Policy (decision D7): trajectories are projected onto the x-y plane and
aligned with a planar rigid transform (SE(2)). Both trajectories are reduced
to planar poses before alignment, so roll, pitch and z cannot inflate the
error, and every configuration is measured with exactly the same settings.

Outputs metrics.json next to the run directory.
"""

import argparse
import json
import math
import os
import sys

import numpy as np
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

WXYZ = 'wxyz'


def to_planar(trajectory):
    """Project a trajectory to the x-y plane with yaw-only orientations."""
    positions = np.array(trajectory.positions_xyz, dtype=float)
    quats = np.array(trajectory.orientations_quat_wxyz, dtype=float)
    timestamps = np.array(trajectory.timestamps, dtype=float)

    w, x, y, z = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    planar_quats = np.stack(
        [np.cos(yaw / 2.0), np.zeros_like(yaw), np.zeros_like(yaw), np.sin(yaw / 2.0)],
        axis=1,
    )
    planar_positions = positions.copy()
    planar_positions[:, 2] = 0.0

    return PoseTrajectory3D(
        positions_xyz=planar_positions,
        orientations_quat_wxyz=planar_quats,
        timestamps=timestamps,
    )


def stats_dict(statistics, count):
    return {
        'rmse': float(statistics['rmse']),
        'mean': float(statistics['mean']),
        'median': float(statistics['median']),
        'std': float(statistics['std']),
        'min': float(statistics['min']),
        'max': float(statistics['max']),
        'n_pairs': int(count),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', required=True, help='ground-truth TUM file')
    parser.add_argument('--est', required=True, help='estimated TUM file')
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--max-diff', type=float, default=0.05,
                        help='maximum timestamp difference for association [s]')
    parser.add_argument('--rpe-delta', type=float, default=1.0)
    parser.add_argument('--rpe-unit', default='meters',
                        choices=['meters', 'seconds', 'frames'])
    parser.add_argument('--all-pairs', action='store_true', default=True)
    args = parser.parse_args()

    trajectory_ref = file_interface.read_tum_trajectory_file(args.gt)
    trajectory_est = file_interface.read_tum_trajectory_file(args.est)

    trajectory_ref, trajectory_est = sync.associate_trajectories(
        trajectory_ref, trajectory_est, max_diff=args.max_diff
    )
    if trajectory_ref.num_poses < 10:
        print(f'ERROR: only {trajectory_ref.num_poses} associated poses', file=sys.stderr)
        return 1

    trajectory_ref = to_planar(trajectory_ref)
    trajectory_est = to_planar(trajectory_est)

    # SE(2) alignment: planar projection + rigid Umeyama, no scale.
    trajectory_est.align(trajectory_ref, correct_scale=False)

    data = (trajectory_ref, trajectory_est)

    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data(data)
    ate = stats_dict(ape_metric.get_all_statistics(), ape_metric.error.size)

    unit = {
        'meters': metrics.Unit.meters,
        'seconds': metrics.Unit.seconds,
        'frames': metrics.Unit.frames,
    }[args.rpe_unit]
    rpe_metric = metrics.RPE(
        metrics.PoseRelation.translation_part,
        args.rpe_delta,
        unit,
        all_pairs=args.all_pairs,
    )
    rpe_metric.process_data(data)
    rpe = stats_dict(rpe_metric.get_all_statistics(), rpe_metric.error.size)

    path_length = float(trajectory_est.path_length)
    gt_path_length = float(trajectory_ref.path_length)
    output = {
        'gt': args.gt,
        'est': args.est,
        'alignment': 'SE(2) planar, Umeyama, scale-free',
        'rpe': {'delta': args.rpe_delta, 'unit': args.rpe_unit},
        'associated_poses': int(trajectory_ref.num_poses),
        'path_length_m': round(path_length, 3),
        'gt_path_length_m': round(gt_path_length, 3),
        'ate': ate,
        'rpe_translation': rpe,
    }

    os.makedirs(args.run_dir, exist_ok=True)
    with open(os.path.join(args.run_dir, 'metrics.json'), 'w', encoding='utf-8') as handle:
        json.dump(output, handle, indent=2)

    print(
        f"ATE RMSE {ate['rmse']:.3f} m | "
        f"RPE RMSE {rpe['rmse']:.3f} m | "
        f"{output['associated_poses']} poses | path {path_length:.1f}/{gt_path_length:.1f} m"
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
