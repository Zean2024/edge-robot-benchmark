#!/usr/bin/env python3
"""Aggregate per-run metrics into thesis tables and figures.

Reads every run directory that contains both summary.json and metrics.json,
then writes:
    evaluation/aggregate/per_run.csv
    evaluation/aggregate/table_ate_rpe.tex
    evaluation/aggregate/table_resources.tex
    evaluation/figures/*.pdf

Only runs that were executed by part1_slam_capture_run.sh are considered, so
every number can be traced back to a run directory.
"""

import argparse
import csv
import json
import math
import os
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

EXPERIMENT_ORDER = (
    'slam_toolbox_no_imu',
    'slam_toolbox_imu',
    'cartographer_no_imu',
    'cartographer_imu',
)
EXPERIMENT_LABELS = {
    'slam_toolbox_no_imu': 'SLAM Toolbox (LiDAR only)',
    'slam_toolbox_imu': 'SLAM Toolbox (LiDAR + IMU)',
    'cartographer_no_imu': 'Cartographer (LiDAR only)',
    'cartographer_imu': 'Cartographer (LiDAR + IMU)',
}
DATASET_LABELS = {
    'anomaly_ros2': 'Anomaly',
    'parking1': 'Parking',
}


def load_records(runs_root):
    records = []
    for experiment in sorted(os.listdir(runs_root)):
        experiment_dir = os.path.join(runs_root, experiment)
        if not os.path.isdir(experiment_dir):
            continue
        for dataset in sorted(os.listdir(experiment_dir)):
            dataset_dir = os.path.join(experiment_dir, dataset)
            if not os.path.isdir(dataset_dir):
                continue
            for run in sorted(os.listdir(dataset_dir)):
                run_dir = os.path.join(dataset_dir, run)
                summary_path = os.path.join(run_dir, 'summary.json')
                metrics_path = os.path.join(run_dir, 'metrics.json')
                if not (os.path.exists(summary_path) and os.path.exists(metrics_path)):
                    continue
                summary = json.load(open(summary_path, encoding='utf-8'))
                metrics = json.load(open(metrics_path, encoding='utf-8'))
                try:
                    play_rate = float(summary.get('play_rate', 1.0))
                except (TypeError, ValueError):
                    play_rate = 1.0
                record = {
                    'experiment': experiment,
                    'dataset': dataset,
                    'run': run,
                    'mode': 'unthrottled' if play_rate > 10 else 'realtime',
                    'ate_rmse': metrics['ate']['rmse'],
                    'ate_mean': metrics['ate']['mean'],
                    'rpe_rmse': metrics['rpe_translation']['rmse'],
                    'cpu_mean': summary['algorithm_cpu_pct']['mean'],
                    'cpu_p95': summary['algorithm_cpu_pct']['p95'],
                    'mem_mean_mb': summary['algorithm_memory_mb']['mean'],
                    'mem_max_mb': summary['algorithm_memory_mb']['max'],
                    'cgroup_peak_mb': summary.get('cgroup_peak_memory_mb', 0.0),
                    'scan_ratio': summary.get('scan_forward_ratio', 0.0),
                    'throughput': summary.get('throughput_ratio', 0.0),
                    'rtf': summary.get('rtf', 0.0),
                    'bag_duration': summary.get('bag_duration_s', 0.0),
                    'wall_time': summary.get('wall_time_s', 0.0),
                    'poses': metrics.get('associated_poses', 0),
                }
                records.append(record)
    return records


def mean_std(values):
    if not values:
        return 0.0, 0.0
    if len(values) == 1:
        return values[0], 0.0
    return statistics.mean(values), statistics.stdev(values)


def fmt(value, std, digits=3):
    return f'{value:.{digits}f} $\\pm$ {std:.{digits}f}'


def latex_table(rows, path, caption, label, columns):
    alt = f'{caption} Values are the mean and one standard deviation over the repeats of each cell.'
    lines = [
        r'\begin{table}[!h]',
        r'\centering',
        rf'\caption{{{caption}}}',
        rf'\label{{{label}}}',
        r'\pdftooltip{%',
        r'\footnotesize',
        r'\begin{tabular}{ll' + 'c' * len(columns) + '}',
        r'\toprule',
        'Dataset & Configuration & ' + ' & '.join(label for _, label in columns) + r' \\',
        r'\midrule',
    ]
    for row in rows:
        cells = ' & '.join(fmt(row[key], row[key + '_std']) for key, _ in columns)
        lines.append(f"{row['dataset_label']} & {row['label']} & {cells} \\\\")
    lines += [
        r'\bottomrule',
        r'\end{tabular}' + '}' + '{' + alt + '}',
        r'\end{table}',
    ]
    open(path, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs-root', default='results/part1_slam/runs')
    parser.add_argument('--out-root', default='results/part1_slam/evaluation')
    args = parser.parse_args()

    records = load_records(args.runs_root)
    if not records:
        print('No complete runs found.')
        return 1

    os.makedirs(os.path.join(args.out_root, 'aggregate'), exist_ok=True)
    os.makedirs(os.path.join(args.out_root, 'figures'), exist_ok=True)

    per_run_path = os.path.join(args.out_root, 'aggregate', 'per_run.csv')
    with open(per_run_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)

    grouped = defaultdict(list)
    for record in records:
        grouped[(record['dataset'], record['experiment'])].append(record)

    rows = []
    for (dataset, experiment), items in sorted(grouped.items()):
        realtime = [item for item in items if item['mode'] == 'realtime'] or items
        unthrottled = [item for item in items if item['mode'] == 'unthrottled']
        row = {
            'dataset': dataset,
            'dataset_label': DATASET_LABELS.get(dataset, dataset),
            'experiment': experiment,
            'label': EXPERIMENT_LABELS.get(experiment, experiment),
            'n': len(realtime),
            'n_unthrottled': len(unthrottled),
        }
        for key in ('ate_rmse', 'rpe_rmse', 'cpu_mean', 'cpu_p95',
                    'mem_max_mb', 'mem_mean_mb', 'scan_ratio', 'throughput'):
            value, std = mean_std([item[key] for item in realtime])
            row[key] = value
            row[key + '_std'] = std
        rtf_value, rtf_std = mean_std([item['rtf'] for item in unthrottled])
        row['rtf'] = rtf_value
        row['rtf_std'] = rtf_std
        rows.append(row)

    latex_table(
        rows,
        os.path.join(args.out_root, 'aggregate', 'table_ate_rpe.tex'),
        'Trajectory accuracy per configuration (mean $\\pm$ standard deviation over repeats).',
        'tab:ate_rpe',
        [('ate_rmse', 'ATE RMSE [m]'), ('rpe_rmse', 'RPE RMSE [m]')],
    )
    latex_table(
        rows,
        os.path.join(args.out_root, 'aggregate', 'table_resources.tex'),
        'Computational cost per configuration (mean $\\pm$ standard deviation over repeats).',
        'tab:resources',
        [('cpu_mean', 'CPU [\\%]'), ('cpu_p95', 'CPU p95 [\\%]'),
         ('mem_max_mb', 'Peak RSS [MB]'), ('scan_ratio', 'Scan forward ratio')],
    )

    summary_json = os.path.join(args.out_root, 'aggregate', 'aggregate.json')
    with open(summary_json, 'w', encoding='utf-8') as handle:
        json.dump(rows, handle, indent=2)

    # ------------------------------------------------------------- figures
    datasets = sorted({row['dataset'] for row in rows},
                      key=lambda name: list(DATASET_LABELS).index(name)
                      if name in DATASET_LABELS else 99)

    def bar_figure(metric, ylabel, filename, title):
        fig, axes = plt.subplots(1, len(datasets), figsize=(5.2 * len(datasets), 3.6),
                                 squeeze=False)
        for index, dataset in enumerate(datasets):
            axis = axes[0][index]
            subset = [row for row in rows if row['dataset'] == dataset]
            subset.sort(key=lambda row: EXPERIMENT_ORDER.index(row['experiment']))
            positions = range(len(subset))
            axis.bar(positions, [row[metric] for row in subset],
                     yerr=[row[metric + '_std'] for row in subset],
                     color='#4c72b0', capsize=3)
            axis.set_xticks(list(positions))
            axis.set_xticklabels([row['label'].replace(' (', '\n(') for row in subset],
                                 fontsize=7, rotation=0)
            axis.set_ylabel(ylabel)
            axis.set_title(DATASET_LABELS.get(dataset, dataset), fontsize=9)
            axis.grid(axis='y', alpha=0.3)
        fig.suptitle(title, fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(args.out_root, 'figures', filename), dpi=200)
        plt.close(fig)

    bar_figure('ate_rmse', 'ATE RMSE [m]', 'fig_ate.pdf',
               'Absolute trajectory error under a 1 CPU / 1 GB budget')
    bar_figure('rpe_rmse', 'RPE RMSE [m]', 'fig_rpe.pdf',
               'Relative pose error under a 1 CPU / 1 GB budget')
    bar_figure('cpu_mean', 'CPU utilisation [% of one core]', 'fig_cpu.pdf',
               'Processor demand under a 1 CPU / 1 GB budget')
    bar_figure('mem_max_mb', 'Peak resident memory [MB]', 'fig_memory.pdf',
               'Memory demand under a 1 CPU / 1 GB budget')

    print(f'{len(records)} runs, {len(rows)} group rows -> {args.out_root}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
