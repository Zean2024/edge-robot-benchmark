# Edge Robot Benchmark

A resource-constrained benchmarking platform for edge robotics.

- Part 1 (LUT): 2D LiDAR SLAM versus 2D LiDAR + IMU SLAM under a 1 CPU / 1 GB budget. Part 2 (HEBUT): edge AI / vision extension.

## Prerequisites

- Ubuntu 24.04 (WSL 2 with systemd enabled) and ROS 2 Jazzy Jalisco.
- SLAM Toolbox, Cartographer, and `robot_localization` installed and sourced (`/opt/ros/jazzy/setup.bash`).
- A Python virtual environment at `~/.venvs/part1` containing `evo`, `numpy`, and `matplotlib`.
- A user systemd session. Docker is **not** required: the resource budget is enforced with a cgroup v2 user scope (decision D10).

## Dataset layout

Raw ROS 2 bags live outside the repository:

```text
~/data/M2DGR-plus/
├── anomaly_ros2/     # ROS 2 bag directory
└── parking1/         # ROS 2 bag directory
```

Override the root with `PART1_DATA_ROOT`. M2DGR-plus also supplies the ground-truth text files (`<sequence>.txt`) alongside the bags.

## Ground truth conversion

M2DGR-plus stores ground truth as `timestamp x y z qw qx qy qz`; the evaluator expects TUM (`timestamp x y z qx qy qz qw`):

```bash
python3 scripts/part1_slam_prepare_groundtruth.py \
    --source ~/data/M2DGR-plus/parking1.txt \
    --output datasets/groundtruth/M2DGR-plus/parking1.tum
```
Repeat for the Anomaly sequence. The converted files are committed under `datasets/groundtruth/M2DGR-plus/`.

## One run

Run one configuration and dataset inside the 1 CPU / 1 GB scope:

```bash
PART1_PLAY_RATE=1.0 bash scripts/part1_slam_run_limited.sh \
    slam_toolbox_no_imu anomaly_ros2 r1
```
Each run produces a self-describing directory under
`results/part1_slam/runs/<experiment>/<dataset>/run_YYYYMMDD_HHMMSS_r1/`
(command, git revision, config hashes, bag info, logs, `resources.csv`,
`trajectory.tum`, `rtf.json`, and `summary.json`; the evaluator later adds
`metrics.json`).

## Full suite

```bash
bash scripts/part1_slam_run_all.sh
```

The driver runs 4 configurations × 2 datasets × 3 real-time repeats, plus one
unthrottled pass per cell for RTF (32 runs), evaluates every run, and aggregates
at the end. Useful environment overrides: `PART1_REPEATS`, `PART1_SKIP_RTF`,
`PART1_DATASETS`, `PART1_EXPERIMENTS`, `PART1_RESULTS_ROOT`.

## Evaluation

Evaluate one run against ground truth with the fixed D7 settings:

```bash
~/.venvs/part1/bin/python scripts/part1_slam_evaluate.py \
    --gt datasets/groundtruth/M2DGR-plus/anomaly.tum \
    --est results/part1_slam/runs/slam_toolbox_no_imu/anomaly_ros2/<run>/trajectory.tum \
    --run-dir results/part1_slam/runs/slam_toolbox_no_imu/anomaly_ros2/<run>
```

## Aggregation

```bash
~/.venvs/part1/bin/python scripts/part1_slam_aggregate.py \
    --runs-root results/part1_slam/runs \
    --out-root results/part1_slam/evaluation
```

## Where results land

- `results/part1_slam/runs/<experiment>/<dataset>/run_*/` — immutable raw runs.
- `results/part1_slam/evaluation/aggregate/` — `per_run.csv`, `aggregate.json`, LaTeX tables.
- `results/part1_slam/evaluation/figures/` — thesis-ready PDF figures.

The resource limit is applied by `scripts/part1_slam_run_limited.sh` through
`systemd-run --user --scope --collect -p CPUQuota=100% -p MemoryMax=1G -p MemorySwapMax=0`,
the same kernel controllers that `docker run --cpus=1 --memory=1g` configures.
