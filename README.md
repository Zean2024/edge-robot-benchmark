# Edge Robot Benchmark

A resource-constrained benchmarking platform for edge robotics.

- Part 1 (LUT): 2D LiDAR SLAM versus 2D LiDAR + IMU SLAM under a 1 CPU / 1 GB
  budget. Part 2 (HEBUT): edge AI / vision extension.

## Prerequisites

- Docker Engine inside the WSL 2 distribution (or any Linux host).
- A Python virtual environment at `~/.venvs/part1` with `evo`, `numpy`,
  `matplotlib`, `pandas` and `rosbags` (the evaluation runs on the host; the
  experiments run inside the container).
- Optional: TinyTeX (`~/.TinyTeX`) for compiling the thesis.

## Dataset layout

```text
~/data/M2DGR-plus/
├── anomaly_ros2/     # ROS 2 bag directory (sqlite3)
└── parking1/         # ROS 2 bag directory
```

Override the root with `PART1_DATA_ROOT`. Raw ROS 1 bags are converted with
`scripts/part1_slam_convert_bag.sh <source.bag> <destination>`, which keeps only
`/scan`, `/tf`, `/odom` and `/imu`.

## One command

The full pipeline is orchestrated by a single script:

```bash
bash scripts/part1_slam_pipeline.sh            # everything
bash scripts/part1_slam_pipeline.sh --help     # options
```

It converts the ground truth, builds the image, runs the real-time suite inside
the container, runs the 2x stress pass, regenerates the maps, aggregates the
tables and figures, copies the thesis assets and compiles the PDF. Steps can be
skipped with `--skip-build`, `--skip-suite`, `--skip-stress`, `--skip-maps` and
`--skip-thesis`; `--fresh` archives the previous run tree; `--repeats N` sets
the number of real-time repeats.

## Individual steps

```bash
# build the image once
docker build -t part1:jazzy -f docker/Dockerfile .

# one run inside the 1 CPU / 1 GB container
PART1_PLAY_RATE=1.0 bash docker/run_experiment.sh slam_toolbox_no_imu anomaly_ros2 r1

# the real-time suite (4 configs x 2 datasets x 3 repeats) and evaluation
PART1_EXEC=docker PART1_SKIP_RTF=1 PART1_REPEATS=3 bash scripts/part1_slam_run_all.sh

# stress pass at twice the recorded rate
PART1_RUNNER=docker/run_experiment.sh bash scripts/part1_slam_rate_pass.sh

# occupancy grids
docker run --rm --network host --cpus=1 --memory=1g --memory-swap=1g \
  -v "$PWD":/repo -v "$HOME/data/M2DGR-plus":/data:ro -e PART1_DATA_ROOT=/data \
  part1:jazzy bash /repo/scripts/part1_slam_map_pass.sh parking1

# evaluate one run against ground truth
~/.venvs/part1/bin/python scripts/part1_slam_evaluate.py \
  --gt datasets/groundtruth/M2DGR-plus/anomaly.tum \
  --est results/part1_slam/runs/<experiment>/<dataset>/<run>/trajectory.tum \
  --run-dir results/part1_slam/runs/<experiment>/<dataset>/<run> --max-diff 0.12

# aggregate, plot and copy assets into the thesis repository
~/.venvs/part1/bin/python scripts/part1_slam_aggregate.py
~/.venvs/part1/bin/python scripts/part1_slam_plot_trajectories.py
bash scripts/part1_slam_thesis_assets.sh
```

## Resource budget

Every capture runs in a container started with
`--cpus=1 --memory=1g --memory-swap=1g --network host`. The limits are the
cgroup v2 CPU quota and memory ceiling (`cpu.max`, `memory.max`,
`memory.swap.max`), the same controllers a host-side `systemd-run` scope uses;
that scope is retained as a fallback through
`scripts/part1_slam_run_limited.sh`.

## Where results land

- `results/part1_slam/runs/<experiment>/<dataset>/run_*/` - immutable raw runs
  (command, git revision, config hashes, bag info, logs, `resources.csv`,
  `trajectory.tum`, `summary.json`, `metrics.json`).
- `results/part1_slam/evaluation/aggregate/` - `per_run.csv`, `aggregate.json`,
  LaTeX tables.
- `results/part1_slam/evaluation/figures/` - thesis-ready PDF figures.
