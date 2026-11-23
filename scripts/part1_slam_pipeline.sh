#!/usr/bin/env bash
# ------------------------------------------------------------------------------
# part1_slam_pipeline.sh - one entry point for the whole Part 1 pipeline.
#
#   usage: part1_slam_pipeline.sh [options]
#
#   --repeats N        real-time repeats per cell (default 3)
#   --fresh            archive the current run tree before starting
#   --skip-build       do not rebuild the Docker image
#   --skip-suite       reuse the existing real-time runs
#   --skip-stress      do not run the 2x stress pass
#   --skip-maps        do not regenerate the occupancy grids
#   --skip-thesis      do not recompile the thesis
#   --datasets "a b"   override the dataset list
#   --experiments "a b" override the configuration list
#
# Environment:
#   PART1_IMAGE          image name (default part1:jazzy)
#   PART1_DATA_ROOT      bag root on the host (default ~/data/M2DGR-plus)
#   PART1_GT_SOURCE_ROOT ground-truth source directory
#                        (default /mnt/e/Data/01-Study/LUT/BachelorThesis/dataset/M2DGR-plus)
#   THESIS_REPO          thesis repository (default ../Master-s-thesis-Part-1-...)
#   PART1_GT_CONVERT     1 = regenerate the ground-truth TUM files (default 1)
# ------------------------------------------------------------------------------
set -uo pipefail

HERE="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
cd "$REPO" || exit 1

IMAGE="${PART1_IMAGE:-part1:jazzy}"
DATA_ROOT="${PART1_DATA_ROOT:-$HOME/data/M2DGR-plus}"
GT_SOURCE="${PART1_GT_SOURCE_ROOT:-/mnt/e/Data/01-Study/LUT/BachelorThesis/dataset/M2DGR-plus}"
THESIS="${THESIS_REPO:-$REPO/../Master-s-thesis-Part-1-SLAM-algorithm-benchmarking}"
RESULTS="$REPO/results/part1_slam/runs"
REPEATS=3
FRESH=0
SKIP_BUILD=0
SKIP_SUITE=0
SKIP_STRESS=0
SKIP_MAPS=0
SKIP_THESIS=0
CONVERT_GT="${PART1_GT_CONVERT:-1}"
DATASETS="anomaly_ros2 parking1"
EXPERIMENTS="slam_toolbox_no_imu slam_toolbox_imu cartographer_no_imu cartographer_imu"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --repeats) REPEATS="$2"; shift 2 ;;
        --fresh) FRESH=1; shift ;;
        --skip-build) SKIP_BUILD=1; shift ;;
        --skip-suite) SKIP_SUITE=1; shift ;;
        --skip-stress) SKIP_STRESS=1; shift ;;
        --skip-maps) SKIP_MAPS=1; shift ;;
        --skip-thesis) SKIP_THESIS=1; shift ;;
        --datasets) DATASETS="$2"; shift 2 ;;
        --experiments) EXPERIMENTS="$2"; shift 2 ;;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

log() { echo "[$(date +%H:%M:%S)] $*"; }

# ---------------------------------------------------------------- preconditions
log "checking prerequisites"
command -v docker >/dev/null || { echo "docker not found" >&2; exit 1; }
docker info >/dev/null 2>&1 || { echo "docker daemon not reachable" >&2; exit 1; }
[[ -d "$DATA_ROOT" ]] || { echo "bag root not found: $DATA_ROOT" >&2; exit 1; }
RAW_ROOT="${PART1_RAW_BAG_ROOT:-$GT_SOURCE}"
for dataset in $DATASETS; do
    if [[ ! -d "$DATA_ROOT/$dataset" ]]; then
        case "$dataset" in
            anomaly_ros2) raw_name=anomaly.bag ;;
            parking1)     raw_name=parking1.bag ;;
            *)            raw_name="$dataset.bag" ;;
        esac
        if [[ -f "$RAW_ROOT/$raw_name" ]]; then
            log "converting $RAW_ROOT/$raw_name -> $DATA_ROOT/$dataset"
            bash scripts/part1_slam_convert_bag.sh "$RAW_ROOT/$raw_name" \
                "$DATA_ROOT/$dataset" || { echo "bag conversion failed" >&2; exit 1; }
        else
            echo "missing bag: $DATA_ROOT/$dataset (and $RAW_ROOT/$raw_name)" >&2
            exit 1
        fi
    fi
done

# --------------------------------------------------------------- ground truth
if [[ "$CONVERT_GT" == "1" ]]; then
    log "converting ground truth from $GT_SOURCE"
    for dataset in $DATASETS; do
        case "$dataset" in
            anomaly_ros2) source_name=anomaly ;;
            parking1)     source_name=parking1 ;;
            *)            source_name="$dataset" ;;
        esac
        if [[ -f "$GT_SOURCE/$source_name.txt" ]]; then
            ~/.venvs/part1/bin/python scripts/part1_slam_prepare_groundtruth.py \
                --source "$GT_SOURCE/$source_name.txt" \
                --output "datasets/groundtruth/M2DGR-plus/$source_name.tum"
        else
            log "WARNING: $GT_SOURCE/$source_name.txt not found, keeping the existing TUM file"
        fi
    done
fi

# --------------------------------------------------------------------- image
if [[ "$SKIP_BUILD" != "1" ]]; then
    log "building $IMAGE (cached layers make this fast after the first build)"
    docker build -t "$IMAGE" -f docker/Dockerfile . || { echo "image build failed" >&2; exit 1; }
fi
docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo "image $IMAGE missing" >&2; exit 1; }

# ------------------------------------------------------------- fresh run tree
if [[ "$FRESH" == "1" && -d "$RESULTS" ]]; then
    archive="$REPO/results/_archive_$(date +%Y%m%d_%H%M%S)"
    log "archiving the existing run tree to $archive"
    mkdir -p "$archive"
    mv "$RESULTS"/* "$archive"/ 2>/dev/null || true
fi
mkdir -p "$RESULTS"

# ---------------------------------------------------------------- real-time
if [[ "$SKIP_SUITE" != "1" ]]; then
    log "real-time suite: $EXPERIMENTS x [$DATASETS] x $REPEATS inside Docker"
    PART1_EXEC=docker PART1_SKIP_RTF=1 PART1_REPEATS="$REPEATS" \
    PART1_DATASETS="$DATASETS" PART1_EXPERIMENTS="$EXPERIMENTS" \
        bash scripts/part1_slam_run_all.sh || { echo "suite failed" >&2; exit 1; }
else
    log "real-time suite skipped"
fi

# ------------------------------------------------------------------ stress
if [[ "$SKIP_STRESS" != "1" ]]; then
    log "stress pass at twice the recorded rate"
    PART1_RUNNER=docker/run_experiment.sh \
        PART1_DATASETS="$DATASETS" PART1_EXPERIMENTS="$EXPERIMENTS" \
        bash scripts/part1_slam_rate_pass.sh || log "WARNING: stress pass reported a failure"
else
    log "stress pass skipped"
fi

# -------------------------------------------------------------------- maps
if [[ "$SKIP_MAPS" != "1" ]]; then
    log "occupancy grids inside the container"
    docker run --rm --network host --cpus=1 --memory=1g --memory-swap=1g \
        -v "$REPO":/repo \
        -v "$DATA_ROOT":/data:ro \
        -e PART1_DATA_ROOT=/data \
        "$IMAGE" bash /repo/scripts/part1_slam_map_pass.sh parking1 \
        || log "WARNING: map pass reported a failure"
else
    log "map pass skipped"
fi

# ----------------------------------------------------- aggregation and figures
log "aggregation, figures and thesis assets"
~/.venvs/part1/bin/python scripts/part1_slam_aggregate.py || exit 1
~/.venvs/part1/bin/python scripts/part1_slam_plot_trajectories.py || exit 1
~/.venvs/part1/bin/python scripts/part1_slam_map_composite.py \
    results/part1_slam/maps results/part1_slam/evaluation/figures/fig_map_composite.pdf || true
bash scripts/part1_slam_thesis_assets.sh >/dev/null || true
if [[ -f results/part1_slam/evaluation/figures/fig_map_composite.pdf && -d "$THESIS/figs" ]]; then
    cp -f results/part1_slam/evaluation/figures/fig_map_composite.pdf "$THESIS/figs/"
fi

# --------------------------------------------------------------------- thesis
if [[ "$SKIP_THESIS" != "1" ]]; then
    if command -v latexmk >/dev/null 2>&1 || [[ -x "$HOME/.TinyTeX/bin/x86_64-linux/latexmk" ]]; then
        log "compiling the thesis"
        export PATH="$HOME/.TinyTeX/bin/x86_64-linux:$PATH"
        ( cd "$THESIS" && rm -rf build_tex && mkdir -p build_tex \
          && latexmk -pdf -interaction=nonstopmode -file-line-error -outdir=build_tex main.tex > compile.log 2>&1 ) \
          || log "WARNING: thesis compile reported errors, see $THESIS/compile.log"
        if [[ -f "$THESIS/build_tex/main.pdf" ]]; then
            cp -f "$THESIS/build_tex/main.pdf" \
                  "$(cd "$REPO/.." && pwd)/thesis_draft.pdf"
            log "thesis copied to $(cd "$REPO/.." && pwd)/thesis_draft.pdf"
        fi
    else
        log "WARNING: latexmk not found, thesis not compiled"
    fi
else
    log "thesis compile skipped"
fi

log "pipeline finished"
log "runs:      $RESULTS"
log "metrics:   $REPO/results/part1_slam/evaluation/aggregate/"
log "figures:   $REPO/results/part1_slam/evaluation/figures/"
