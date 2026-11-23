#!/usr/bin/env bash
# Copy the generated tables and figures into the thesis repository so the
# LaTeX sources reference stable, versioned assets.
set -euo pipefail

REPO="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/.." && pwd)"
THESIS="${THESIS_REPO:-$REPO/../Master-s-thesis-Part-1-SLAM-algorithm-benchmarking}"
EVAL="$REPO/results/part1_slam/evaluation"

mkdir -p "$THESIS/tables" "$THESIS/figs"

cp -f "$EVAL/aggregate/table_ate_rpe.tex" "$THESIS/tables/"
cp -f "$EVAL/aggregate/table_resources.tex" "$THESIS/tables/"

for figure in fig_ate.pdf fig_rpe.pdf fig_cpu.pdf fig_memory.pdf; do
    [[ -f "$EVAL/figures/$figure" ]] && cp -f "$EVAL/figures/$figure" "$THESIS/figs/"
done
for figure in "$EVAL"/figures/fig_trajectory_*.pdf; do
    [[ -f "$figure" ]] && cp -f "$figure" "$THESIS/figs/"
done

# Map images for the qualitative section, if the map pass has been executed.
if [[ -d "$REPO/results/part1_slam/maps" ]]; then
    for map_dir in "$REPO"/results/part1_slam/maps/*/; do
        name="$(basename "$map_dir")"
        [[ -f "$map_dir/map.pgm" ]] && cp -f "$map_dir/map.pgm" "$THESIS/figs/map_${name}.pgm"
    done
fi

echo "Thesis assets:"
ls -la "$THESIS/figs" | tail -12
ls -la "$THESIS/tables"
