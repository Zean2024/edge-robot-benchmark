#!/usr/bin/env python3
"""Convert an M2DGR-plus ground-truth file to TUM format.

M2DGR-plus ships ground truth as
    timestamp x y z qw qx qy qz
TUM expects
    timestamp x y z qx qy qz qw

The scripts keeps the original timestamps (the bag recorded with --clock uses
the same epoch) and only reorders the quaternion columns, so no time
re-alignment is introduced here.
"""

import argparse
import math
import os
import sys


def convert(source, output):
    rows = []
    with open(source, encoding='utf-8', errors='replace') as handle:
        for line in handle:
            parts = line.split()
            if len(parts) != 8:
                continue
            try:
                values = [float(value) for value in parts]
            except ValueError:
                continue
            rows.append(values)

    rows.sort(key=lambda row: row[0])

    kept = []
    previous = None
    for timestamp, x, y, z, qw, qx, qy, qz in rows:
        if previous is not None and timestamp <= previous:
            continue
        previous = timestamp
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if norm < 1e-6:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        else:
            qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm
        kept.append((timestamp, x, y, z, qx, qy, qz, qw))

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, 'w', encoding='utf-8') as handle:
        for timestamp, x, y, z, qx, qy, qz, qw in kept:
            handle.write(
                f'{timestamp:.9f} {x:.9f} {y:.9f} {z:.9f} '
                f'{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n'
            )

    if kept:
        duration = kept[-1][0] - kept[0][0]
        print(
            f'{os.path.basename(source)} -> {output}: {len(kept)} poses, '
            f'{duration:.2f} s, start {kept[0][0]:.3f}'
        )
    else:
        print(f'ERROR: no usable rows in {source}', file=sys.stderr)
        return 1
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    return convert(args.source, args.output)


if __name__ == '__main__':
    sys.exit(main())
