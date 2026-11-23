#!/usr/bin/env python3
"""Convert an M2DGR-plus ground-truth file to TUM in a local metric frame.

M2DGR-plus ships ground truth as
    timestamp x y z qw qx qy qz
and the Parking sequences are expressed in ECEF coordinates (the position
norm is about 6.37e6 m), while the Anomaly sequence is already in a local
frame. TUM expects
    timestamp x y z qx qy qz qw

If the positions are ECEF, they are rotated into a local east-north-up frame
anchored at the first pose, so that the x-y plane is the local horizontal
plane and a planar trajectory comparison is geometrically meaningful. The
timestamps are preserved: the bag recorded with --clock uses the same epoch,
so no time re-alignment is introduced here.
"""

import argparse
import math
import os
import sys

A = 6378137.0
E2 = 6.69437999014e-3


def ecef_to_lla(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - E2))
    for _ in range(6):
        n = A / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - E2 * n / (n + h)))
    n = A / math.sqrt(1.0 - E2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return lat, lon, h


def ecef_to_enu(rows, anchor):
    lat0, lon0, _ = ecef_to_lla(*anchor)
    sin_lat0, cos_lat0 = math.sin(lat0), math.cos(lat0)
    sin_lon0, cos_lon0 = math.sin(lon0), math.cos(lon0)
    converted = []
    for timestamp, x, y, z, qw, qx, qy, qz in rows:
        dx, dy, dz = x - anchor[0], y - anchor[1], z - anchor[2]
        east = -sin_lon0 * dx + cos_lon0 * dy
        north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
        up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
        converted.append((timestamp, east, north, up, qw, qx, qy, qz))
    return converted


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
        kept.append([timestamp, x, y, z, qw, qx, qy, qz])

    if not kept:
        print(f'ERROR: no usable rows in {source}', file=sys.stderr)
        return 1

    norm0 = math.sqrt(kept[0][1] ** 2 + kept[0][2] ** 2 + kept[0][3] ** 2)
    frame = 'local'
    if norm0 > 1.0e6:
        frame = 'ecef->enu'
        kept = ecef_to_enu(kept, kept[0][1:4])

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, 'w', encoding='utf-8') as handle:
        for timestamp, x, y, z, qw, qx, qy, qz in kept:
            handle.write(
                f'{timestamp:.9f} {x:.9f} {y:.9f} {z:.9f} '
                f'{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n'
            )

    duration = kept[-1][0] - kept[0][0]
    print(
        f'{os.path.basename(source)} -> {output}: {len(kept)} poses, '
        f'{duration:.2f} s, frame {frame}'
    )
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    return convert(args.source, args.output)


if __name__ == '__main__':
    sys.exit(main())
