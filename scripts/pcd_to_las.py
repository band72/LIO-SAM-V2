#!/usr/bin/env python3
"""Convert a LIO-SAM PCD map (x y z [intensity], binary or ascii) to LAS/LAZ with laspy.

No PDAL needed. Output format follows the extension: .las or .laz.
Usage: pcd_to_las.py INPUT.pcd OUTPUT.las|OUTPUT.laz
"""
import sys

import laspy
import numpy as np

PCD_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
             ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}


def read_pcd(path):
    header = {}
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            line = line.decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            key, *vals = line.split()
            header[key.upper()] = vals
            if key.upper() == "DATA":
                break
        fields = header["FIELDS"]
        sizes = [int(s) for s in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(c) for c in header.get("COUNT", ["1"] * len(fields))]
        n = int(header["POINTS"][0])
        dtype = np.dtype([(name, PCD_TYPES[(t, s)]) if c == 1 else (name, PCD_TYPES[(t, s)], (c,))
                          for name, s, t, c in zip(fields, sizes, types, counts)])
        mode = header["DATA"][0].lower()
        if mode == "binary":
            data = np.fromfile(f, dtype=dtype, count=n)
        elif mode == "ascii":
            data = np.loadtxt(f, dtype=dtype, max_rows=n)
        else:
            raise ValueError(f"Unsupported PCD DATA mode: {mode} (save as binary or ascii)")
    return data


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    src, dst = sys.argv[1], sys.argv[2]
    pts = read_pcd(src)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    good = np.isfinite(xyz).all(axis=1)
    xyz = xyz[good]

    hdr = laspy.LasHeader(point_format=1, version="1.2")
    hdr.scales = np.array([0.001, 0.001, 0.001])  # millimetre resolution
    hdr.offsets = np.floor(xyz.min(axis=0))
    las = laspy.LasData(hdr)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if "intensity" in pts.dtype.names:
        inten = pts["intensity"][good].astype(np.float64)
        las.intensity = np.clip(np.nan_to_num(inten), 0, 65535).astype(np.uint16)
    las.write(dst)
    ext = xyz.max(axis=0) - xyz.min(axis=0)
    print(f"Wrote {len(xyz):,} points to {dst} (extent {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
