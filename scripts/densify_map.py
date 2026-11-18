#!/usr/bin/env python3
"""Build a dense point cloud from every raw scan using LIO-SAM's optimized trajectory.

LIO-SAM's GlobalMap only keeps feature points from keyframes. This replays every scan in
the bag, places each point with the keyframe poses interpolated to that point's own
timestamp (Ouster per-point 't' field, which also deskews the scan), and voxel-averages
the result to the requested spacing. Output is georeferenced if map_to_utm.txt exists
(run georeference_map.py first).

Usage: densify_map.py MAP_DIR ROS2_BAG [--voxel 0.02] [--min-range 1.0] [--max-range 60]
                      [--epsg 32617] [--tmp DIR]
Prints "PROGRESS <pct>" lines.
"""
import argparse
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import laspy
import numpy as np
from pyproj import CRS
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_to_las import read_pcd  # noqa: E402

PF_DTYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
TILE = 50.0          # metres, on-disk bucket size
TIME_BINS = 32       # pose samples per scan for deskewing
REC = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<u2")])


def log(msg):
    print(msg, flush=True)


def voxel_keys(xyz, voxel):
    """One int64 per voxel (21 bits per axis, ~42 km range at 2 cm)."""
    k = np.floor(xyz / voxel).astype(np.int64)
    k -= k.min(0)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def cloud_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name); formats.append(PF_DTYPES[f.datatype]); offsets.append(f.offset)
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_dir")
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/points_raw")
    ap.add_argument("--voxel", type=float, default=0.02)
    ap.add_argument("--min-range", type=float, default=1.0)
    ap.add_argument("--max-range", type=float, default=60.0,
                    help="far points are sparse and noisy; default 60 m")
    ap.add_argument("--epsg", type=int, default=None, help="default: read from map_to_utm.txt")
    ap.add_argument("--tmp", default=None, help="temp dir for tile buckets")
    ap.add_argument("--max-scans", type=int, default=0, help="stop after N scans (testing)")
    ap.add_argument("--output", default=None, help="output LAS path (default: in MAP_DIR)")
    args = ap.parse_args()
    t0 = time.time()

    # --- keyframe trajectory -------------------------------------------------------
    kf = read_pcd(os.path.join(args.map_dir, "transformations.pcd"))
    kf_t = kf["time"].astype(np.float64)
    order = np.argsort(kf_t)
    kf_t = kf_t[order]
    kf_p = np.stack([kf["x"], kf["y"], kf["z"]], 1).astype(np.float64)[order]
    # pcl::getTransformation: R = Rz(yaw) Ry(pitch) Rx(roll)
    kf_r = Rotation.from_euler("ZYX", np.stack([kf["yaw"], kf["pitch"], kf["roll"]], 1)[order].astype(np.float64))
    slerp = Slerp(kf_t, kf_r)

    def poses_at(ts):
        ts = np.clip(ts, kf_t[0], kf_t[-1])
        p = np.stack([np.interp(ts, kf_t, kf_p[:, i]) for i in range(3)], 1)
        return slerp(ts).as_matrix(), p

    # --- optional georeference -----------------------------------------------------
    geo = os.path.join(args.map_dir, "map_to_utm.txt")
    R_utm, T_utm, epsg = np.eye(3), np.zeros(3), args.epsg
    if os.path.exists(geo):
        with open(geo) as f:
            first = f.readline()
        m = np.loadtxt(geo)
        R_utm, T_utm = m[:, :3], m[:, 3]
        if epsg is None and "EPSG:" in first:
            epsg = int(first.split("EPSG:")[1].split()[0])
    # Output coordinates are large (UTM); tile/voxel in map frame, transform at write time.

    tmp = tempfile.mkdtemp(prefix="densify_", dir=args.tmp)
    buckets = {}
    n_in = n_kept = n_scans = 0
    try:
        # --- pass 1: transform every scan into the map frame, bucket by tile --------
        with AnyReader([Path(args.bag)]) as r:
            conns = [c for c in r.connections if c.topic == args.topic]
            if not conns:
                raise SystemExit(f"ERROR: {args.topic} not in {args.bag}")
            total = sum(c.msgcount for c in conns)
            last = -1
            dt = None
            for c, _t, raw in r.messages(connections=conns):
                if args.max_scans and n_scans >= args.max_scans:
                    break
                msg = r.deserialize(raw, c.msgtype)
                n_scans += 1
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if stamp < kf_t[0] - 0.2 or stamp > kf_t[-1] + 0.2:
                    continue  # outside the optimized trajectory
                if dt is None:
                    dt = cloud_dtype(msg)
                pts = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
                xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
                rng = np.linalg.norm(xyz, axis=1)
                ok = np.isfinite(rng) & (rng > args.min_range) & (rng < args.max_range)
                n_in += len(pts)
                if not ok.any():
                    continue
                xyz = xyz[ok]
                inten = np.clip(pts["intensity"][ok], 0, 65535).astype(np.uint16)
                pt_t = stamp + pts["t"][ok].astype(np.float64) * 1e-9 if "t" in dt.names else np.full(len(xyz), stamp)

                # deskew: sample poses at TIME_BINS instants across the scan
                t_lo, t_hi = pt_t.min(), pt_t.max()
                edges = np.linspace(t_lo, t_hi + 1e-9, TIME_BINS + 1)
                bin_idx = np.clip(np.searchsorted(edges, pt_t, side="right") - 1, 0, TIME_BINS - 1)
                Rs, Ps = poses_at(0.5 * (edges[:-1] + edges[1:]))
                world = np.einsum("nij,nj->ni", Rs[bin_idx], xyz) + Ps[bin_idx]

                # per-scan voxel thinning to cut temp volume (final averaging happens later)
                _, first = np.unique(voxel_keys(world, args.voxel), return_index=True)
                world, inten = world[first], inten[first]

                tiles = np.floor(world[:, :2] / TILE).astype(np.int64)
                tkey = tiles[:, 0] * 100003 + tiles[:, 1]
                srt = np.argsort(tkey, kind="stable")
                tkey, world, inten = tkey[srt], world[srt], inten[srt]
                cuts = np.flatnonzero(np.diff(tkey)) + 1
                for s, e in zip(np.r_[0, cuts], np.r_[cuts, len(tkey)]):
                    k = int(tkey[s])
                    rec = np.empty(e - s, REC)
                    rec["x"], rec["y"], rec["z"] = world[s:e, 0], world[s:e, 1], world[s:e, 2]
                    rec["i"] = inten[s:e]
                    if k not in buckets:
                        buckets[k] = open(os.path.join(tmp, f"{k}.bin"), "ab")
                    buckets[k].write(rec.tobytes())
                n_kept += len(world)

                pct = int(80 * n_scans / total)
                if pct != last:
                    log(f"PROGRESS {pct}"); last = pct
        for f in buckets.values():
            f.close()
        log(f"Pass 1: {n_scans} scans, {n_in:,} raw points -> {n_kept:,} after range filter + per-scan thinning "
            f"({(time.time() - t0) / 60:.1f} min)")

        # --- pass 2: voxel-average each tile and stream into one LAS ----------------
        suffix = f"_EPSG{epsg}" if epsg else ""
        cm = f"{args.voxel * 100:g}cm"
        out = args.output or os.path.join(args.map_dir, f"GlobalMap_dense_{cm}{suffix}.las")
        hdr = laspy.LasHeader(point_format=1, version="1.4")
        hdr.scales = np.array([0.001, 0.001, 0.001])
        # bound for offsets: transform keyframe positions (good enough; offset only sets precision origin)
        hdr.offsets = np.floor((kf_p @ R_utm.T + T_utm).min(0)) - 1000
        if epsg:
            hdr.add_crs(CRS.from_epsg(epsg))
        n_out = 0
        files = sorted(os.listdir(tmp))
        with laspy.open(out, mode="w", header=hdr) as w:
            for j, fn in enumerate(files):
                rec = np.fromfile(os.path.join(tmp, fn), dtype=REC)
                xyz = np.stack([rec["x"], rec["y"], rec["z"]], 1).astype(np.float64)
                _, inv, cnt = np.unique(voxel_keys(xyz, args.voxel), return_inverse=True, return_counts=True)
                mean = np.stack([np.bincount(inv, xyz[:, i]) for i in range(3)], 1) / cnt[:, None]
                mi = (np.bincount(inv, rec["i"].astype(np.float64)) / cnt).astype(np.uint16)
                g = mean @ R_utm.T + T_utm
                pr = laspy.ScaleAwarePointRecord.zeros(len(g), header=hdr)
                pr.x, pr.y, pr.z = g[:, 0], g[:, 1], g[:, 2]
                pr.intensity = mi
                w.write_points(pr)
                n_out += len(g)
                os.remove(os.path.join(tmp, fn))
                log(f"PROGRESS {80 + int(20 * (j + 1) / len(files))}")
        log(f"Wrote {n_out:,} points at {cm} spacing to {out}")
        log(f"Done in {(time.time() - t0) / 60:.1f} min")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
