#!/usr/bin/env python3
"""Georeference a saved LIO-SAM map using GNSS fixes from the original bag.

Matches each LIO-SAM keyframe (transformations.pcd, which carries timestamps) to the GNSS
position at the same time, fits a rigid 3D transform (rotation + translation, no scale)
map -> UTM, rejects GNSS outliers, and writes GlobalMap_<EPSG>.las with the CRS embedded.

Heights are GNSS ellipsoidal heights (WGS84), not orthometric/NAVD88.
Accuracy is limited by the GNSS: a plain (non-RTK) fix is ~1 m.

Usage: georeference_map.py MAP_DIR BAG [--gnss-topic /gnss1/fix] [--epsg auto]
  BAG may be a ROS 1 .bag or a ROS 2 bag folder containing a NavSatFix topic.
"""
import argparse
import os
import sys

import laspy
import numpy as np
from pyproj import CRS, Transformer
from rosbags.highlevel import AnyReader

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_to_las import read_pcd  # noqa: E402


def read_gnss(bag, topic):
    rows = []
    with AnyReader([__import__("pathlib").Path(bag)]) as r:
        conns = [c for c in r.connections if c.topic == topic]
        if not conns:
            raise SystemExit(f"ERROR: {topic} not found in {bag}")
        for c, _t, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            if m.status.status < 0:  # no fix
                continue
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            rows.append((t, m.latitude, m.longitude, m.altitude))
    return np.array(rows)


def kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((dst - cd).T @ (src - cs))
    d = np.sign(np.linalg.det(u @ vt))
    R = u @ np.diag([1, 1, d]) @ vt
    return R, cd - R @ cs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_dir")
    ap.add_argument("bag")
    ap.add_argument("--gnss-topic", default="/gnss1/fix")
    ap.add_argument("--epsg", default="auto", help="target EPSG code, default: WGS84 UTM zone of the data")
    args = ap.parse_args()

    poses = read_pcd(os.path.join(args.map_dir, "transformations.pcd"))
    kf_t = poses["time"].astype(np.float64)
    kf_xyz = np.stack([poses["x"], poses["y"], poses["z"]], 1).astype(np.float64)

    print(f"Reading GNSS {args.gnss_topic} from {args.bag} ...", flush=True)
    g = read_gnss(args.bag, args.gnss_topic)
    if args.epsg == "auto":
        zone = int((g[:, 2].mean() + 180) // 6) + 1
        epsg = (32600 if g[:, 1].mean() >= 0 else 32700) + zone
    else:
        epsg = int(args.epsg)
    crs = CRS.from_epsg(epsg)
    to_utm = Transformer.from_crs(4326, crs, always_xy=True)
    e, n = to_utm.transform(g[:, 2], g[:, 1])
    gnss = np.stack([e, n, g[:, 3]], 1)

    inside = (kf_t >= g[0, 0]) & (kf_t <= g[-1, 0])
    if inside.sum() < 10:
        raise SystemExit("ERROR: keyframe times do not overlap the GNSS times (clock mismatch?).")
    src = kf_xyz[inside]
    dst = np.stack([np.interp(kf_t[inside], g[:, 0], gnss[:, i]) for i in range(3)], 1)

    keep = np.ones(len(src), bool)
    for _ in range(3):  # fit, drop outliers, refit
        R, T = kabsch(src[keep], dst[keep])
        res = np.linalg.norm(dst - (src @ R.T + T), axis=1)
        keep = res < max(3 * np.median(res[keep]), 0.5)
    R, T = kabsch(src[keep], dst[keep])
    res = np.linalg.norm(dst - (src @ R.T + T), axis=1)[keep]
    horiz = np.linalg.norm((dst - (src @ R.T + T))[keep, :2], axis=1)
    print(f"Fit on {keep.sum()}/{len(src)} keyframes -> {crs.name} (EPSG:{epsg})")
    print(f"  residual 3D: median {np.median(res):.2f} m, 95% {np.percentile(res, 95):.2f} m;"
          f" horizontal median {np.median(horiz):.2f} m")

    pts = read_pcd(os.path.join(args.map_dir, "GlobalMap.pcd"))
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
    good = np.isfinite(xyz).all(1)
    xyz = xyz[good] @ R.T + T

    hdr = laspy.LasHeader(point_format=1, version="1.4")
    hdr.scales = np.array([0.001, 0.001, 0.001])
    hdr.offsets = np.floor(xyz.min(0))
    hdr.add_crs(crs)
    las = laspy.LasData(hdr)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if "intensity" in pts.dtype.names:
        las.intensity = np.clip(np.nan_to_num(pts["intensity"][good]), 0, 65535).astype(np.uint16)
    out = os.path.join(args.map_dir, f"GlobalMap_EPSG{epsg}.las")
    las.write(out)

    traj = os.path.join(args.map_dir, f"trajectory_EPSG{epsg}.csv")
    kf_utm = kf_xyz @ R.T + T
    lon, lat = Transformer.from_crs(crs, 4326, always_xy=True).transform(kf_utm[:, 0], kf_utm[:, 1])
    np.savetxt(traj, np.column_stack([kf_t, kf_utm, lat, lon]), delimiter=",", fmt="%.9f",
               header="time,easting,northing,ell_height,lat,lon", comments="")
    np.savetxt(os.path.join(args.map_dir, "map_to_utm.txt"), np.vstack([np.column_stack([R, T])]),
               header=f"3x4 [R|t], map -> EPSG:{epsg}  (p_utm = R @ p_map + t)", fmt="%.9f")
    print(f"Wrote {len(xyz):,} points to {out}")
    print(f"Wrote {traj}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
