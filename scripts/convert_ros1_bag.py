#!/usr/bin/env python3
"""Convert a ROS 1 Ouster + MicroStrain bag into a ROS 2 bag that LIO-SAM can play.

Why this exists
  * The Ouster's own IMU (/os_cloud_node/imu) has no orientation (all-zero quaternion),
    which LIO-SAM rejects ("Invalid quaternion, please use a 9-axis IMU!").
  * The MicroStrain IMU (/imu/data) has orientation, but it reports in a NED world /
    FRD body frame and there is no TF linking it to the lidar.

What it does
  1. Calibration pass: pairs MicroStrain and Ouster-IMU gyro samples by timestamp and
     solves (Kabsch/SVD) for the rotation R that maps MicroStrain body axes into the
     lidar (os_sensor) axes. The Ouster IMU shares os_sensor axes per /tf_static.
  2. Write pass: writes
        /points_raw   <- /os_cloud_node/points  ('ambient' field renamed to 'noise')
        /imu_correct  <- /imu/data rotated into os_sensor axes, orientation in ENU
     so LIO-SAM runs with identity extrinsics (config/params_ouster64.yaml).

Usage: convert_ros1_bag.py INPUT.bag [-o OUTPUT_DIR] [--calibrate-only]
Prints "PROGRESS <pct>" lines so the GUI can show progress.
"""
import argparse
import os
import sys
import time

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

CLOUD_IN, IMU_IN, OS_IMU = "/os_cloud_node/points", "/imu/data", "/os_cloud_node/imu"
CLOUD_OUT, IMU_OUT = "/points_raw", "/imu_correct"

# ENU world <- NED world (proper rotation, det = +1)
R_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def log(msg):
    print(msg, flush=True)


def quat_to_mat(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return x, y, z, w


def open_reader(path):
    ts1 = get_typestore(Stores.ROS1_NOETIC)
    reader = Reader(path)
    reader.open()
    for c in reader.connections:
        try:
            ts1.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        except Exception:
            pass
    return reader, ts1


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def calibrate(reader, ts1):
    """Return (R lidar <- MicroStrain body, Ouster clock offset ns) from gyro correlation.

    The Ouster stamps with sensor uptime while the MicroStrain uses wall time, so the
    Ouster offset is estimated coarsely from bag receive times, then refined by
    cross-correlating the gyro magnitudes of the two IMUs.
    """
    conns = [c for c in reader.connections if c.topic in (IMU_IN, OS_IMU)]
    ms_t, ms_w, os_t, os_w, os_bag = [], [], [], [], []
    total = reader.duration or 1
    last = 0
    for c, t, raw in reader.messages(connections=conns):
        m = ts1.deserialize_ros1(raw, c.msgtype)
        w = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        if c.topic == IMU_IN:
            ms_t.append(stamp_ns(m.header)); ms_w.append(w)
        else:
            os_t.append(stamp_ns(m.header)); os_w.append(w); os_bag.append(t)
        pct = int(50 * (t - reader.start_time) / total)
        if pct != last:
            log(f"PROGRESS {pct}"); last = pct
    ms_t, ms_w = np.array(ms_t, dtype=np.int64), np.array(ms_w)
    os_t, os_w = np.array(os_t, dtype=np.int64), np.array(os_w)
    order = np.argsort(os_t); os_t, os_w = os_t[order], os_w[order]
    os_bag = np.array(os_bag, dtype=np.int64)[order]

    # Coarse offset from bag receive time, then refine with gyro-magnitude correlation.
    coarse = int(np.median(os_bag - os_t))
    ms_rel = (ms_t - ms_t[0]) / 1e9
    ms_n = np.linalg.norm(ms_w, axis=1)
    os_n = np.linalg.norm(os_w, axis=1)
    best, best_score = 0, -np.inf
    for shift_ms in range(-200, 201):
        shifted = (os_t + coarse + shift_ms * 1_000_000 - ms_t[0]) / 1e9
        interp = np.interp(ms_rel, shifted, os_n, left=np.nan, right=np.nan)
        ok = ~np.isnan(interp)
        if ok.sum() < 1000:
            continue
        score = np.corrcoef(ms_n[ok], interp[ok])[0, 1]
        if score > best_score:
            best, best_score = shift_ms, score
    offset = coarse + best * 1_000_000
    log(f"Ouster clock offset: {offset / 1e9:.3f} s (refined {best:+d} ms, gyro correlation {best_score:.3f})")
    os_t = os_t + offset

    idx = np.clip(np.searchsorted(os_t, ms_t), 1, len(os_t) - 1)
    idx = np.where(np.abs(os_t[idx - 1] - ms_t) < np.abs(os_t[idx] - ms_t), idx - 1, idx)
    close = np.abs(os_t[idx] - ms_t) < 5_000_000  # within 5 ms
    a, b = ms_w[close], os_w[idx[close]]
    moving = np.linalg.norm(a, axis=1) > 0.15
    a, b = a[moving], b[moving]
    if len(a) < 200:
        raise RuntimeError(f"Not enough moving IMU samples to calibrate ({len(a)}).")
    a = a - a.mean(0); b = b - b.mean(0)  # removes constant gyro bias
    u, _, vt = np.linalg.svd(b.T @ a)
    d = np.sign(np.linalg.det(u @ vt))
    R = u @ np.diag([1, 1, d]) @ vt
    resid = np.linalg.norm(b - a @ R.T, axis=1)
    log(f"Calibration: {len(a)} paired samples, median gyro residual {np.median(resid):.4f} rad/s")
    rpy = np.degrees([np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(R[2, 0]), np.arctan2(R[1, 0], R[0, 0])])
    log("R lidar<-imu =\n" + np.array2string(R, precision=4, suppress_small=True))
    log(f"  (roll, pitch, yaw) = ({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}) deg")
    return R, offset


def convert(reader, ts1, out_dir, R, offset):
    ts2 = get_typestore(Stores.ROS2_JAZZY)
    Header = ts2.types["std_msgs/msg/Header"]
    Time = ts2.types["builtin_interfaces/msg/Time"]
    PC2 = ts2.types["sensor_msgs/msg/PointCloud2"]
    PF = ts2.types["sensor_msgs/msg/PointField"]
    Imu = ts2.types["sensor_msgs/msg/Imu"]
    Vec3 = ts2.types["geometry_msgs/msg/Vector3"]
    Quat = ts2.types["geometry_msgs/msg/Quaternion"]
    R_body_lidar = R.T

    def hdr(h, frame, shift=0):
        ns = stamp_ns(h) + shift
        return Header(stamp=Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000), frame_id=frame)

    def rot_cov(cov):
        c = np.asarray(cov, dtype=np.float64).reshape(3, 3)
        return (R @ c @ R.T).reshape(9) if c[0, 0] >= 0 else np.asarray(cov, dtype=np.float64)

    conns = [c for c in reader.connections if c.topic in (CLOUD_IN, IMU_IN)]
    total = reader.duration or 1
    last = 0
    n_cloud = n_imu = 0
    with Writer(out_dir, version=8) as w:
        cc = w.add_connection(CLOUD_OUT, PC2.__msgtype__, typestore=ts2)
        ic = w.add_connection(IMU_OUT, Imu.__msgtype__, typestore=ts2)
        for c, t, raw in reader.messages(connections=conns):
            m = ts1.deserialize_ros1(raw, c.msgtype)
            if c.topic == CLOUD_IN:
                fields = [PF(name="noise" if f.name == "ambient" else f.name, offset=f.offset,
                             datatype=f.datatype, count=f.count) for f in m.fields]
                out = PC2(header=hdr(m.header, m.header.frame_id, offset), height=m.height, width=m.width,
                          fields=fields, is_bigendian=m.is_bigendian, point_step=m.point_step,
                          row_step=m.row_step, data=m.data, is_dense=m.is_dense)
                w.write(cc, t, ts2.serialize_cdr(out, PC2.__msgtype__))
                n_cloud += 1
            else:
                acc = R @ [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
                gyr = R @ [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                q = m.orientation
                # This driver's quaternion is world-in-body (inverse of the ROS convention):
                # gravity agrees with the accelerometer to ~3 deg only after transposing.
                q_ned_body = quat_to_mat(q.x, q.y, q.z, q.w).T
                qx, qy, qz, qw = mat_to_quat(R_ENU_NED @ q_ned_body @ R_body_lidar)
                out = Imu(header=hdr(m.header, "os_sensor"),
                          orientation=Quat(x=qx, y=qy, z=qz, w=qw),
                          orientation_covariance=np.asarray(m.orientation_covariance, dtype=np.float64),
                          angular_velocity=Vec3(x=gyr[0], y=gyr[1], z=gyr[2]),
                          angular_velocity_covariance=rot_cov(m.angular_velocity_covariance),
                          linear_acceleration=Vec3(x=acc[0], y=acc[1], z=acc[2]),
                          linear_acceleration_covariance=rot_cov(m.linear_acceleration_covariance))
                w.write(ic, t, ts2.serialize_cdr(out, Imu.__msgtype__))
                n_imu += 1
                if n_imu == 50:  # sanity check: gravity from orientation vs accelerometer
                    up_from_q = (R_ENU_NED @ q_ned_body @ R_body_lidar).T @ [0, 0, 1]
                    up_from_acc = acc / np.linalg.norm(acc)
                    ang = np.degrees(np.arccos(np.clip(up_from_q @ up_from_acc, -1, 1)))
                    log(f"Check: gravity from orientation vs accelerometer differ by {ang:.1f} deg")
            pct = 50 + int(50 * (t - reader.start_time) / total)
            if pct != last:
                log(f"PROGRESS {pct}"); last = pct
    log(f"Wrote {n_cloud} clouds and {n_imu} IMU messages to {out_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("-o", "--output", help="output ROS 2 bag dir (default: <bag>_ros2 next to input)")
    ap.add_argument("--calibrate-only", action="store_true")
    args = ap.parse_args()

    out_dir = args.output or os.path.splitext(args.bag)[0] + "_ros2"
    if not args.calibrate_only and os.path.exists(out_dir):
        log(f"ERROR: output already exists: {out_dir}")
        return 2
    t0 = time.time()
    reader, ts1 = open_reader(args.bag)
    try:
        topics = {c.topic for c in reader.connections}
        missing = {CLOUD_IN, IMU_IN, OS_IMU} - topics
        if missing:
            log(f"ERROR: bag is missing topics: {', '.join(sorted(missing))}")
            return 2
        log(f"Input: {args.bag} ({reader.duration / 1e9:.0f} s)")
        R, offset = calibrate(reader, ts1)
        if not args.calibrate_only:
            convert(reader, ts1, out_dir, R, offset)
    finally:
        reader.close()
    log(f"PROGRESS 100\nDone in {(time.time() - t0) / 60:.1f} min")
    return 0


if __name__ == "__main__":
    sys.exit(main())
