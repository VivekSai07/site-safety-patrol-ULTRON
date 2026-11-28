#!/usr/bin/env python3
"""Characterize a rosbag2 recording without a ROS installation (pip package `rosbags`).

Reports per-topic counts, rates, gaps, frame_ids and header-vs-receive offsets, the
full TF tree (static values and dynamic edges with rates and motion), PointCloud2
layout and range statistics, IMU, CameraInfo, Image, RealSense extrinsics/metadata
and RoboSense packet facts. Writes `report.json` and `report.md` to --out.

Reports hold numbers and names only, but they are derived from private recordings:
keep them in an ignored/local folder unless reviewed.

    python3 ros2_ws/tools/bag_inspect.py /path/to/bag_dir --out runs/bag_inspect/<name>
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

# Message definitions missing from the Humble type store. Sources: RoboSense
# rslidar_msg (ROS 2 branch) and IntelRealSense realsense-ros realsense2_camera_msgs.
EXTRA_MSGS = {
    "rslidar_msg/msg/RslidarPacket": "std_msgs/Header header\nuint8 is_difop\nuint8 is_frame_begin\nuint8[] data\n",
    "realsense2_camera_msgs/msg/Extrinsics": "float64[9] rotation\nfloat64[3] translation\n",
    "realsense2_camera_msgs/msg/Metadata": "std_msgs/Header header\nstring json_data\n",
}
NEAR_RANGE_M = 1.0  # returns closer than this are summarised as candidate robot-body/self hits
HEAVY_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/PointCloud2"}
POINTFIELD_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
POINTFIELD_NAMES = {1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16", 5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64"}
ROBOSENSE_MSOP_MAGIC = {
    "55aa055a": "RS-Helios / RS-Ruby / RS-80/128 family (4-byte MSOP id)",
    "55aa050a": "RS-16/32/Bpearl family (8-byte MSOP id)",
}


def typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    store = get_typestore(Stores.ROS2_HUMBLE)
    for name, text in EXTRA_MSGS.items():
        if name not in store.types:
            store.register(get_types_from_msg(text, name))
    return store


def _stats(values):
    a = np.asarray(values, dtype=np.float64).ravel()
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    return {"n": int(a.size), "min": float(a.min()), "p5": float(np.percentile(a, 5)),
            "median": float(np.median(a)), "mean": float(a.mean()), "p95": float(np.percentile(a, 95)),
            "max": float(a.max())}


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _quat_angle(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2 * math.acos(min(1.0, dot))


def _transform_dict(t):
    tr, q = t.translation, t.rotation
    rpy = _quat_to_rpy(q.x, q.y, q.z, q.w)
    return {"translation_m": [tr.x, tr.y, tr.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w],
            "rpy_deg": [math.degrees(v) for v in rpy]}


def cloud_array(msg):
    """Structured numpy view of a PointCloud2 (scalar fields only)."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.count != 1 or f.datatype not in POINTFIELD_DTYPES:
            continue
        names.append(f.name)
        formats.append((">" if msg.is_bigendian else "<") + POINTFIELD_DTYPES[f.datatype])
        offsets.append(f.offset)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    n = msg.width * msg.height
    return np.frombuffer(bytes(msg.data[: n * msg.point_step]), dtype=dtype, count=n)


def _header_from_cdr(raw):
    """(stamp_s, frame_id) of a message that starts with std_msgs/Header, without full decoding."""
    sec, nsec, flen = struct.unpack_from("<iII", raw, 4)
    return sec + nsec * 1e-9, bytes(raw[16:16 + flen - 1]).decode(errors="replace")


class TopicAcc:
    def __init__(self, name, msgtype):
        self.name, self.msgtype = name, msgtype
        self.recv, self.header, self.sizes = [], [], []
        self.frame_ids = defaultdict(int)
        self.errors = 0
        self.error_text = None
        self.extra = {}

    def summary(self, gap_factor):
        r = np.asarray(self.recv, dtype=np.float64) * 1e-9
        out = {"type": self.msgtype, "count": len(r), "frame_ids": dict(self.frame_ids),
               "bytes_per_msg": _stats(self.sizes), "deserialize_errors": self.errors}
        if self.error_text:
            out["deserialize_error_text"] = self.error_text
        if len(r):
            out["first_recv_s"], out["last_recv_s"] = float(r[0]), float(r[-1])
        if len(r) > 1:
            dt = np.diff(r)
            span = r[-1] - r[0]
            med = float(np.median(dt))
            out["rate_hz_mean"] = (len(r) - 1) / span if span > 0 else None
            out["period_s"] = _stats(dt)
            big = dt > gap_factor * med if med > 0 else np.zeros(dt.shape, bool)
            out["gaps"] = {"threshold_s": gap_factor * med, "count": int(big.sum()), "max_s": float(dt.max()),
                           "at_s_from_start": [float(r[i] - r[0]) for i in np.nonzero(big)[0][:20]]}
        if self.header and len(self.header) == len(r):
            h = np.asarray(self.header, dtype=np.float64)
            nonzero = h > 0
            out["header_zero_stamps"] = int((~nonzero).sum())
            out["recv_minus_header_s"] = _stats((r - h)[nonzero])
            h = h[nonzero]
            if len(h) > 1:
                hd = np.diff(h)
                out["header_nonmonotonic"] = int((hd < 0).sum())
                out["header_rate_hz_mean"] = (len(h) - 1) / (h[-1] - h[0]) if h[-1] > h[0] else None
                med = float(np.median(hd))
                big = hd > gap_factor * med if med > 0 else np.zeros(hd.shape, bool)
                out["header_period_s"] = _stats(hd)
                out["header_gaps"] = {"threshold_s": gap_factor * med, "count": int(big.sum()),
                                      "max_s": float(hd.max()),
                                      "at_s_from_start": [float(h[i] - h[0]) for i in np.nonzero(big)[0][:20]]}
        out.update(self.extra)
        return out


class Inspector:
    def __init__(self, store, cloud_every=10, image_every=30):
        self.store = store
        self.cloud_every, self.image_every = max(1, cloud_every), max(1, image_every)
        self.topics: dict[str, TopicAcc] = {}
        self.tf_static, self.tf_dyn = {}, {}
        self.clouds = defaultdict(lambda: defaultdict(list))
        self.imu = defaultdict(lambda: {"acc": [], "gyro": [], "quat": [], "stamp": [], "cov": None})
        self.caminfo = defaultdict(list)
        self.images = defaultdict(lambda: defaultdict(list))
        self.lidar_pkts = defaultdict(lambda: {"difop": 0, "frame_begin": 0, "data_len": defaultdict(int),
                                                "magic": defaultdict(int), "block_hdr_ok": 0, "checked": 0,
                                                "difop_head": None})
        self.metadata_keys = {}

    def feed(self, topic, msgtype, recv_ns, raw):
        acc = self.topics.setdefault(topic, TopicAcc(topic, msgtype))
        acc.recv.append(recv_ns)
        acc.sizes.append(len(raw))
        if msgtype not in self.store.types:
            return
        if msgtype in HEAVY_TYPES:
            every = self.cloud_every if msgtype.endswith("PointCloud2") else self.image_every
            if (len(acc.recv) - 1) % every:
                try:
                    stamp, frame = _header_from_cdr(raw)
                    acc.header.append(stamp)
                    acc.frame_ids[frame] += 1
                except struct.error:
                    acc.errors += 1
                return
        try:
            msg = self.store.deserialize_cdr(raw, msgtype)
        except Exception as exc:  # noqa: BLE001 - mismatched definitions are reported, not fatal
            acc.errors += 1
            acc.error_text = f"{type(exc).__name__}: {exc}"[:200]
            return
        header = getattr(msg, "header", None)
        if header is not None:
            acc.header.append(_stamp(header))
            acc.frame_ids[header.frame_id] += 1
        handler = getattr(self, "_on_" + msgtype.split("/")[-1], None)
        if handler:
            handler(topic, msg)

    def _on_TFMessage(self, topic, msg):
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if topic == "/tf_static":
                self.tf_static[key] = _transform_dict(t.transform)
                continue
            q, tr = t.transform.rotation, t.transform.translation
            e = self.tf_dyn.setdefault(key, {"stamps": [], "xyz": [], "quat": []})
            e["stamps"].append(_stamp(t.header))
            e["xyz"].append((tr.x, tr.y, tr.z))
            e["quat"].append((q.x, q.y, q.z, q.w))

    def _on_PointCloud2(self, topic, msg):
        c = self.clouds[topic]
        if not c["layout"]:
            c["layout"].append({"fields": [{"name": f.name, "offset": f.offset, "count": f.count,
                                            "datatype": POINTFIELD_NAMES.get(f.datatype, f.datatype)} for f in msg.fields],
                                "point_step": msg.point_step, "is_bigendian": bool(msg.is_bigendian)})
        c["shape"].append((msg.height, msg.width))
        c["is_dense"].append(bool(msg.is_dense))
        arr = cloud_array(msg)
        names = set(arr.dtype.names)
        c["points"].append(len(arr))
        if {"x", "y", "z"} <= names:
            xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
            finite = np.isfinite(xyz).all(axis=1)
            c["nan_frac"].append(1.0 - float(finite.mean()) if len(xyz) else 0.0)
            c["valid_points"].append(int(finite.sum()))
            xyz = xyz[finite]
            rng = np.linalg.norm(xyz, axis=1)
            c["zero_frac"].append(float((rng == 0).mean()) if len(rng) else 0.0)
            c["ranges"].append(rng[rng > 0][::7])
            c["z"].append(xyz[rng > 0][::7, 2])
            c["xyz_sample"].append(xyz[::23])
            c["near"].append(xyz[(rng > 0) & (rng < NEAR_RANGE_M)])
        if "ring" in names:
            c["rings"].append(np.unique(arr["ring"]))
            if msg.height > 1:
                c["ring_per_row"].append(bool((arr["ring"].reshape(msg.height, msg.width) ==
                                               arr["ring"].reshape(msg.height, msg.width)[:, :1]).all()))
        for tname in ("timestamp", "time", "t"):
            if tname in names:
                tv = arr[tname].astype(np.float64)
                tv = tv[np.isfinite(tv) & (tv != 0)]
                c["time_field"] = [tname]
                if tv.size:
                    c["time_span"].append(float(tv.max() - tv.min()))
                    c["time_min_minus_header"].append(float(tv.min() - _stamp(msg.header)))
                    c["time_max_minus_header"].append(float(tv.max() - _stamp(msg.header)))
                break
        if "intensity" in names:
            c["intensity"].append(arr["intensity"][::50].astype(np.float64))

    def _on_Imu(self, topic, msg):
        d = self.imu[topic]
        a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        d["acc"].append((a.x, a.y, a.z))
        d["gyro"].append((g.x, g.y, g.z))
        d["quat"].append((q.x, q.y, q.z, q.w))
        d["stamp"].append(_stamp(msg.header))
        if d["cov"] is None:
            d["cov"] = {"orientation": [float(v) for v in msg.orientation_covariance],
                        "angular_velocity": [float(v) for v in msg.angular_velocity_covariance],
                        "linear_acceleration": [float(v) for v in msg.linear_acceleration_covariance]}

    def _on_CameraInfo(self, topic, msg):
        rec = {"width": msg.width, "height": msg.height, "distortion_model": msg.distortion_model,
               "d": [float(v) for v in msg.d], "k": [float(v) for v in msg.k], "p": [float(v) for v in msg.p],
               "r": [float(v) for v in msg.r], "binning": [msg.binning_x, msg.binning_y]}
        lst = self.caminfo[topic]
        if not lst or lst[-1]["rec"] != rec:
            lst.append({"rec": rec, "count": 1})
        else:
            lst[-1]["count"] += 1

    def _on_Image(self, topic, msg):
        d = self.images[topic]
        layout = (msg.encoding, msg.width, msg.height, msg.step, bool(msg.is_bigendian))
        if layout not in d["layout"]:
            d["layout"].append(layout)
        if msg.encoding in ("16UC1", "mono16"):
            img = np.frombuffer(bytes(msg.data), dtype=">u2" if msg.is_bigendian else "<u2")
            valid = img > 0
            d["depth_valid"].append(float(valid.mean()))
            if valid.any():
                d["depth_median_raw"].append(float(np.median(img[valid])))
                d["depth_p95_raw"].append(float(np.percentile(img[valid], 95)))
                d["depth_min_raw"].append(float(img[valid].min()))
        elif msg.encoding == "32FC1":
            img = np.frombuffer(bytes(msg.data), dtype="<f4")
            d["depth_valid"].append(float((np.isfinite(img) & (img > 0)).mean()))
        else:
            d["mean_intensity"].append(float(np.frombuffer(bytes(msg.data), np.uint8)[::97].mean()))

    def _on_Extrinsics(self, topic, msg):
        rot = np.asarray(msg.rotation, dtype=np.float64)
        cos = (np.trace(rot.reshape(3, 3)) - 1) / 2
        self.topics[topic].extra["extrinsics"] = {
            "rotation_col_major": rot.tolist(), "translation_m": [float(v) for v in msg.translation],
            "rotation_angle_deg": math.degrees(math.acos(max(-1.0, min(1.0, cos)))),
            "baseline_m": float(np.linalg.norm(msg.translation))}

    def _on_Metadata(self, topic, msg):
        if topic in self.metadata_keys:
            return
        try:
            js = json.loads(msg.json_data)
        except ValueError:
            js = {}
        skip = ("frame_number", "frame_timestamp", "sensor_timestamp", "hw_timestamp", "time_of_arrival",
                "backend_timestamp", "frame_counter")
        self.metadata_keys[topic] = {"keys": sorted(js),
                                     "first_values": {k: v for k, v in js.items()
                                                      if isinstance(v, (int, float, str)) and k not in skip}}

    def _on_RslidarPacket(self, topic, msg):
        d = self.lidar_pkts[topic]
        data = bytes(msg.data)
        d["data_len"][len(data)] += 1
        d["difop"] += int(msg.is_difop)
        d["frame_begin"] += int(msg.is_frame_begin)
        if msg.is_difop:
            if d["difop_head"] is None:
                d["difop_head"] = data[:8].hex()
            return
        d["magic"][data[:4].hex()] += 1
        if len(data) == 1248 and data[42:44] == b"\xff\xee":
            # Helios-style MSOP: 42-byte header then 12 blocks of 100 bytes (0xFFEE, azimuth BE 0.01 deg, 32 x 3 B).
            az = [int.from_bytes(data[44 + 100 * b: 46 + 100 * b], "big") for b in range(12)]
            d.setdefault("az_first", []).append(az[0])
            d.setdefault("az_step", []).append(((az[-1] - az[0]) % 36000) / 11)
            if d["checked"] < 2000:
                d["checked"] += 1
                if all(data[42 + 100 * b: 44 + 100 * b] == b"\xff\xee" for b in range(12)):
                    d["block_hdr_ok"] += 1

    # ---------------------------------------------------------------- report
    @staticmethod
    def _packet_coverage(d):
        """Azimuth resolution and how much of each revolution the recorded MSOP packets cover."""
        if not d.get("az_first"):
            return {}
        step = float(np.median(d["az_step"])) / 100.0
        az = np.asarray(d["az_first"], dtype=np.float64) / 100.0
        jumps = np.diff(az) % 360.0
        per_packet = 12 * step
        revolutions = float(jumps.sum() / 360.0)
        expected = revolutions * 360.0 / per_packet if per_packet > 0 else None
        return {"azimuth_step_deg": step, "packets_per_rev_expected": 360.0 / per_packet if per_packet else None,
                "revolutions_spanned": revolutions,
                "msop_packets_recorded_vs_expected": (len(az) / expected) if expected else None,
                "azimuth_jump_gt_2_packets": int((jumps > 2.5 * per_packet).sum())}

    def _tf_report(self):
        static = [{"parent": p, "child": c, **v} for (p, c), v in sorted(self.tf_static.items())]
        dyn = []
        for (p, c), e in sorted(self.tf_dyn.items()):
            s, xyz, quat = np.asarray(e["stamps"]), np.asarray(e["xyz"]), e["quat"]
            rpy = np.degrees(np.array([_quat_to_rpy(*q) for q in quat]))
            yaw = np.unwrap(np.radians(rpy[:, 2]))
            dyn.append({"parent": p, "child": c, "count": len(s),
                        "rate_hz": (len(s) - 1) / (s[-1] - s[0]) if len(s) > 1 and s[-1] > s[0] else None,
                        "translation_constant": bool(np.ptp(xyz, axis=0).max() < 1e-9),
                        "translation_first_m": xyz[0].tolist(), "translation_last_m": xyz[-1].tolist(),
                        "path_length_m": float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
                        "net_displacement_m": float(np.linalg.norm(xyz[-1] - xyz[0])),
                        "rotation_first_to_last_deg": math.degrees(_quat_angle(quat[0], quat[-1])),
                        "roll_deg": _stats(rpy[:, 0]), "pitch_deg": _stats(rpy[:, 1]),
                        "yaw_first_deg": float(rpy[0, 2]), "yaw_last_deg": float(rpy[-1, 2]),
                        "yaw_range_deg": float(np.degrees(yaw.max() - yaw.min())),
                        "yaw_total_abs_change_deg": float(np.degrees(np.abs(np.diff(yaw)).sum()))})
        parents = defaultdict(set)
        for p, c in list(self.tf_static) + list(self.tf_dyn):
            parents[c].add(p)
        nodes = set(parents) | {p for ps in parents.values() for p in ps}
        return {"static": static, "dynamic": dyn, "roots": sorted(nodes - set(parents)),
                "multi_parent_children": sorted(c for c, ps in parents.items() if len(ps) > 1)}

    def _cloud_report(self, c):
        rings = np.unique(np.concatenate(c["rings"])) if c["rings"] else np.array([])
        layout = c["layout"][0] if c["layout"] else {}
        xyz = np.concatenate(c["xyz_sample"]) if c["xyz_sample"] else np.zeros((0, 3))
        cat = lambda k: np.concatenate(c[k]) if c[k] else []  # noqa: E731
        return {**layout, "sampled_scans": len(c["points"]),
                "shapes_height_width": sorted(set(c["shape"])), "organized": any(h > 1 and w > 1 for h, w in c["shape"]),
                "is_dense": sorted(set(c["is_dense"])), "points_per_scan": _stats(c["points"]),
                "finite_points_per_scan": _stats(c["valid_points"]),
                "nan_fraction": _stats(c["nan_frac"]), "zero_range_fraction": _stats(c["zero_frac"]),
                "range_m": _stats(cat("ranges")), "z_m": _stats(cat("z")), "intensity": _stats(cat("intensity")),
                "x_m": _stats(xyz[:, 0]), "y_m": _stats(xyz[:, 1]),
                "near_returns": self._near_report(c),
                "ring_values": {"count": int(rings.size), "min": int(rings.min()) if rings.size else None,
                                "max": int(rings.max()) if rings.size else None},
                "ring_constant_per_row": (all(c["ring_per_row"]) if c["ring_per_row"] else None),
                "time_field": c["time_field"][0] if c["time_field"] else None,
                "time_span_per_scan_s": _stats(c["time_span"]),
                "time_min_minus_header_s": _stats(c["time_min_minus_header"]),
                "time_max_minus_header_s": _stats(c["time_max_minus_header"])}

    @staticmethod
    def _near_report(c):
        near = np.concatenate(c["near"]) if c["near"] else np.zeros((0, 3))
        if not len(near):
            return {"range_lt_m": NEAR_RANGE_M, "points_per_scan_mean": 0.0}
        lo, hi = np.percentile(near, 1, axis=0), np.percentile(near, 99, axis=0)
        return {"range_lt_m": NEAR_RANGE_M, "points_per_scan_mean": len(near) / max(1, len(c["near"])),
                "box_p1_m": lo.tolist(), "box_p99_m": hi.tolist()}

    def _imu_report(self, topic, d):
        acc, gyro = np.asarray(d["acc"]), np.asarray(d["gyro"])
        rpy = np.degrees(np.array([_quat_to_rpy(*q) for q in d["quat"]]))
        yaw = np.unwrap(np.radians(rpy[:, 2]))
        st = np.asarray(d["stamp"])
        dt = np.diff(st) if len(st) > 1 else np.zeros(0)
        return {"acc_mean_mps2": acc.mean(axis=0).tolist(), "acc_std_mps2": acc.std(axis=0).tolist(),
                "acc_norm_mean": float(np.linalg.norm(acc, axis=1).mean()),
                "gyro_mean_rps": gyro.mean(axis=0).tolist(), "gyro_std_rps": gyro.std(axis=0).tolist(),
                "gyro_abs_max_rps": np.abs(gyro).max(axis=0).tolist(),
                "gyro_z_integrated_deg": float(np.degrees((gyro[1:, 2] * dt).sum())) if dt.size else 0.0,
                "orientation_roll_deg": _stats(rpy[:, 0]), "orientation_pitch_deg": _stats(rpy[:, 1]),
                "orientation_yaw_first_last_deg": [float(rpy[0, 2]), float(rpy[-1, 2])],
                "orientation_yaw_range_deg": float(np.degrees(yaw.max() - yaw.min())),
                "covariance_first": d["cov"]}

    def report(self, gap_factor=3.0):
        out = {"topics": {n: self.topics[n].summary(gap_factor) for n in sorted(self.topics)},
               "tf": self._tf_report(),
               "point_clouds": {t: self._cloud_report(c) for t, c in self.clouds.items()},
               "imu": {t: self._imu_report(t, d) for t, d in self.imu.items()},
               "camera_info": {t: {"distinct_versions": len(l), "first": l[0]["rec"], "counts": [v["count"] for v in l]}
                               for t, l in self.caminfo.items()},
               "images": {}, "lidar_packets": {}, "realsense_metadata": self.metadata_keys}
        for topic, d in self.images.items():
            rec = {"layouts": [dict(zip(("encoding", "width", "height", "step", "is_bigendian"), l))
                               for l in d["layout"]]}
            for k in ("depth_valid", "depth_median_raw", "depth_p95_raw", "depth_min_raw", "mean_intensity"):
                if d[k]:
                    rec[k] = _stats(d[k])
            out["images"][topic] = rec
        for topic, d in self.lidar_pkts.items():
            magic = dict(d["magic"])
            out["lidar_packets"][topic] = {
                "difop_packets": d["difop"], "frame_begin_flags": d["frame_begin"],
                "data_len_counts": {str(k): v for k, v in d["data_len"].items()},
                "msop_magic_counts": magic,
                "msop_magic_family": {k: ROBOSENSE_MSOP_MAGIC.get(k, "unknown") for k in magic},
                "helios_12x32_block_layout_ok": f"{d['block_hdr_ok']}/{d['checked']}",
                **self._packet_coverage(d),
                "difop_head_hex": d["difop_head"]}
        return out


def _qos_summary(profiles):
    """Reliability/durability of the first offered profile (rosbag2 enum values)."""
    if isinstance(profiles, list):  # rosbags >= 0.10 parses profiles into Qos objects
        if not profiles:
            return None
        q = profiles[0]
        return {"reliability": q.reliability.name.lower(), "durability": q.durability.name.lower()}
    if not isinstance(profiles, str) or not profiles:
        return None
    rel, dur, out = {"1": "reliable", "2": "best_effort"}, {"1": "transient_local", "2": "volatile"}, {}
    for line in profiles.splitlines():
        key, _, val = line.strip().lstrip("- ").partition(":")
        if key == "reliability":
            out.setdefault("reliability", rel.get(val.strip(), val.strip()))
        elif key == "durability":
            out.setdefault("durability", dur.get(val.strip(), val.strip()))
    return out


def inspect(bag, cloud_every=10, image_every=30, topics=None, gap_factor=3.0):
    from rosbags.highlevel import AnyReader
    store = typestore()
    ins = Inspector(store, cloud_every, image_every)
    with AnyReader([Path(bag)], default_typestore=store) as reader:
        conns = [c for c in reader.connections if topics is None or c.topic in topics]
        meta = {"path_name": Path(bag).resolve().name, "duration_s": (reader.end_time - reader.start_time) * 1e-9,
                "start_ns": reader.start_time, "end_ns": reader.end_time, "message_count": reader.message_count,
                "connections": len(reader.connections),
                "qos": {c.topic: _qos_summary(getattr(c.ext, "offered_qos_profiles", "")) for c in reader.connections}}
        for conn, ts, raw in reader.messages(connections=conns):
            ins.feed(conn.topic, conn.msgtype, ts, raw)
        for c in conns:
            ins.topics.setdefault(c.topic, TopicAcc(c.topic, c.msgtype))
    rep = ins.report(gap_factor)
    rep["bag"] = meta
    return rep


def _fmt(v, nd=3):
    if v is None:
        return "–"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def _g(d, k):
    return (d or {}).get(k)


def markdown(rep):
    b = rep["bag"]
    lines = [f"# Bag inspection: {b['path_name']}", "",
             f"Duration {b['duration_s']:.3f} s, {b['message_count']} messages, {b['connections']} connections.", "",
             "## Topics", "",
             "Gaps are periods > 3× the median period; receive-time gaps reflect delivery batching, "
             "header-time gaps reflect the sensor.", "",
             "| Topic | Type | Count | Rate Hz | Recv gaps n / max s | Header Hz | Header gaps n / max s "
             "| recv−header median s | frame_ids | QoS |",
             "|---|---|---:|---:|---|---:|---|---:|---|---|"]
    for name, t in rep["topics"].items():
        q = b["qos"].get(name) or {}
        lines.append(f"| `{name}` | {t['type'].split('/')[-1]} | {t['count']} | {_fmt(t.get('rate_hz_mean'), 2)} | "
                     f"{_fmt(_g(t.get('gaps'), 'count'))} / {_fmt(_g(t.get('gaps'), 'max_s'))} | "
                     f"{_fmt(t.get('header_rate_hz_mean'), 2)} | "
                     f"{_fmt(_g(t.get('header_gaps'), 'count'))} / {_fmt(_g(t.get('header_gaps'), 'max_s'))} | "
                     f"{_fmt(_g(t.get('recv_minus_header_s'), 'median'), 4)} | "
                     f"{', '.join(f'`{k}`' for k in t['frame_ids'])} | {q.get('reliability', '–')}/{q.get('durability', '–')} |")
    tf = rep["tf"]
    lines += ["", "## TF", "", f"Roots: {', '.join(tf['roots']) or '–'}; multi-parent children: "
              f"{tf['multi_parent_children'] or 'none'}", "", "### /tf_static", "",
              "| Parent → child | xyz m | rpy deg | quaternion xyzw |", "|---|---|---|---|"]
    for s in tf["static"]:
        lines.append(f"| `{s['parent']}` → `{s['child']}` | {', '.join(f'{v:.4f}' for v in s['translation_m'])} | "
                     f"{', '.join(f'{v:.2f}' for v in s['rpy_deg'])} | {', '.join(f'{v:.4f}' for v in s['quaternion_xyzw'])} |")
    lines += ["", "### /tf (dynamic)", "",
              "| Parent → child | n | Hz | translation const | path m | roll / pitch median deg | yaw first→last / range deg |",
              "|---|---:|---:|---|---:|---|---|"]
    for d in tf["dynamic"]:
        lines.append(f"| `{d['parent']}` → `{d['child']}` | {d['count']} | {_fmt(d['rate_hz'], 1)} | "
                     f"{d['translation_constant']} | {_fmt(d['path_length_m'])} | "
                     f"{_fmt(_g(d['roll_deg'], 'median'), 2)} / {_fmt(_g(d['pitch_deg'], 'median'), 2)} | "
                     f"{d['yaw_first_deg']:.1f}→{d['yaw_last_deg']:.1f} / {d['yaw_range_deg']:.1f} |")
    for topic, c in rep["point_clouds"].items():
        r = c["range_m"] or {}
        lines += ["", f"## PointCloud2 `{topic}`", "",
                  "Fields: " + ", ".join(f"`{f['name']}`:{f['datatype']}@{f['offset']}" for f in c.get("fields", [])),
                  "", f"point_step {c.get('point_step')}, shapes (h,w) {c['shapes_height_width'][:4]}, "
                  f"organized {c['organized']}, is_dense {c['is_dense']}, sampled scans {c['sampled_scans']}. "
                  f"Points/scan median {_fmt(_g(c['points_per_scan'], 'median'), 0)}, finite median "
                  f"{_fmt(_g(c['finite_points_per_scan'], 'median'), 0)}. Range m p5/median/p95/max "
                  f"{_fmt(r.get('p5'))}/{_fmt(r.get('median'))}/{_fmt(r.get('p95'))}/{_fmt(r.get('max'))}. "
                  f"Rings {c['ring_values']} (constant per row: {c['ring_constant_per_row']}). Time field "
                  f"`{c['time_field']}`, span/scan median {_fmt(_g(c['time_span_per_scan_s'], 'median'), 4)} s, "
                  f"min−header median {_fmt(_g(c['time_min_minus_header_s'], 'median'), 4)} s.",
                  "", f"Near returns (< {c['near_returns']['range_lt_m']} m): {c['near_returns']}."]
    for topic, d in rep["imu"].items():
        lines += ["", f"## IMU `{topic}`", "",
                  f"acc mean {[round(v, 3) for v in d['acc_mean_mps2']]} m/s² (|a| {d['acc_norm_mean']:.3f}); "
                  f"gyro std {[round(v, 4) for v in d['gyro_std_rps']]} rad/s, |max| "
                  f"{[round(v, 3) for v in d['gyro_abs_max_rps']]}; gyro-z integral {d['gyro_z_integrated_deg']:.1f}°; "
                  f"orientation yaw range {d['orientation_yaw_range_deg']:.1f}°.", "",
                  f"Covariance (first msg): `{d['covariance_first']}`"]
    for topic, d in rep["camera_info"].items():
        f = d["first"]
        lines += ["", f"## CameraInfo `{topic}`", "",
                  f"{f['width']}×{f['height']}, `{f['distortion_model']}`, K {[round(v, 3) for v in f['k']]}, "
                  f"D {[round(v, 5) for v in f['d']]}, distinct versions {d['distinct_versions']}."]
    for topic, d in rep["images"].items():
        lines += ["", f"## Image `{topic}`", "", f"Layouts {d['layouts']}."]
        if "depth_valid" in d:
            lines.append(f"Depth valid fraction median {d['depth_valid']['median']:.3f}; median raw "
                         f"{_fmt(_g(d.get('depth_median_raw'), 'median'), 0)}, p95 raw "
                         f"{_fmt(_g(d.get('depth_p95_raw'), 'median'), 0)}.")
    for topic, t in rep["topics"].items():
        if "extrinsics" in t:
            e = t["extrinsics"]
            lines += ["", f"## Extrinsics `{topic}`", "",
                      f"translation {[round(v, 5) for v in e['translation_m']]} m (|t| {e['baseline_m']:.4f}), "
                      f"rotation angle {e['rotation_angle_deg']:.3f}°."]
    for topic, d in rep["lidar_packets"].items():
        lines += ["", f"## Lidar packets `{topic}`", "", "```json", json.dumps(d, indent=1), "```"]
    if rep["realsense_metadata"]:
        lines += ["", "## RealSense metadata", "", "```json", json.dumps(rep["realsense_metadata"], indent=1), "```"]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory (with metadata.yaml)")
    ap.add_argument("--out", type=Path, default=None, help="output folder (default runs/bag_inspect/<bag-name>)")
    ap.add_argument("--cloud-every", type=int, default=10, help="fully decode every Nth PointCloud2")
    ap.add_argument("--image-every", type=int, default=30, help="fully decode every Nth Image")
    ap.add_argument("--topics", nargs="*", default=None, help="restrict to these topics")
    ap.add_argument("--gap-factor", type=float, default=3.0, help="gap = period > factor × median period")
    args = ap.parse_args(argv)
    rep = inspect(args.bag, args.cloud_every, args.image_every, set(args.topics) if args.topics else None,
                  args.gap_factor)
    out = args.out or Path("runs/bag_inspect") / Path(args.bag).resolve().name
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    (out / "report.md").write_text(markdown(rep))
    print(f"wrote {out / 'report.json'} and {out / 'report.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
