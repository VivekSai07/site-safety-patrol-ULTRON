"""Wire format between the robot-side relay (ROS) and the pipeline (GPU machine).

Standard library and numpy only, so the relay runs in any ROS 2 Python without extra packages.
One TCP connection carries length-prefixed messages:

    4 bytes  header length (big endian)
    header   JSON: {"kind": "frame" | "cloud", "stamp": seconds, ..., "blobs": {name: byte length}}
    blobs    in the order of the header's "blobs" entries, each zlib-compressed

frame  colour image (rgb8) and depth image (uint16 millimetres) taken together, both cameras'
       intrinsics, the depth-to-colour transform, and the robot pose in the map if TF has one
cloud  lidar points (float32 x, y, z in the lidar frame), for odometry when TF has no pose
"""
import json
import struct
import zlib

import numpy as np

MAX_HEADER = 1 << 20
MAX_BLOB = 1 << 28


def pack(header, blobs):
    """Bytes for one message. `blobs` maps names to bytes-like payloads."""
    packed = {name: zlib.compress(bytes(data), 1) for name, data in blobs.items()}
    head = json.dumps({**header, "blobs": {name: len(data) for name, data in packed.items()}}).encode()
    return struct.pack(">I", len(head)) + head + b"".join(packed.values())


def _exactly(read, count):
    chunks, remaining = [], count
    while remaining:
        chunk = read(remaining)
        if not chunk:
            raise EOFError("relay connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def unpack(read):
    """One message from `read(n)` (a socket's recv): (header, {name: bytes}). EOFError at the end."""
    (length,) = struct.unpack(">I", _exactly(read, 4))
    if length > MAX_HEADER:
        raise ValueError("relay header too large: not a relay stream?")
    header = json.loads(_exactly(read, length))
    blobs = {}
    for name, size in header.get("blobs", {}).items():
        if not 0 <= size <= MAX_BLOB:
            raise ValueError("relay blob too large")
        blobs[name] = zlib.decompress(_exactly(read, size))
    return header, blobs


def frame_message(stamp, color, depth_mm, color_k, depth_k, rotation, translation, pose=None):
    """color: HxWx3 uint8 RGB. depth_mm: HxW uint16. *_k: (fx, fy, cx, cy). pose: (x, y, yaw) or None."""
    color, depth_mm = np.ascontiguousarray(color, np.uint8), np.ascontiguousarray(depth_mm, np.uint16)
    return pack({"kind": "frame", "stamp": float(stamp), "pose": None if pose is None else [float(v) for v in pose],
                 "color_shape": list(color.shape), "depth_shape": list(depth_mm.shape),
                 "color_k": [float(v) for v in color_k], "depth_k": [float(v) for v in depth_k],
                 "rotation": [float(v) for v in np.asarray(rotation).reshape(9)],
                 "translation": [float(v) for v in np.asarray(translation).reshape(3)]},
                {"color": color.tobytes(), "depth": depth_mm.tobytes()})


def cloud_message(stamp, points):
    points = np.ascontiguousarray(points, np.float32).reshape(-1, 3)
    return pack({"kind": "cloud", "stamp": float(stamp), "count": len(points)}, {"points": points.tobytes()})
