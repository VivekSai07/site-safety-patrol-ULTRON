"""Read a patrol rendered by the ladderworld simulator (Isaac Sim, the B2's real sensor rig).

A patrol folder holds sensors/frames.jsonl (one line per frame: `k`, `t` and `base_render`, the
body pose [x y z roll pitch yaw] in the field's world frame, R = Rz Ry Rx), sensors/color/{k}.jpg,
sensors/depth/{k}.png (uint16 mm, the D457 depth camera's own image, not registered) and
sensors/rig.json (camera intrinsics, the head mount on base_link, the colour camera's offset in
the head). Poses are the simulator's truth, so they are in one frame for every patrol of a field:
the pipeline sees them as poses in a saved map.

`depth=noisy` reads sensors/depth_noisy/ (the D457's noise added, where the simulator provides it).
`error=DEG/DX/DY` moves every pose by a fixed rotation about the world origin and shift, as a
robot localised in a wrongly aligned map, or started from an unknown place, would report it.
"""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

from .bag import register_depth
from .frames import Frame
from .lidar import Calibration


def _rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _quaternion(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rig(path):
    return json.loads((Path(path) / "sensors" / "rig.json").read_text())


def _k(info):
    k = info["K"]
    return (k[0], k[4], k[2], k[5])


def calibration(path):
    """The colour camera's `Calibration` from the patrol's rig.json."""
    color = _rig(path)["camera_info"]["color"]
    return Calibration(_k(color), (color["width"], color["height"]))


def camera_pose(base, rig):
    """(x, y, yaw) of the colour camera in the world: its position, and where it looks, on the floor."""
    x, y, z, roll, pitch, yaw = base
    body = _rotation(roll, pitch, yaw)
    head = rig["frames"]["head_link"]
    head_rotation = body @ _quaternion(*head["q_wxyz"])
    position = np.array([x, y, z]) + body @ np.asarray(head["xyz"]) + head_rotation @ np.asarray(rig["color_in_head"])
    forward = head_rotation[:, 0]
    return float(position[0]), float(position[1]), math.atan2(forward[1], forward[0])


def _misplace(pose, error):
    angle, dx, dy = math.radians(error[0]), error[1], error[2]
    c, s = math.cos(angle), math.sin(angle)
    x, y, yaw = pose
    return c * x - s * y + dx, s * x + c * y + dy, yaw + angle


def frames(path, rate=2.0, error=None, depth_folder="depth"):
    """Colour frames at about `rate` per second, with depth registered to colour and the camera's pose."""
    path = Path(path)
    rig = _rig(path)
    color, depth_info = rig["camera_info"]["color"], rig["camera_info"]["depth"]
    camera = calibration(path)
    # Both cameras share the head's orientation; the colour camera sits `color_in_head` (head link
    # axes: x forward, y left, z up) from the depth camera. In optical axes (x right, y down, z
    # forward) a depth-camera point is, seen from the colour camera, shifted by minus that offset.
    fx, fy, fz = rig["color_in_head"]
    translation = -np.array([-fy, -fz, fx])
    last = None
    with open(path / "sensors" / "frames.jsonl") as lines:
        for line in lines:
            record = json.loads(line)
            stamp = float(record["t"])
            if last is not None and stamp - last < 1 / rate - 1e-6:
                continue
            last = stamp
            k = record["k"]
            image = Image.open(path / "sensors" / "color" / f"{k:06d}.jpg").convert("RGB")
            depth = np.asarray(Image.open(path / "sensors" / depth_folder / f"{k:06d}.png"), dtype=np.float32) / 1000
            registered = register_depth(depth, _k(depth_info), _k(color), (color["width"], color["height"]),
                                        np.eye(3), translation)
            pose = camera_pose(record["base_render"], rig)
            if error:
                pose = _misplace(pose, error)
            yield Frame(image, name=f"sim-{k:06d}", stamp=stamp, pose=pose, pose_frame="map",
                        depth=registered, camera=camera)
