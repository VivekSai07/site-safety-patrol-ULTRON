"""Kinematic capture adapter for the organizer kit's official B2 MJCF.

This adapter loads the kit image's B2 model directly. It does not import ROS,
DDS, Unitree SDKs, or the legacy ``mujoco_sim`` demo.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import queue
import struct
import threading
import xml.etree.ElementTree as ET
from typing import Any
import zlib


STEP_METRES = 0.10
TURN_RADIANS = math.radians(5.0)
WORKSPACE_BOUNDS = (-3.0, 3.0, -3.0, 3.0)
BASE_HEIGHT_METRES = 0.85

_FONT = {
    " ": ("00000",) * 7,
    "-": ("00000", "00000", "00000", "11111", "00000", "00000", "00000"),
    ":": ("00000", "00100", "00100", "00000", "00100", "00100", "00000"),
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "6": ("01110", "10000", "10000", "11110", "10001", "10001", "01110"),
    "7": ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    "9": ("01110", "10001", "10001", "01111", "00001", "00001", "01110"),
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("01110", "00100", "00100", "00100", "00100", "00100", "01110"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "X": ("10001", "10001", "01010", "00100", "01010", "10001", "10001"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
}


@dataclass(frozen=True)
class SimPose:
    x: float
    y: float
    z: float
    yaw: float

    def as_dict(self) -> dict[str, float]:
        return {"x_m": self.x, "y_m": self.y, "z_m": self.z, "yaw_rad": self.yaw}


def build_scene_xml(b2_model: str | Path) -> str:
    """Build a small, team-owned capture scene around the image's B2 MJCF."""
    model_path = Path(b2_model).resolve()
    if model_path.name != "b2.xml" or model_path.parent.name != "b2":
        raise ValueError("the organizer kit's unitree_robots/b2/b2.xml is required")
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    tree = ET.parse(model_path)
    root = tree.getroot()
    if root.tag != "mujoco":
        raise ValueError("organizer B2 model must have a MuJoCo root element")
    root.set("model", "ultron organizer B2 capture scene")
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", (model_path.parent / "assets").as_posix())
    compiler.set("angle", "radian")

    insert_before = ("visual", "asset", "worldbody", "contact", "equality", "tendon", "actuator", "sensor", "keyframe")
    insert_at = next(
        (index for index, child in enumerate(root) if child.tag in insert_before),
        len(root),
    )
    statistic = ET.Element("statistic", center="0 0 0.5", extent="3.5")
    root.insert(insert_at, statistic)
    visual = root.find("visual")
    if visual is None:
        visual = ET.Element("visual")
        insert_at = next(
            (index for index, child in enumerate(root) if child.tag in ("asset", "worldbody")),
            len(root),
        )
        root.insert(insert_at, visual)
    ET.SubElement(
        visual,
        "headlight",
        diffuse="0.7 0.7 0.7",
        ambient="0.3 0.3 0.3",
        specular="0.05 0.05 0.05",
    )
    ET.SubElement(
        visual, "global", azimuth="90", elevation="-18",
        offwidth="1920", offheight="1080",
    )

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        insert_at = next(
            (index for index, child in enumerate(root) if child.tag == "worldbody"),
            len(root),
        )
        root.insert(insert_at, asset)
    ET.SubElement(
        asset,
        "texture",
        name="floor_grid",
        type="2d",
        builtin="checker",
        width="256",
        height="256",
        rgb1="0.18 0.22 0.25",
        rgb2="0.28 0.31 0.32",
        mark="edge",
        markrgb="0.4 0.42 0.42",
    )
    ET.SubElement(
        asset,
        "material",
        name="floor_mat",
        texture="floor_grid",
        texrepeat="8 8",
        reflectance="0.05",
    )
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 -2 5", dir="0 0 -1", directional="true")
    ET.SubElement(
        worldbody,
        "geom",
        name="capture_floor",
        type="plane",
        size="5 5 0.1",
        material="floor_mat",
    )
    base = worldbody.find(".//body[@name='base_link']")
    if base is not None:
        ET.SubElement(
            base, "camera", name="ultron_inspection_camera", pos="0.48 0 0.20",
            xyaxes="0 -1 0 -0.207912 0 0.978148", fovy="55",
        )

    def ladder(name: str, x: str, y_center: str, y_offset: str, color: str) -> None:
        body = ET.SubElement(worldbody, "body", name=f"fixture_ladder_{name}")
        y1 = str(float(y_center) - float(y_offset))
        y2 = str(float(y_center) + float(y_offset))
        for side, y in (("left", y1), ("right", y2)):
            ET.SubElement(
                body,
                "geom",
                name=f"{name}_{side}_rail",
                type="box",
                pos=f"{x} {y} 0.9",
                size="0.035 0.035 0.9",
                rgba=color,
            )
        for index, z in enumerate(("0.28", "0.57", "0.86", "1.15"), 1):
            ET.SubElement(
                body,
                "geom",
                name=f"{name}_rung_{index}",
                type="box",
                pos=f"{x} {y_center} {z}",
                size=f"0.04 {y_offset} 0.025",
                rgba=color,
            )
        if name == "alpha":
            def label(z: float, color: str, lines: tuple[str, ...]) -> None:
                ET.SubElement(
                    body, "geom", type="box",
                    pos=f"{float(x) - 0.04} {y_center} {z}",
                    size="0.025 0.23 0.105", rgba=color,
                )
                pixel = 0.004
                for line_number, text in enumerate(lines):
                    left = (len(text) * 6 - 1) * pixel / 2
                    for column, character in enumerate(text):
                        for row, pattern in enumerate(_FONT[character]):
                            for dot, on in enumerate(pattern):
                                if on == "1":
                                    y = float(y_center) + left - (column * 6 + dot) * pixel
                                    height = z + 0.070 - line_number * 0.055 - row * pixel
                                    ET.SubElement(
                                        body, "geom", type="box",
                                        pos=f"{float(x) - 0.066} {y:.5f} {height:.5f}",
                                        size="0.0008 0.002 0.002",
                                        rgba="0.02 0.02 0.02 1", contype="0",
                                        conaffinity="0",
                                    )
            label(1.58, "0.78 0.88 0.97 1", ("SYNTHETIC", "MAKER: ULTRON", "MFG: 2024-01"))
            label(1.31, "1 0.93 0.68 1", ("SYNTHETIC", "LAST CHECK: 2026-03", "NEXT CHECK: 2027-03"))

    ladder("alpha", "2", "0", "0.22", "0.83 0.78 0.58 1")
    ladder("bravo", "-2", "1.7", "0.22", "0.58 0.72 0.79 1")
    return ET.tostring(root, encoding="unicode")


def encode_png(rgb: bytes, width: int, height: int) -> bytes:
    """Encode packed RGB pixels as PNG without relying on Pillow in the kit image."""
    if width <= 0 or height <= 0 or len(rgb) != width * height * 3:
        raise ValueError("RGB frame dimensions do not match its pixel data")

    def chunk(kind: bytes, payload: bytes) -> bytes:
        crc = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", crc)

    stride = width * 3
    scanlines = b"".join(
        b"\x00" + rgb[offset : offset + stride]
        for offset in range(0, len(rgb), stride)
    )
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(scanlines))
        + chunk(b"IEND", b"")
    )


class KitSimulation:
    """Small-step, collision-checked kinematic B2 placement and rendered preview."""

    def __init__(
        self,
        kit_root: str | Path = "/opt/unitree_mujoco",
        *,
        width: int = 640,
        height: int = 360,
    ) -> None:
        if not (160 <= width <= 1920 and 120 <= height <= 1080):
            raise ValueError("preview dimensions must be within 160x120 and 1920x1080")
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                "MuJoCo Python is required; run inside the organizer image with "
                "/opt/unitree-venv/bin/python"
            ) from exc

        self._mujoco = mujoco
        self.kit_root = Path(kit_root).resolve()
        self.b2_xml = self.kit_root / "unitree_robots" / "b2" / "b2.xml"
        if not self.b2_xml.is_file():
            raise FileNotFoundError(
                f"organizer kit B2 model not found: {self.b2_xml}; "
                "legacy ULTRON assets are not used as a fallback"
            )
        self.model = mujoco.MjModel.from_xml_string(build_scene_xml(self.b2_xml))
        self.data = mujoco.MjData(self.model)
        free_joints = [
            joint_id
            for joint_id in range(self.model.njnt)
            if self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        ]
        if len(free_joints) != 1:
            raise ValueError("expected exactly one free base joint in kit B2 model")
        self._joint_id = free_joints[0]
        self._qpos_address = int(self.model.jnt_qposadr[self._joint_id])
        self._base_body = int(self.model.jnt_bodyid[self._joint_id])
        self._robot_bodies = self._descendant_bodies(self._base_body)
        self._height = BASE_HEIGHT_METRES
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._width = width
        self._height_pixels = height
        self.data.qpos[self._qpos_address : self._qpos_address + 7] = (
            self._x,
            self._y,
            self._height,
            1.0,
            0.0,
            0.0,
            0.0,
        )
        mujoco.mj_forward(self.model, self.data)
        self._render_queue: queue.Queue = queue.Queue()
        self._render_thread = threading.Thread(
            target=self._render_loop, name="ultron-mujoco-renderer", daemon=True
        )
        self._camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self._camera)
        self._camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self._camera.trackbodyid = self._base_body
        self._camera.distance = 4.0
        self._camera.azimuth = 90
        self._camera.elevation = -12
        self._view = "overview"
        self._camera_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "ultron_inspection_camera"
        )
        if self._camera_id < 0:
            raise ValueError("the organizer B2 base body is missing the inspection camera")
        self._last_collision = False
        self._render_thread.start()

    def _descendant_bodies(self, root: int) -> set[int]:
        bodies = set()
        for body_id in range(self.model.nbody):
            candidate = body_id
            while candidate != 0:
                if candidate == root:
                    bodies.add(body_id)
                    break
                candidate = int(self.model.body_parentid[candidate])
        return bodies

    def _set_pose(self, x: float, y: float, yaw: float) -> None:
        qpos = self.data.qpos
        qpos[self._qpos_address : self._qpos_address + 7] = (
            x,
            y,
            self._height,
            math.cos(yaw / 2),
            0.0,
            0.0,
            math.sin(yaw / 2),
        )
        self._mujoco.mj_forward(self.model, self.data)

    def _has_obstacle_contact(self) -> bool:
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            body_a = int(self.model.geom_bodyid[contact.geom1])
            body_b = int(self.model.geom_bodyid[contact.geom2])
            a_is_robot = body_a in self._robot_bodies
            b_is_robot = body_b in self._robot_bodies
            if a_is_robot == b_is_robot:
                continue
            other_geom = contact.geom2 if a_is_robot else contact.geom1
            other_name = self._mujoco.mj_id2name(
                self.model, self._mujoco.mjtObj.mjOBJ_GEOM, other_geom
            )
            if other_name != "capture_floor":
                return True
        return False

    def step(self, action: str) -> bool:
        """Apply one fixed translation/turn or reject it without moving."""
        if action not in {"forward", "back", "turn_left", "turn_right"}:
            raise ValueError("unsupported teleoperation action")
        x, y, yaw = self._x, self._y, self._yaw
        if action == "forward":
            x += STEP_METRES * math.cos(yaw)
            y += STEP_METRES * math.sin(yaw)
        elif action == "back":
            x -= STEP_METRES * math.cos(yaw)
            y -= STEP_METRES * math.sin(yaw)
        elif action == "turn_left":
            yaw = math.atan2(math.sin(yaw + TURN_RADIANS), math.cos(yaw + TURN_RADIANS))
        else:
            yaw = math.atan2(math.sin(yaw - TURN_RADIANS), math.cos(yaw - TURN_RADIANS))

        min_x, max_x, min_y, max_y = WORKSPACE_BOUNDS
        margin = 0.45
        if not (
            min_x + margin <= x <= max_x - margin
            and min_y + margin <= y <= max_y - margin
        ):
            self._last_collision = True
            return False

        self._set_pose(x, y, yaw)
        if self._has_obstacle_contact():
            self._set_pose(self._x, self._y, self._yaw)
            self._last_collision = True
            return False
        self._x, self._y, self._yaw = x, y, yaw
        self._last_collision = False
        return True

    def pose(self) -> SimPose:
        return SimPose(self._x, self._y, self._height, self._yaw)

    def camera_metadata(self) -> dict[str, Any]:
        metadata = {
            "type": (
                "SIMULATED_B2_INSPECTION_CAMERA"
                if self._view == "inspection" else "MUJOCO_TRACKING_OBSERVER"
            ),
            "onboard_sensor": self._view == "inspection",
            "width": self._width,
            "height": self._height_pixels,
            "frame": "simulation_scene",
        }
        if self._view == "inspection":
            metadata["position_m"] = self.data.cam_xpos[self._camera_id].tolist()
            metadata["rotation_matrix"] = self.data.cam_xmat[self._camera_id].tolist()
            metadata["vertical_fov_degrees"] = float(self.model.cam_fovy[self._camera_id])
        return metadata

    def set_view(self, view: str) -> None:
        if view not in {"overview", "inspection"}:
            raise ValueError("view must be overview or inspection")
        self._view = view

    def _render_loop(self) -> None:
        renderer = None
        try:
            while True:
                task = self._render_queue.get()
                if task is None:
                    return
                done, result = task
                try:
                    if renderer is None:
                        renderer = self._mujoco.Renderer(
                            self.model, height=self._height_pixels, width=self._width
                        )
                    camera = (
                        "ultron_inspection_camera"
                        if self._view == "inspection" else self._camera
                    )
                    renderer.update_scene(self.data, camera=camera)
                    result.append(renderer.render().tobytes(order="C"))
                except Exception as exc:
                    result.append(exc)
                finally:
                    done.set()
        finally:
            if renderer is not None:
                renderer.close()

    def render_rgb(self) -> bytes:
        done = threading.Event()
        result: list[Any] = []
        self._render_queue.put((done, result))
        done.wait()
        if isinstance(result[0], Exception):
            raise result[0]
        return result[0]

    def frame_size(self) -> tuple[int, int]:
        return self._width, self._height_pixels

    @property
    def last_collision(self) -> bool:
        return self._last_collision

    def close(self) -> None:
        self._render_queue.put(None)
        self._render_thread.join()
