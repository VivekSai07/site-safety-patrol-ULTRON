"""Synthetic detection capture with an optional, locally supplied team hall."""

from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import types
import uuid
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from . import example_scene


CATEGORIES = [{"id": 1, "name": "ladder"}, {"id": 2, "name": "inspection_label"}]
SAHIL_HALL_SHA256 = "92c31c79afaaba385f5d8d9e77920d05aa9ba9962917091ae7c9f3eed9000b5e"
# Camera positions are free virtual viewpoints, not robot poses or traversable paths.
DEFAULT_VIEWS = (
    ((2.0, 1.4, 1.35), (4.0, 4.0, 1.1)),
    ((6.2, 1.0, 1.45), (6.2, 4.7, 1.2)),
    ((10.8, 3.2, 1.25), (8.0, 6.3, 1.15)),
    ((3.0, 8.6, 1.5), (6.0, 6.0, 1.2)),
    ((1.2, 2.0, 1.4), (0.1, 2.0, 1.4)),
)


@lru_cache(maxsize=4)
def _load_external_scene(filename):
    path = Path(filename).expanduser().resolve(strict=True)
    if not path.is_file() or not (path.suffix == ".py" or path.name.endswith(".py.local")):
        raise ValueError("the supplied team hall must be a Python source file")
    source = path.read_bytes()
    if hashlib.sha256(source).hexdigest() != SAHIL_HALL_SHA256:
        raise ValueError("the supplied team hall does not match the reviewed source snapshot")
    name = "_ultron_external_sahil_" + hashlib.sha256(os.fsencode(path)).hexdigest()[:16]
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    try:
        exec(compile(source, str(path), "exec"), module.__dict__)
    except Exception:
        sys.modules.pop(name, None)
        raise
    if not callable(getattr(module, "build_hall", None)) or not isinstance(
        getattr(module, "BASE_HEIGHT_M", None), (int, float)
    ):
        sys.modules.pop(name, None)
        raise ValueError("the supplied team hall has an unsupported interface")
    return module


def _scene():
    filename = os.environ.get("ULTRON_HALL_SCENE")
    return _load_external_scene(filename) if filename else example_scene


def _geom(parent, name, kind, **attrs):
    ET.SubElement(parent, "geom", name=name, type=kind, contype="0",
                  conaffinity="0", **attrs)


def build_xml():
    """Augment the independent example or an explicitly supplied team hall."""
    scene = _scene()
    hall_xml, _ = scene.build_hall()
    root = ET.fromstring(hall_xml)
    root.set("model", "detection_hall")
    root.find("visual/global").set("offwidth", "1920")
    root.find("visual/global").set("offheight", "1080")
    root.find("visual/headlight").set("ambient", ".25 .25 .25")
    root.find("visual/headlight").set("diffuse", ".35 .35 .35")
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="3 3 4", dir="0 0 -1", diffuse=".65 .55 .45")
    ET.SubElement(world, "light", pos="9 8 4", dir="0 0 -1", diffuse=".25 .35 .55")
    ET.SubElement(world, "light", pos="11.5 2.5 2.5", dir="-1 0 -.3", diffuse=".45 .42 .35")
    for index in range(3):
        _geom(world, f"material_stack_{index}", "box",
              pos=f"10.4 {2.1 + index * .16} .08",
              size=".7 .06 .08", rgba=".48 .40 .31 1")
    label_positions = (
        (("01", -.27, 1.25), ("02", -.27, 1.25), ("03", -.30, 1.25),
         ("04", .29, 1.25), ("05", .29, 1.25), ("06", -.055, .72),
         ("07", -.055, .72))
        if scene is not example_scene
        else tuple((f"{index:02}", -.07, 1.25) for index in range(1, 8))
    )
    for ident, y, z in label_positions:
        body = world.find(f"body[@name='ladder_{ident}']")
        _geom(body, f"label_{ident}_card", "box", pos=f"0 {y} {z}",
              size=".13 .012 .09", rgba=".96 .88 .25 1")
        for n in range(3):
            _geom(body, f"label_{ident}_stripe_{n}", "box",
                  pos=f"0 {y - .014} {z - .055 + n * .055}", size=".085 .003 .006",
                  rgba=".12 .12 .12 1")
    ET.SubElement(world, "camera", name="capture", pos="2 1 1.3", fovy="55")
    return ET.tostring(root, encoding="unicode")


def _initial_data(model):
    """Place a stationary kinematic B2; this is not walking."""
    data = mujoco.MjData(model)
    yaw = math.pi / 2
    data.qpos[:7] = [1.6, 1.5, _scene().BASE_HEIGHT_M, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or ""
        data.qpos[model.jnt_qposadr[joint]] = .65 if "thigh" in name else -1.3 if "calf" in name else 0
    mujoco.mj_forward(model, data)
    return data


def _rle(mask):
    """Uncompressed COCO RLE, column-major, including a zero run when needed."""
    flat = np.asarray(mask, dtype=np.uint8).ravel(order="F")
    changes = np.flatnonzero(flat[1:] != flat[:-1]) + 1
    counts = np.diff(np.concatenate(([0], changes, [flat.size]))).tolist()
    if flat.size and flat[0]:
        counts.insert(0, 0)
    return {"size": list(mask.shape), "counts": counts}


def visible_instances(segmentation, geom_instances):
    """Extract clipped visible masks only, indexed by simulator geom id."""
    ids = segmentation[:, :, 0]
    types = segmentation[:, :, 1]
    result = []
    for (name, category), geom_ids in sorted(geom_instances.items()):
        mask = (types == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(ids, geom_ids)
        ys, xs = np.nonzero(mask)
        if not len(xs):
            continue
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        result.append({"instance_id": name, "category_id": category,
                       "bbox": [x0, y0, x1 - x0 + 1, y1 - y0 + 1],
                       "area": int(mask.sum()), "iscrowd": 0, "segmentation": _rle(mask)})
    return result


def _camera(model, data, position, target):
    position = np.asarray(position, dtype=float)
    target = np.asarray(target, dtype=float)
    if position.shape != (3,) or target.shape != (3,) or not np.isfinite(position).all() or not np.isfinite(target).all():
        raise ValueError("Camera position and target must be finite XYZ triples")
    if not (0.05 <= position[0] <= 11.95 and 0.05 <= position[1] <= 9.95 and 0.1 <= position[2] <= 2.9):
        raise ValueError("Camera must be inside the hall bounds")
    forward = target - position
    norm = np.linalg.norm(forward)
    if norm < 1e-8:
        raise ValueError("Camera target must differ from position")
    forward /= norm
    right = np.cross(forward, [0, 0, 1])
    if np.linalg.norm(right) < 1e-8:
        raise ValueError("Camera cannot point exactly vertically")
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    matrix = np.column_stack((right, up, -forward))
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, matrix.ravel())
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "capture")
    model.cam_pos[cid] = position
    model.cam_quat[cid] = quat
    mujoco.mj_forward(model, data)
    return cid


def _instances(model):
    groups = {}
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        fields = name.split("_")
        if len(fields) >= 3 and fields[0] in ("ladder", "label"):
            kind = fields[0]
            key = (f"{kind}_{fields[1]}", 1 if kind == "ladder" else 2)
            groups.setdefault(key, []).append(geom_id)
    return groups


def capture(output=None, *, views=DEFAULT_VIEWS, width=640, height=480):
    """Create one non-overwriting COCO-style synthetic capture dataset."""
    if not (1 <= width <= 1920 and 1 <= height <= 1080):
        raise ValueError("Image dimensions must be within 1..1920 by 1..1080")
    views = list(views)
    if not views:
        raise ValueError("At least one camera view is required")
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = _initial_data(model)
    # Validate all camera arguments before creating any output.
    for position, target in views:
        _camera(model, data, position, target)
    output = Path(output) if output is not None else Path("runs") / f"detection-world-{uuid.uuid4().hex}"
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").mkdir()
    scene_id = example_scene.SCENE_ID if _scene() is example_scene else f"sahil-{SAHIL_HALL_SHA256}"
    dataset = {"info": {"description": "Synthetic visible-only MuJoCo detection world",
                        "scene_id": scene_id},
               "licenses": [], "categories": CATEGORIES, "images": [], "annotations": []}
    groups = _instances(model)
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        for image_id, (position, target) in enumerate(views, 1):
            cid = _camera(model, data, position, target)
            renderer.update_scene(data, camera=cid)
            rgb = renderer.render()
            renderer.enable_segmentation_rendering()
            try:
                segmentation = renderer.render()
            finally:
                renderer.disable_segmentation_rendering()
            filename = f"images/{image_id:06d}.png"
            Image.fromarray(rgb).save(output / filename)
            focal = height / (2 * math.tan(math.radians(float(model.cam_fovy[cid])) / 2))
            dataset["images"].append({
                "id": image_id, "file_name": filename, "width": width, "height": height,
                "group_id": example_scene.SCENE_ID if _scene() is example_scene else "detection-hall-v1",
                "camera": {"position_world_m": data.cam_xpos[cid].tolist(),
                           "rotation_world_from_camera": data.cam_xmat[cid].reshape(3, 3).tolist(),
                           "convention": "MuJoCo camera: right, up, back; world Z up",
                           "target_world_m": list(target), "fovy_degrees": float(model.cam_fovy[cid]),
                           "intrinsics": {"fx": focal, "fy": focal, "cx": width / 2, "cy": height / 2}},
            })
            for instance in visible_instances(segmentation, groups):
                instance.update(id=len(dataset["annotations"]) + 1, image_id=image_id)
                dataset["annotations"].append(instance)
    finally:
        renderer.close()
    (output / "instances.json").write_text(json.dumps(dataset, indent=2) + "\n", encoding="utf-8")
    return output
