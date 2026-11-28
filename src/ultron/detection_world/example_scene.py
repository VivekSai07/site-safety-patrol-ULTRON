"""Independent synthetic detection example using the existing team scene."""

import math
import xml.etree.ElementTree as ET

from ultron.scene import make_scene


BASE_HEIGHT_M = 0.65
SCENE_ID = "independent-example-v1"


def build_hall():
    placements = (
        (2.7, 3.4, -0.25, "A_FRAME_STEPLADDER"),
        (5.0, 4.1, 0.45, "A_FRAME_STEPLADDER"),
        (8.5, 3.7, 0.0, "LEANING_SINGLE"),
        (3.0, 6.8, 0.8, "A_FRAME_STEPLADDER"),
        (6.5, 6.5, 0.0, "LEANING_SINGLE"),
        (9.5, 6.7, -0.6, "A_FRAME_STEPLADDER"),
        (10.6, 8.0, 1.2, "A_FRAME_STEPLADDER"),
    )
    ladders = [
        {
            "key": f"ladder_{index:02}",
            "position": [x, y, 0.0],
            "type": kind,
            "due": "2099-12-31",
            "defect": None,
        }
        for index, (x, y, _, kind) in enumerate(placements, 1)
    ]
    root = ET.fromstring(make_scene(ladders))
    world = root.find("worldbody")
    for index, (_, _, yaw, _) in enumerate(placements, 1):
        body = world.find(f"body[@name='ladder_{index:02}']")
        body.set("quat", f"{math.cos(yaw / 2)} 0 0 {math.sin(yaw / 2)}")
    for geom in world.findall("geom"):
        name = geom.get("name", "")
        if name.startswith("ladder_") and name.endswith("_support_wall"):
            geom.set("name", "support_" + name)
    return ET.tostring(root, encoding="unicode"), ladders
