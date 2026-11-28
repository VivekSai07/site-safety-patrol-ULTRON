"""Isaac Sim 6.1 feasibility smoke test for ULTRON (SIMULATION ONLY, no remote assets).

Headless: builds a primitive room, creates a bare RTX lidar (no vendor asset) at the B2 lidar
height and publishes PointCloud2 on /ultron/sim/isaac/points through Isaac's ROS 2 bridge.
Run with Isaac's interpreter via scripts/isaac/run-smoke.sh (isolated ROS domain).
"""
import argparse
import sys
import time

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=20.0)
parser.add_argument('--snapshot', help='Folder for a rendered RGB image of the scene')
args, _ = parser.parse_known_args()
app = SimulationApp({'headless': True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

app_utils.enable_extension('isaacsim.ros2.bridge')
app.update()
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor  # noqa: E402

stage = stage_utils.get_current_stage()
UsdGeom.Xform.Define(stage, '/World')
# Lights (RTX renders black without them); they do not affect the lidar.
UsdLux.DomeLight.Define(stage, '/World/dome').CreateIntensityAttr(900.0)
sun = UsdLux.DistantLight.Define(stage, '/World/sun')
sun.CreateIntensityAttr(2500.0)
UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(-50.0, 20.0, 0.0))
# Primitive 8 x 6 m room: floor, four walls, two pillars (render geometry for RTX ray tracing).
for name, center, size in [('floor', (0, 0, -0.05), (8, 6, 0.1)), ('north', (0, 3, 1.5), (8, 0.2, 3)),
                           ('south', (0, -3, 1.5), (8, 0.2, 3)), ('east', (4, 0, 1.5), (0.2, 6, 3)),
                           ('west', (-4, 0, 1.5), (0.2, 6, 3)), ('pillar_a', (1.5, 1.0, 1.5), (0.4, 0.4, 3)),
                           ('pillar_b', (-1.5, -1.2, 1.5), (0.4, 0.4, 3))]:
    cube = UsdGeom.Cube.Define(stage, f'/World/{name}')
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*center))
    xform.SetScale(Gf.Vec3f(*size))
# Simple A-frame ladder from primitives (two inclined rail pairs and steps) and a robot stand-in.
def box(path, center, size, rot_x=0.0):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    api = UsdGeom.XformCommonAPI(cube)
    api.SetTranslate(Gf.Vec3d(*center))
    api.SetRotate(Gf.Vec3f(rot_x, 0.0, 0.0))
    api.SetScale(Gf.Vec3f(*size))
    return cube

for side, sign in (('front', -1), ('rear', 1)):
    for sx in (-1, 1):
        box(f'/World/ladder/{side}_rail_{"l" if sx < 0 else "r"}', (2.0 + sx * 0.27, -0.33 * sign * -1 + 0.0, 1.0), (0.07, 0.025, 2.1),
            rot_x=sign * 18.0)
    for n in range(1, 7):
        z = n * 0.27
        y = sign * 0.65 * (1 - z / 2.0)   # on the rails: foot at |y| = 0.65, apex at y = 0, z = 2
        box(f'/World/ladder/{side}_step_{n}', (2.0, y, z), (0.5, 0.08, 0.025))
box('/World/robot_standin', (0.0, 0.0, 0.45), (0.9, 0.5, 0.35))
app.update()

try:
    # accumulate_outputs: publish full 360-degree rotations, not per-frame partial sweeps (measured).
    lidar = Lidar.create(path='/World/ultron_lidar', tick_rate=10.0, translations=[[0.0, 0.0, 0.83]],
                         accumulate_outputs=True)
except Exception as exc:  # report precisely what a bare prim lacks
    print(f'SMOKE_RESULT lidar_create_failed: {exc!r}', flush=True)
    app.close()
    sys.exit(2)
prim = stage.GetPrimAtPath(lidar.paths[0])
print('SMOKE_INFO lidar prim type', prim.GetTypeName(), 'emitters',
      prim.GetAttribute('omni:sensor:Core:numberOfEmitters').Get(),
      'scanRate', prim.GetAttribute('omni:sensor:Core:scanRateBaseHz').Get(), flush=True)

SimulationManager.setup_simulation(dt=1.0 / 60.0, device='cpu')
app.update()
sensor = LidarSensor(lidar, annotators=[])
sensor.attach_writer('RtxLidarROS2PublishPointCloud', topicName='/ultron/sim/isaac/points', frameId='lidar_link')
app.update()
app_utils.play()
rgb = None
if args.snapshot:
    import omni.replicator.core as rep
    camera = rep.create.camera(position=(-3.3, 0.4, 2.3), look_at=(1.6, -0.1, 0.8))
    product = rep.create.render_product(camera, (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator('rgb')
    rgb.attach([product])
start, frames = time.monotonic(), 0
while app.is_running() and time.monotonic() - start < args.seconds:
    app.update()
    frames += 1
print(f'SMOKE_RESULT ran {frames} frames in {time.monotonic() - start:.1f} s', flush=True)
if rgb is not None:
    import os
    import numpy as np
    from PIL import Image
    for _ in range(10):
        app.update()
    data = np.asarray(rgb.get_data())
    os.makedirs(args.snapshot, exist_ok=True)
    Image.fromarray(data[..., :3].astype(np.uint8)).save(os.path.join(args.snapshot, 'isaac_scene.png'))
    print('SMOKE_RESULT snapshot', data.shape, flush=True)
app_utils.stop()
app.close()
