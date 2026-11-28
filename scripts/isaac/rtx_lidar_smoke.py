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
args, _ = parser.parse_known_args()
app = SimulationApp({'headless': True})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

app_utils.enable_extension('isaacsim.ros2.bridge')
app.update()
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor  # noqa: E402

stage = stage_utils.get_current_stage()
UsdGeom.Xform.Define(stage, '/World')
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
app.update()

try:
    lidar = Lidar.create(path='/World/ultron_lidar', tick_rate=10.0, translations=[[0.0, 0.0, 0.83]])
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
start, frames = time.monotonic(), 0
while app.is_running() and time.monotonic() - start < args.seconds:
    app.update()
    frames += 1
print(f'SMOKE_RESULT ran {frames} frames in {time.monotonic() - start:.1f} s', flush=True)
app_utils.stop()
app.close()
