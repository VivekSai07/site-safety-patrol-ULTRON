"""MuJoCo-rendered RGB-D head camera for the ULTRON hall simulation. SIMULATION ONLY.

Renders color, planar depth and an ORACLE ladder-segmentation image from the kinematic B2 pose
in ``ultron.autonomy.hall_scene.HallSim``. The base is kinematic and level, so the camera has
yaw and pitch only (roll must be zero). Images are simulation renders of primitive geometry,
not sensor data; the segmentation is simulator truth for the labelled ``oracle_sim`` backend.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class StreamSpec:
    width: int
    height: int
    fy: float
    frame_id: str

    @property
    def fovy_deg(self):
        return math.degrees(2.0 * math.atan(self.height / 2.0 / self.fy))

    def intrinsics(self):
        """Pinhole intrinsics of a MuJoCo render: square pixels, centred principal point."""
        return {'width': self.width, 'height': self.height, 'fx': self.fy, 'fy': self.fy,
                'cx': (self.width - 1) / 2.0, 'cy': (self.height - 1) / 2.0}


def stream_from_config(cfg):
    return StreamSpec(int(cfg['width']), int(cfg['height']), float(cfg['fy']), str(cfg['frame_id']))


class HeadCamera:
    def __init__(self, sim, mount_xyz, mount_rpy, color, depth, *, min_depth_m=0.3, max_depth_m=10.0,
                 segmentation=True, base_height_m=None):
        import mujoco
        from ultron.autonomy.hall_scene import BASE_HEIGHT_M
        if abs(mount_rpy[0]) > 1e-9:
            raise ValueError('camera mount roll is not supported by the level kinematic base')
        self.mujoco, self.sim = mujoco, sim
        self.mount_xyz = tuple(float(v) for v in mount_xyz)
        self.pitch = float(mount_rpy[1])
        self.yaw = float(mount_rpy[2])
        self.color, self.depth = color, depth
        self.min_depth_m, self.max_depth_m = float(min_depth_m), float(max_depth_m)
        self.base_height = BASE_HEIGHT_M if base_height_m is None else base_height_m
        model = sim.model
        # Offscreen buffer must hold the largest stream.
        model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), color.width, depth.width)
        model.vis.global_.offheight = max(int(model.vis.global_.offheight), color.height, depth.height)
        self._color_r = mujoco.Renderer(model, color.height, color.width)
        self._depth_r = mujoco.Renderer(model, depth.height, depth.width)
        self._depth_r.enable_depth_rendering()
        self._seg_r = None
        if segmentation:
            self._seg_r = mujoco.Renderer(model, color.height, color.width)
            self._seg_r.enable_segmentation_rendering()
        self._cam = mujoco.MjvCamera()
        self._cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._cam.distance = 1.0
        self.ladder_labels = {}
        self._label_lut = self._ladder_lut()

    def _ladder_lut(self):
        """geom id -> ladder index (1-based, by ladder name order); 0 = not a ladder."""
        mj, model = self.mujoco, self.sim.model
        names = sorted(o.name for o in self.sim.objects if o.name.startswith('ladder_'))
        self.ladder_labels = {name: i + 1 for i, name in enumerate(names)}
        body_label = np.zeros(model.nbody, dtype=np.uint16)
        for name, label in self.ladder_labels.items():
            bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                body_label[bid] = label
        lut = np.zeros(model.ngeom + 1, dtype=np.uint16)  # index -1 (background) -> last slot = 0
        lut[:model.ngeom] = body_label[model.geom_bodyid]
        return lut

    def pose(self):
        """Camera optical centre (world), azimuth and elevation (degrees) from the true base pose."""
        x, y, yaw = self.sim.pose
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my, mz = self.mount_xyz
        pos = np.array([x + c * mx - s * my, y + s * mx + c * my, self.base_height + mz])
        return pos, math.degrees(yaw + self.yaw), -math.degrees(self.pitch)

    def forward(self):
        _, az, el = self.pose()
        a, e = math.radians(az), math.radians(el)
        return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])

    def _aim(self, spec):
        pos, az, el = self.pose()
        self._cam.azimuth, self._cam.elevation = az, el
        self._cam.lookat[:] = pos + self.forward() * self._cam.distance
        self.sim.model.vis.global_.fovy = spec.fovy_deg

    # Scene update (reads MjData; main thread) and GL render (renderer's thread) are separate so
    # rendering can run in a worker thread that owns the EGL contexts.
    def prepare(self):
        self._aim(self.color)
        self._color_r.update_scene(self.sim.data, self._cam)
        if self._seg_r is not None:
            self._seg_r.update_scene(self.sim.data, self._cam)
        self._aim(self.depth)
        self._depth_r.update_scene(self.sim.data, self._cam)

    def render_prepared(self):
        """Render the prepared scenes: (rgb uint8 HxWx3, depth uint16 mm, ladder labels uint16 | None)."""
        rgb = self._color_r.render()
        d = self._depth_r.render()
        valid = (d >= self.min_depth_m) & (d <= self.max_depth_m) & np.isfinite(d)
        depth_mm = np.where(valid, np.rint(d * 1000.0), 0).astype(np.uint16)
        labels = None
        if self._seg_r is not None:
            seg = self._seg_r.render()
            ids, types = seg[..., 0], seg[..., 1]
            geom = np.where(types == int(self.mujoco.mjtObj.mjOBJ_GEOM), ids, -1)
            labels = self._label_lut[geom]
        return rgb, depth_mm, labels

    def render(self):
        self.prepare()
        return self.render_prepared()

    def close(self):
        for r in (self._color_r, self._depth_r, self._seg_r):
            if r is not None:
                try:
                    r.close()
                except Exception:  # noqa: BLE001 - EGL teardown errors at shutdown are harmless
                    pass
        self._color_r = self._depth_r = self._seg_r = None
