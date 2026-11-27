"""Indoor test hall for lidar SLAM, modelled loosely on the team's reference photos.

SIMULATION ONLY. All dimensions are approximate (``assumed``) primitives: Layher-style
double-sided A-frame stepladders with strap spreaders, combination ladders with top
rollers, trestles, walls, pillars, furniture and static people. This is not a replica
of the organizer course. The protected demo scene in ``ultron.scene`` is unchanged.

Ground-truth object metadata returned here is for evaluation only and must never be
passed to an estimator.
"""
from dataclasses import dataclass, field
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).resolve().parents[1] / 'assets' / 'b2'
BASE_HEIGHT_M = 0.65          # kinematic standing height used by the existing simulation
LIDAR_HOUSING_GROUP = 4       # excluded from ray casting (the sensor cannot see itself)
ROBOT_GROUP = 1               # B2 collision primitives; the lidar CAN hit these
SCENE_GROUP = 0

ALU = '.78 .80 .82 1'
ORANGE = '.95 .45 .10 1'
DARK = '.08 .08 .09 1'


@dataclass
class HallObject:
    """Evaluation-only ground truth for one placed object."""
    name: str
    kind: str
    position: tuple
    yaw: float
    footprint: list = field(default_factory=list)  # world-frame XY corners


def _fmt(values):
    return ' '.join(f'{v:.4f}' for v in values)


def _geom(parent, name, gtype, color, group=SCENE_GROUP, **attrs):
    attrs = {k: (_fmt(v) if isinstance(v, (tuple, list)) else str(v)) for k, v in attrs.items()}
    return ET.SubElement(parent, 'geom', name=name, type=gtype, rgba=color, group=str(group),
                         contype='0', conaffinity='0', **attrs)


MAX_TILE_HALF_M = 0.15


def _box(parent, name, pos, half, color=ALU):
    """Axis-aligned box, tiled horizontally into pieces of at most 0.3 x 0.3 m footprint.

    mujoco.mj_multiRay (3.3.7) culls geoms using the angles of their bounding-box corners
    seen from the ray origin. For a large box near the sensor the nearest face point is
    not a corner, so low beams were measured passing through long walls. Small tiles keep
    corners close to the face; tests compare against exact single-ray mj_ray.
    """
    nx = max(1, math.ceil(half[0] / MAX_TILE_HALF_M - 1e-9))
    ny = max(1, math.ceil(half[1] / MAX_TILE_HALF_M - 1e-9))
    if nx == ny == 1:
        return _geom(parent, name, 'box', color, pos=pos, size=half)
    hx, hy = half[0] / nx, half[1] / ny
    for i in range(nx):
        for j in range(ny):
            center = (pos[0] - half[0] + (2 * i + 1) * hx, pos[1] - half[1] + (2 * j + 1) * hy, pos[2])
            _geom(parent, f'{name}__{i}_{j}', 'box', color, pos=center, size=(hx, hy, half[2]))


def _bar(parent, name, start, end, half, color=ALU):
    """Rectangular bar between two points (box with fromto; half = two cross-section half sizes)."""
    return _geom(parent, name, 'box', color, fromto=(*start, *end), size=half)


def _rod(parent, name, start, end, radius, color=ALU):
    return _geom(parent, name, 'cylinder', color, fromto=(*start, *end), size=(radius,))


def _yaw_quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def _footprint(position, yaw, half_x, half_y, y_offset=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for dx, dy in ((-half_x, -half_y), (half_x, -half_y), (half_x, half_y), (-half_x, half_y)):
        dy += y_offset
        corners.append((position[0] + c * dx - s * dy, position[1] + s * dx + c * dy))
    return corners


def _lerp(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def layher_aframe(world, name, position, yaw, height=2.0, spread=1.3, steps=7):
    """Double-sided A-frame stepladder: flat steps on both sides, two strap spreaders."""
    body = ET.SubElement(world, 'body', name=name, pos=_fmt((*position, 0)), quat=_fmt(_yaw_quat(yaw)))
    bottom_w, top_w = 0.30, 0.22  # half widths
    sides = {'front': -spread / 2, 'rear': spread / 2}
    for side, y0 in sides.items():
        for sx in (-1, 1):
            start, end = (sx * bottom_w, y0, 0.0), (sx * top_w, 0.0, height)
            _bar(body, f'{name}_{side}_rail_{sx}', start, end, (0.035, 0.0125))
            _box(body, f'{name}_{side}_foot_{sx}', (sx * bottom_w, y0, 0.02), (0.045, 0.03, 0.02), ORANGE)
        for n in range(1, steps + 1):
            t = n / (steps + 1)
            half = bottom_w + (top_w - bottom_w) * t
            _box(body, f'{name}_{side}_step_{n}', (0, y0 * (1 - t), height * t), (half, 0.04, 0.012))
    _box(body, f'{name}_top_cap', (0, 0, height + 0.03), (top_w + 0.03, 0.06, 0.03), ORANGE)
    t = 0.38  # strap height fraction
    for sx in (-1, 1):
        x = sx * (bottom_w + (top_w - bottom_w) * t)
        _bar(body, f'{name}_strap_{sx}', (x, -spread / 2 * (1 - t), height * t),
             (x, spread / 2 * (1 - t), height * t), (0.002, 0.0125), ORANGE)
    return HallObject(name, 'LAYHER_A_FRAME_STEPLADDER', position, yaw,
                      _footprint(position, yaw, bottom_w + 0.05, spread / 2 + 0.05))


def combination_aframe(world, name, position, yaw, height=2.5, spread=1.1, rungs=8):
    """Combination ladder set up as an A-frame: round rungs, top rollers, stabiliser bar."""
    body = ET.SubElement(world, 'body', name=name, pos=_fmt((*position, 0)), quat=_fmt(_yaw_quat(yaw)))
    half_w = 0.22
    for side, y0 in (('front', -spread / 2), ('rear', spread / 2)):
        for sx in (-1, 1):
            _bar(body, f'{name}_{side}_rail_{sx}', (sx * half_w, y0, 0.0), (sx * half_w, 0.0, height), (0.03, 0.012))
        for n in range(1, rungs + 1):
            t = n / (rungs + 1)
            y, z = y0 * (1 - t), height * t
            _rod(body, f'{name}_{side}_rung_{n}', (-half_w, y, z), (half_w, y, z), 0.014)
    _bar(body, f'{name}_stabiliser', (-0.45, -spread / 2, 0.03), (0.45, -spread / 2, 0.03), (0.03, 0.015))
    for sx in (-1, 1):
        _rod(body, f'{name}_roller_{sx}', (sx * (half_w - 0.05), 0, height + 0.05),
             (sx * (half_w + 0.05), 0, height + 0.05), 0.07, '.85 .80 .65 1')
    return HallObject(name, 'COMBINATION_LADDER_A_FRAME', position, yaw,
                      _footprint(position, yaw, 0.5, spread / 2 + 0.05))


def leaning_ladder(world, name, position, yaw, length=3.0, angle_deg=75.0, rungs=9):
    """Single ladder leaning on a wall. Local +y points from the foot towards the wall."""
    body = ET.SubElement(world, 'body', name=name, pos=_fmt((*position, 0)), quat=_fmt(_yaw_quat(yaw)))
    half_w, a = 0.22, math.radians(angle_deg)
    run, rise = length * math.cos(a), length * math.sin(a)
    for sx in (-1, 1):
        _bar(body, f'{name}_rail_{sx}', (sx * half_w, 0, 0), (sx * half_w, run, rise), (0.03, 0.012))
        _rod(body, f'{name}_roller_{sx}', (sx * (half_w - 0.05), run - 0.06, rise),
             (sx * (half_w + 0.05), run - 0.06, rise), 0.06, '.85 .80 .65 1')
    for n in range(1, rungs + 1):
        t = n / (rungs + 1)
        _rod(body, f'{name}_rung_{n}', (-half_w, run * t, rise * t), (half_w, run * t, rise * t), 0.014)
    _bar(body, f'{name}_stabiliser', (-0.45, 0, 0.03), (0.45, 0, 0.03), (0.03, 0.015))
    return HallObject(name, 'LEANING_COMBINATION_LADDER', position, yaw,
                      _footprint(position, yaw, 0.5, run / 2 + 0.05, y_offset=run / 2))


def trestle(world, name, position, yaw, height=0.8, length=0.9, spread=0.6):
    body = ET.SubElement(world, 'body', name=name, pos=_fmt((*position, 0)), quat=_fmt(_yaw_quat(yaw)))
    _bar(body, f'{name}_top', (-length / 2, 0, height), (length / 2, 0, height), (0.04, 0.03))
    for sx in (-1, 1):
        for sy in (-1, 1):
            _bar(body, f'{name}_leg_{sx}_{sy}', (sx * length / 2, sy * spread / 2, 0),
                 (sx * (length / 2 - 0.05), 0, height), (0.02, 0.012))
    return HallObject(name, 'TRESTLE', position, yaw, _footprint(position, yaw, length / 2 + 0.05, spread / 2 + 0.05))


def _prepare_robot(root):
    """Official B2 joint tree/inertials with primitive collision geometry only (as ultron.scene)."""
    root.find('compiler').attrib.pop('meshdir', None)
    root.remove(root.find('asset'))
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'geom' and child.get('type') == 'mesh':
                parent.remove(child)
    ET.SubElement(root.find('default/default'), 'geom', type='box')
    for geom in root.findall('.//geom'):
        if geom.get('class') == 'collision':
            geom.set('rgba', '0.28 0.58 0.68 1')
            geom.set('group', str(ROBOT_GROUP))
    base = root.find(".//body[@name='base_link']")
    for geom in base.findall('geom'):
        if geom.get('type') == 'cylinder' and geom.get('pos', '').startswith('0.34218'):
            # The stock front cylinder sits where the team's RoboSense lidar is mounted.
            geom.set('name', 'lidar_housing')
            geom.set('group', str(LIDAR_HOUSING_GROUP))
            geom.set('rgba', '.12 .12 .13 1')
    return base


def _add_payload(base, mount_xyz, mount_rpy):
    """Sensor head and compute box from the team photos (assumed sizes); they occlude the lidar."""
    payload = '.9 .9 .9 1'
    ET.SubElement(base, 'geom', name='payload_camera_bracket', type='box', group=str(ROBOT_GROUP),
                  pos='0.36 0 0.30', size='0.08 0.11 0.05', rgba=DARK, contype='0', conaffinity='0')
    ET.SubElement(base, 'geom', name='payload_floodlight', type='box', group=str(ROBOT_GROUP),
                  pos='0.36 0 0.47', size='0.035 0.15 0.12', rgba=DARK, contype='0', conaffinity='0')
    ET.SubElement(base, 'geom', name='payload_compute_box', type='box', group=str(ROBOT_GROUP),
                  pos='-0.05 0 0.22', size='0.27 0.17 0.14', rgba=payload, contype='0', conaffinity='0')
    for sy in (-1, 1):
        ET.SubElement(base, 'geom', name=f'payload_antenna_{sy}', type='cylinder', group=str(ROBOT_GROUP),
                      fromto=f'0.15 {sy * 0.14} 0.36 0.15 {sy * 0.14} 0.52', size='0.008', rgba=DARK,
                      contype='0', conaffinity='0')
    roll, pitch, yaw = mount_rpy
    if roll or pitch:
        raise ValueError('Only yaw-only lidar mounting is implemented')
    ET.SubElement(base, 'site', name='lidar_link', pos=_fmt(mount_xyz), quat=_fmt(_yaw_quat(yaw)), size='0.02')


def build_hall(mount_xyz=(0.342, 0.0, 0.18), mount_rpy=(0.0, 0.0, 0.0), people=True):
    """Return (mjcf_xml, objects). Hall interior is x in [0, 12], y in [0, 10] metres."""
    root = ET.parse(ASSETS / 'upstream.xml').getroot()
    root.set('model', 'ULTRON_B2_lidar_hall_sim')
    base = _prepare_robot(root)
    _add_payload(base, mount_xyz, mount_rpy)
    ET.SubElement(root, 'option', timestep='0.01', gravity='0 0 -9.81')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.SubElement(visual, 'headlight', ambient='.45 .45 .45', diffuse='.6 .6 .6')
    ET.SubElement(visual, 'map', znear='0.01', zfar='40')
    assets = ET.SubElement(root, 'asset')
    ET.SubElement(assets, 'texture', name='tiles', type='2d', builtin='checker',
                  rgb1='.60 .30 .22', rgb2='.55 .27 .20', width='256', height='256')
    ET.SubElement(assets, 'material', name='tile_mat', texture='tiles', texrepeat='24 20', reflectance='.02')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='6 5 6', dir='0 0 -1', diffuse='.7 .7 .7')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='8 7 .1', pos='6 5 0',
                  material='tile_mat', group=str(SCENE_GROUP))
    objects = []
    wall = '.90 .90 .88 1'

    def static_box(name, kind, pos, half, color=wall):
        _box(world, name, pos, half, color)
        x, y = pos[:2]
        objects.append(HallObject(name, kind, (x, y), 0.0, _footprint((x, y), 0.0, half[0], half[1])))

    # Perimeter walls (3 m) with door recesses and wall fixtures that break symmetry.
    static_box('wall_south', 'WALL', (6, -0.1, 1.5), (6.2, 0.1, 1.5))
    static_box('wall_north', 'WALL', (6, 10.1, 1.5), (6.2, 0.1, 1.5))
    static_box('wall_west', 'WALL', (-0.1, 5, 1.5), (0.1, 5.2, 1.5))
    static_box('wall_east_a', 'WALL', (12.1, 2.25, 1.5), (0.1, 2.35, 1.5))
    static_box('wall_east_b', 'WALL', (12.1, 7.75, 1.5), (0.1, 2.35, 1.5))
    static_box('door_east', 'DOOR', (12.25, 5.0, 1.05), (0.05, 0.9, 1.05), '.15 .25 .55 1')  # recessed double door
    static_box('door_north', 'DOOR', (3.0, 10.02, 1.05), (0.5, 0.02, 1.05), '.25 .27 .30 1')
    static_box('hose_cabinet', 'WALL_FIXTURE', (0.12, 3.2, 1.2), (0.12, 0.35, 0.40), '.80 .10 .10 1')
    static_box('first_aid_box', 'WALL_FIXTURE', (11.93, 8.6, 1.3), (0.07, 0.2, 0.18), '.95 .45 .10 1')
    static_box('radiator', 'WALL_FIXTURE', (7.5, 9.93, 0.5), (0.8, 0.07, 0.3), '.95 .95 .95 1')
    for i, (x, y) in enumerate(((4.2, 8.3), (8.8, 1.6))):
        _geom(world, f'pillar_{i}', 'cylinder', '.70 .72 .74 1', pos=(x, y, 1.5), size=(0.22, 1.5))
        objects.append(HallObject(f'pillar_{i}', 'PILLAR', (x, y), 0.0, _footprint((x, y), 0.0, 0.22, 0.22)))
    for i, (x, y) in enumerate(((4.75, 8.3), (9.35, 1.6))):
        _geom(world, f'bollard_{i}', 'cylinder', '.95 .80 .10 1', pos=(x, y, 0.45), size=(0.06, 0.45))
        objects.append(HallObject(f'bollard_{i}', 'BOLLARD', (x, y), 0.0, _footprint((x, y), 0.0, 0.06, 0.06)))
    # Table with legs, stacked chairs, rolling cart, bin, extinguisher.
    table = ET.SubElement(world, 'body', name='table', pos='10.6 8.9 0')
    _box(table, 'table_top', (0, 0, 0.74), (0.8, 0.4, 0.02), '.85 .78 .62 1')
    for sx in (-1, 1):
        for sy in (-1, 1):
            _box(table, f'table_leg_{sx}_{sy}', (sx * 0.75, sy * 0.35, 0.36), (0.02, 0.02, 0.36), DARK)
    objects.append(HallObject('table', 'TABLE', (10.6, 8.9), 0.0, _footprint((10.6, 8.9), 0.0, 0.8, 0.4)))
    static_box('chair_stack', 'CHAIRS', (0.6, 8.9, 0.5), (0.25, 0.25, 0.5), '.80 .60 .35 1')
    static_box('cart', 'CART', (1.2, 6.5, 0.55), (0.45, 0.65, 0.55), '.75 .76 .78 1')
    static_box('bin', 'BIN', (11.6, 0.5, 0.45), (0.2, 0.2, 0.45), DARK)
    _geom(world, 'extinguisher', 'cylinder', '.85 .10 .10 1', pos=(11.75, 1.1, 0.3), size=(0.08, 0.3))
    objects.append(HallObject('extinguisher', 'EXTINGUISHER', (11.75, 1.1), 0.0, _footprint((11.75, 1.1), 0.0, 0.08, 0.08)))
    static_box('robot_mat', 'MAT', (1.6, 1.5, 0.008), (0.5, 1.0, 0.008), '.15 .15 .16 1')
    if people:
        for i, (x, y) in enumerate(((7.2, 8.6), (11.0, 4.0))):
            _geom(world, f'person_{i}', 'capsule', '.25 .30 .45 1', pos=(x, y, 0.9), size=(0.2, 0.7))
            objects.append(HallObject(f'person_{i}', 'PERSON_STATIC', (x, y), 0.0, _footprint((x, y), 0.0, 0.2, 0.2)))
    # Ladders: 7 in an approx. 10 x 10 m course, as in the briefing (6-8 ladders).
    objects += [
        layher_aframe(world, 'ladder_01', (4.0, 4.0), 0.0),
        layher_aframe(world, 'ladder_02', (8.0, 6.3), math.radians(30)),
        combination_aframe(world, 'ladder_03', (6.2, 1.9), math.radians(90)),
        leaning_ladder(world, 'ladder_04', (6.0, 9.15), 0.0),
        leaning_ladder(world, 'ladder_05', (0.85, 5.0), math.radians(-90)),
        trestle(world, 'ladder_06', (2.6, 7.4), math.radians(15)),
        trestle(world, 'ladder_07', (10.6, 6.2), math.radians(90)),
    ]
    return ET.tostring(root, encoding='unicode'), objects


class HallSim:
    """Kinematic B2 placement in the hall. Not a walking controller."""

    def __init__(self, mount_xyz=(0.342, 0.0, 0.18), mount_rpy=(0.0, 0.0, 0.0), people=True):
        import mujoco
        self.mujoco = mujoco
        self.xml, self.objects = build_hall(mount_xyz, mount_rpy, people)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.lidar_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_link')
        base = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        # Robot bodies are base_link and its descendants (legs).
        self.robot_bodies = {b for b in range(self.model.nbody)
                             if b == base or self._descends(b, base)}
        self.robot_geoms = {g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] in self.robot_bodies}
        self.pose = (1.6, 1.5, math.pi / 2)
        self.place(*self.pose)

    def _descends(self, body, ancestor):
        while body > 0:
            body = int(self.model.body_parentid[body])
            if body == ancestor:
                return True
        return False

    def place(self, x, y, yaw):
        """Set the true base pose (simulation ground truth) and recompute kinematics."""
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('Nonfinite pose')
        self.pose = (float(x), float(y), float(yaw))
        self.data.qpos[:7] = [x, y, BASE_HEIGHT_M, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        for j in range(1, self.model.njnt):
            name = self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, j)
            adr = self.model.jnt_qposadr[j]
            self.data.qpos[adr] = 0.65 if 'thigh' in name else -1.3 if 'calf' in name else 0.0
        self.mujoco.mj_forward(self.model, self.data)
