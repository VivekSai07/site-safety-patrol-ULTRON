"""Map manager: versioned save/load of 2D (+ optional 3D) maps with hashes; publishes ``map/info``.

Services (relative; the bringup namespace applies):
- ``map/save`` (ultron_b2_interfaces/SaveMap): writes ``<map_root>/<map_id>/v<N>/`` with
  ``map.pgm``/``map.yaml`` from the latest ``map`` OccupancyGrid (map_saver trinary semantics,
  free_thresh 0.196 so unknown stays unknown), the slam_toolbox pose graph
  (``slam_toolbox/serialize_map`` -> ``map.posegraph``/``map.data``) and, with ``save_3d``,
  the MOLA metric map (``map_save`` -> ``map3d*``). ``map_meta.json`` lists SHA-256 of every
  file. Versions are never overwritten.
- ``map/load`` (ultron_b2_interfaces/LoadMap): resolves map_id/version or yaml_path, verifies the
  hashes (refuses on mismatch), asks slam_toolbox to ``deserialize_map`` (localize at
  ``initial_pose``) when that service exists, and publishes the MapInfo.

``initial_map`` (yaml path, ``map_id`` or ``map_id:version``) announces a map loaded by
slam_toolbox at launch (``map_file_name``), after the same hash check. A map whose hashes do not
verify is never announced, so localization_monitor never reports VALID against it.
"""
from datetime import datetime
import os
from pathlib import Path
import threading

import numpy as np

from . import map_store

OCCUPIED_THRESH = 0.65
MAP_SAVER_FREE_THRESH = 0.25   # nav2 map_saver default used for the pixel classes


def grid_to_pgm(width, height, data, free_thresh=MAP_SAVER_FREE_THRESH, occupied_thresh=OCCUPIED_THRESH):
    """OccupancyGrid data -> PGM rows (top row = max y), exactly like nav2 map_saver trinary."""
    cells = np.asarray(data, dtype=np.int16).reshape(height, width)
    free_i, occ_i = int(round(free_thresh * 100)), int(round(occupied_thresh * 100))
    img = np.full(cells.shape, map_store.UNKNOWN_PIXEL, dtype=np.uint8)
    valid = (cells >= 0) & (cells <= 100)
    img[valid & (cells <= free_i)] = 254
    img[valid & (cells >= occ_i)] = 0
    return img[::-1]


def write_grid(directory, info, data, name='map'):
    """Write ``name.pgm`` + ``name.yaml``; returns the yaml path. Rotated origins are refused."""
    q = info.origin.orientation
    if abs(q.z) > 1e-9 or abs(q.x) > 1e-9 or abs(q.y) > 1e-9:
        raise map_store.MapStoreError('rotated map origin not supported')
    img = grid_to_pgm(info.width, info.height, data)
    directory = Path(directory)
    header = f'P5\n# ULTRON map_manager (map_saver trinary)\n{info.width} {info.height}\n255\n'.encode()
    (directory / f'{name}.pgm').write_bytes(header + img.tobytes())
    o = info.origin.position
    yaml_path = directory / f'{name}.yaml'
    yaml_path.write_text(
        f'image: {name}.pgm\nmode: trinary\nresolution: {info.resolution:.6g}\n'
        f'origin: [{o.x:.6f}, {o.y:.6f}, 0]\nnegate: 0\noccupied_thresh: {OCCUPIED_THRESH}\n'
        f'free_thresh: {map_store.SAFE_FREE_THRESH}\n')
    return yaml_path


def parse_map_ref(ref):
    """``initial_map`` -> (map_id, version, yaml_path)."""
    if not ref:
        return '', 0, ''
    if ref.endswith('.yaml') or os.sep in ref:
        return '', 0, ref
    if ':' in ref:
        map_id, version = ref.rsplit(':', 1)
        return map_id, int(version.lstrip('v')), ''
    return ref, 0, ''


def default_map_root():
    return os.environ.get('ULTRON_MAP_ROOT') or str(Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / 'ultron_maps')


class MapManager:
    def __init__(self, node):
        from nav_msgs.msg import OccupancyGrid
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from ultron_b2_interfaces.msg import MapInfo
        from ultron_b2_interfaces.srv import LoadMap, SaveMap
        self.node = node
        p = lambda name, default: node.declare_parameter(name, default).value
        self.root = Path(os.path.expanduser(p('map_root', '') or default_map_root()))
        self.prefix = p('map_id_prefix', 'ultron-map')
        self.source = p('source', 'slam_toolbox')
        self.provenance = p('provenance', 'SLAM_TOOLBOX_ESTIMATE')
        slam = p('slam_service_prefix', 'slam_toolbox')
        self.mola_save_name = p('mola_map_save_service', 'map_save')
        self.initial_pose = list(p('initial_pose', [0.0, 0.0, 0.0]))
        self.timeout = p('service_timeout_s', 30.0)
        self.MapInfo, self.SaveMap, self.LoadMap = MapInfo, SaveMap, LoadMap
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info_pub = node.create_publisher(MapInfo, 'map/info', latched)
        self.grid, self.grid_lock = None, threading.Lock()
        node.create_subscription(OccupancyGrid, p('map_topic', 'map'), self._on_grid, latched)
        clients = ReentrantCallbackGroup()
        from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph
        self.Serialize, self.Deserialize = SerializePoseGraph, DeserializePoseGraph
        self.serialize = node.create_client(SerializePoseGraph, f'{slam}/serialize_map', callback_group=clients)
        self.deserialize = node.create_client(DeserializePoseGraph, f'{slam}/deserialize_map', callback_group=clients)
        self.mola_save = None
        try:
            from mola_msgs.srv import MapSave
            self.MolaSave = MapSave
            self.mola_save = node.create_client(MapSave, self.mola_save_name, callback_group=clients)
        except ImportError:
            pass
        services = MutuallyExclusiveCallbackGroup()
        node.create_service(SaveMap, 'map/save', self._save, callback_group=services)
        node.create_service(LoadMap, 'map/load', self._load, callback_group=services)
        initial = p('initial_map', '')
        if initial:
            map_id, version, yaml_path = parse_map_ref(initial)
            try:
                ok, message, _info = self._announce(map_id, version, yaml_path, deserialize=False)
            except (OSError, ValueError) as exc:
                ok, message = False, f'not announced: {exc}'
            log = node.get_logger().info if ok else node.get_logger().error
            log(f'initial map {initial}: {message}')
        node.get_logger().info(f'map_manager: map_root={self.root}')

    def _on_grid(self, msg):
        with self.grid_lock:
            self.grid = msg

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=2.0):
            return None, f'service {client.srv_name} unavailable'
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self.timeout):
            return None, f'service {client.srv_name} timed out after {self.timeout} s'
        return future.result(), ''

    def _info_msg(self, d):
        m = self.MapInfo()
        m.schema_version = self.MapInfo.SCHEMA_VERSION
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        for key in ('map_id', 'map_version', 'yaml_path', 'image_sha256', 'posegraph_path', 'map3d_path',
                    'map3d_sha256', 'resolution', 'width', 'height', 'free_thresh', 'occupied_thresh', 'source'):
            setattr(m, key, type(getattr(m, key))(d[key]))
        m.origin.position.x, m.origin.position.y = d['origin'][:2]
        qz, qw = map_store.yaw_to_quaternion_z_w(d['origin'][2])
        m.origin.orientation.z, m.origin.orientation.w = qz, qw
        if d['created_at']:
            created = datetime.fromisoformat(d['created_at'])
            m.created_at.sec = int(created.timestamp())
        return m

    # ---------------------------------------------------------------- save
    def _save(self, request, response):
        response.schema_version = self.SaveMap.Request.SCHEMA_VERSION
        try:
            response.success, response.message, info = self._do_save(request)
            if info is not None:
                response.info = info
        except Exception as exc:
            response.success, response.message = False, f'save failed: {exc}'
        log = self.node.get_logger().info if response.success else self.node.get_logger().error
        log(f'map/save: {response.message}')
        return response

    def _do_save(self, request):
        if request.schema_version and request.schema_version != self.SaveMap.Request.SCHEMA_VERSION:
            return False, f'schema {request.schema_version!r} != {self.SaveMap.Request.SCHEMA_VERSION}', None
        with self.grid_lock:
            grid = self.grid
        if grid is None:
            return False, 'no map received on the map topic yet', None
        root = Path(request.directory).expanduser() if request.directory else self.root
        map_id = map_store.check_map_id(request.map_id or map_store.new_map_id(self.prefix))
        version, directory = map_store.allocate_version(root, map_id)
        write_grid(directory, grid.info, grid.data)
        problems = []
        req = self.Serialize.Request()
        req.filename = str(directory / 'map')
        result, err = self._call(self.serialize, req)
        if result is None or not (directory / 'map.posegraph').is_file():
            problems.append(f'pose graph not saved ({err or "serialize_map returned without files"})')
        if request.save_3d:
            if self.mola_save is None:
                problems.append('3D map requested but mola_msgs is not available')
            else:
                req3 = self.MolaSave.Request()
                req3.map_path = str(directory / 'map3d')
                result3, err3 = self._call(self.mola_save, req3)
                if result3 is None or not result3.success:
                    problems.append(f'3D map not saved ({err3 or result3.error_message})')
        stamp = grid.header.stamp.sec + grid.header.stamp.nanosec * 1e-9
        map_store.write_meta(directory, map_id, version, source=self.source, provenance=self.provenance, extra={
            'grid_stamp': stamp, 'free_thresh_rewritten_to_keep_unknown': True,
            'free_thresh_note': 'map_saver default 0.25 reads unknown (205) as free; written as 0.196',
            'save_problems': problems})
        info = self._info_msg(map_store.info_dict(directory))
        self.info_pub.publish(info)
        ok = not problems
        return ok, (f'saved {map_id} v{version} in {directory}' + ('' if ok else '; ' + '; '.join(problems))), info

    # ---------------------------------------------------------------- load
    def _load(self, request, response):
        response.schema_version = self.LoadMap.Request.SCHEMA_VERSION
        try:
            response.success, response.message, info = self._announce(request.map_id, request.map_version,
                                                                       request.yaml_path, deserialize=True)
            if info is not None:
                response.info = info
        except Exception as exc:
            response.success, response.message = False, f'load failed: {exc}'
        log = self.node.get_logger().info if response.success else self.node.get_logger().error
        log(f'map/load: {response.message}')
        return response

    def _announce(self, map_id, version, yaml_path, deserialize):
        directory = map_store.resolve(self.root, map_id, version, yaml_path)
        problems = map_store.verify(directory)
        if problems:
            return False, f'{directory}: integrity check failed: {", ".join(problems)}', None
        d = map_store.info_dict(directory)
        if not d['yaml_path']:
            return False, f'{directory}: no 2D map', None
        if deserialize and d['posegraph_path'] and self.deserialize.service_is_ready():
            req = self.Deserialize.Request()
            req.filename = d['posegraph_path']
            req.match_type = self.Deserialize.Request.LOCALIZE_AT_POSE
            req.initial_pose.x, req.initial_pose.y, req.initial_pose.theta = map(float, self.initial_pose)
            result, err = self._call(self.deserialize, req)
            if result is None:
                return False, f'slam_toolbox deserialize_map failed: {err}', None
        info = self._info_msg(d)
        self.info_pub.publish(info)
        return True, f'announced {d["map_id"]} v{d["map_version"]} ({directory})', info


def main(argv=None):
    from .node_runtime import run
    run('map_manager', MapManager, argv, multithreaded=True)


__all__ = ['MapManager', 'grid_to_pgm', 'parse_map_ref', 'write_grid']
