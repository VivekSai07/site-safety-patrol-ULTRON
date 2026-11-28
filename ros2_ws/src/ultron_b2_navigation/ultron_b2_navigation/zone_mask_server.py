"""zone_mask_server: PatrolZone -> latched keepout/speed filter masks for Nav2 costmap filters.

Service ``nav/set_patrol_zone`` (ultron_b2_interfaces/SetPatrolZone); optional ``zone_file`` (YAML,
``ultron.patrol_zone/v1``) at startup. Publishes (transient local) ``nav/keepout_mask``,
``nav/speed_mask`` (OccupancyGrid) and ``nav/keepout_filter_info``, ``nav/speed_filter_info``
(nav2_msgs/CostmapFilterInfo). Without a zone it publishes unrestricted (all-zero) masks and warns.
"""
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from ultron_b2_interfaces.srv import SetPatrolZone

from .zone_masks import Grid, SLOW_BAND_M, SLOW_PERCENT, grid_for_zone, make_masks, zone_from_dict

KEEPOUT_FILTER, SPEED_FILTER_PERCENT = 0, 1


def grid_from_map_yaml(path):
    import yaml
    meta = yaml.safe_load(Path(path).read_text())
    with open(Path(path).parent / meta['image'], 'rb') as f:
        header = []
        while len(header) < 4:            # P5 w h maxval (comments skipped)
            for tok in f.readline().split():
                if tok.startswith(b'#'):
                    break
                header.append(tok)
    return Grid(float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1]),
                int(header[1]), int(header[2]))


class ZoneMaskServer(Node):
    def __init__(self):
        super().__init__('zone_mask_server')
        self.declare_parameter('zone_file', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_meta_path', '')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('margin_m', 2.0)
        self.declare_parameter('slow_band_m', SLOW_BAND_M)
        self.declare_parameter('slow_percent', SLOW_PERCENT)
        self.declare_parameter('mask_frame', 'map')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {name: self.create_publisher(OccupancyGrid, f'nav/{name}_mask', latched)
                     for name in ('keepout', 'speed')}
        self.info_pubs = {name: self.create_publisher(CostmapFilterInfo, f'nav/{name}_filter_info', latched)
                          for name in ('keepout', 'speed')}
        self.create_service(SetPatrolZone, 'nav/set_patrol_zone', self._on_set)
        self.map_id, self.map_version = self._loaded_map()
        self.zone_id = None
        zone_file = str(self.get_parameter('zone_file').value)
        if zone_file:
            import yaml
            ok, why = self.apply(zone_from_dict(yaml.safe_load(Path(zone_file).read_text())))
            (self.get_logger().info if ok else self.get_logger().error)(f'zone_file {zone_file}: {why}')
        if self.zone_id is None:
            self.get_logger().warn('no patrol zone set: publishing unrestricted masks (map bounds only)')
            self._publish_unrestricted()

    def _loaded_map(self):
        meta = str(self.get_parameter('map_meta_path').value)
        if not meta:
            return None, None
        p = Path(meta)
        data = json.loads((p / 'map_meta.json' if p.is_dir() else p).read_text())
        return str(data['map_id']), int(data['map_version'])

    def _grid(self, boundary):
        map_yaml = str(self.get_parameter('map_yaml').value)
        if map_yaml:
            return grid_from_map_yaml(map_yaml)
        return grid_for_zone(boundary, float(self.get_parameter('resolution').value),
                             float(self.get_parameter('margin_m').value))

    def apply(self, zone):
        if self.map_id is not None and (zone['map_id'], zone['map_version']) != (self.map_id, self.map_version):
            return False, (f'MAP_MISMATCH: zone {zone["map_id"]} v{zone["map_version"]}, '
                           f'loaded {self.map_id} v{self.map_version}')
        grid = self._grid(zone['boundary'])
        keepout, speed = make_masks(grid, zone['boundary'], zone['keepout'],
                                    float(self.get_parameter('slow_band_m').value),
                                    int(self.get_parameter('slow_percent').value))
        self._publish(grid, keepout, speed)
        self.zone_id = zone['zone_id']
        return True, f'zone {zone["zone_id"]}: {int((keepout == 0).sum())} allowed cells of {keepout.size}'

    def _publish_unrestricted(self):
        import numpy as np
        grid = Grid(0.5, -0.5, -0.5, 2, 2)
        z = np.zeros((2, 2), dtype=np.int8)
        self._publish(grid, z, z)

    def _publish(self, grid, keepout, speed):
        stamp = self.get_clock().now().to_msg()
        frame = str(self.get_parameter('mask_frame').value)
        for name, data in (('keepout', keepout), ('speed', speed)):
            m = OccupancyGrid()
            m.header.stamp, m.header.frame_id = stamp, frame
            m.info.resolution = float(grid.resolution)
            m.info.width, m.info.height = int(grid.width), int(grid.height)
            m.info.origin.position.x, m.info.origin.position.y = grid.origin_x, grid.origin_y
            m.info.origin.orientation.w = 1.0
            m.data = data.reshape(-1).tolist()
            self.pubs[name].publish(m)
            info = CostmapFilterInfo()
            info.header.stamp, info.header.frame_id = stamp, frame
            info.type = KEEPOUT_FILTER if name == 'keepout' else SPEED_FILTER_PERCENT
            info.filter_mask_topic = self.pubs[name].topic_name   # fully qualified
            info.base, info.multiplier = 0.0, 1.0
            self.info_pubs[name].publish(info)

    def _on_set(self, req, res):
        res.schema_version = 'ultron.patrol_zone/v1'
        try:
            zone = zone_from_dict({
                'zone_id': req.zone_id, 'map_id': req.map_id, 'map_version': req.map_version,
                'boundary': [(p.x, p.y) for p in req.boundary.points],
                'keepout': [[(p.x, p.y) for p in k.points] for k in req.keepout]})
            res.success, res.message = self.apply(zone)
        except (ValueError, KeyError) as exc:
            res.success, res.message = False, f'INVALID_ZONE: {exc}'
        (self.get_logger().info if res.success else self.get_logger().warn)(f'set_patrol_zone: {res.message}')
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ZoneMaskServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
