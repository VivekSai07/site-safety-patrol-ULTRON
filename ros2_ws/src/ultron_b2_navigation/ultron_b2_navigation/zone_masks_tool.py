"""Offline: PatrolZone YAML -> keepout/speed mask PGM + map_server YAML (for costmap_filter_info_server setups).

  ros2 run ultron_b2_navigation zone_masks_tool zone.yaml OUT_DIR [--map-yaml map.yaml]
"""
import argparse
from pathlib import Path

import yaml

from .zone_masks import grid_for_zone, make_masks, write_pgm, zone_from_dict


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('zone', type=Path)
    ap.add_argument('out', type=Path)
    ap.add_argument('--map-yaml', type=Path, help='Use this map grid (resolution/origin/size)')
    a = ap.parse_args(argv)
    zone = zone_from_dict(yaml.safe_load(a.zone.read_text()))
    if a.map_yaml:
        from .zone_mask_server import grid_from_map_yaml
        grid = grid_from_map_yaml(a.map_yaml)
    else:
        grid = grid_for_zone(zone['boundary'])
    keepout, speed = make_masks(grid, zone['boundary'], zone['keepout'])
    a.out.mkdir(parents=True, exist_ok=True)
    for name, mask in (('keepout', keepout), ('speed', speed)):
        write_pgm(a.out / f'{name}_mask.pgm', mask)
        (a.out / f'{name}_mask.yaml').write_text(yaml.safe_dump({
            'image': f'{name}_mask.pgm', 'mode': 'scale', 'resolution': grid.resolution,
            'origin': [grid.origin_x, grid.origin_y, 0.0], 'negate': 0,
            'occupied_thresh': 1.0, 'free_thresh': 0.0}, sort_keys=False))
    print(f'wrote {a.out}/keepout_mask.* and speed_mask.* for zone {zone["zone_id"]}')


if __name__ == '__main__':
    main()
