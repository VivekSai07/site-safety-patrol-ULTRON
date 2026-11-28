import numpy as np
import pytest

from ultron_b2_navigation.zone_masks import grid_for_zone, make_masks, zone_from_dict

BOUNDARY = [[0, 0], [10, 0], [10, 6], [0, 6]]
KEEPOUT = [[4, 2], [6, 2], [6, 4], [4, 4]]


def cell(grid, x, y):
    return int((y - grid.origin_y) / grid.resolution), int((x - grid.origin_x) / grid.resolution)


def test_keepout_and_speed_masks():
    g = grid_for_zone(BOUNDARY, 0.1, 1.0)
    keep, speed = make_masks(g, BOUNDARY, [KEEPOUT], slow_band_m=1.0, slow_percent=50)
    assert keep[cell(g, -0.5, 3)] == 100          # outside boundary
    assert keep[cell(g, 5, 3)] == 100             # inside keepout
    assert keep[cell(g, 2, 3)] == 0 and speed[cell(g, 2, 3)] == 0
    assert speed[cell(g, 0.5, 3)] == 50           # near boundary edge
    assert speed[cell(g, 3.5, 3)] == 50           # near keepout edge
    assert speed[cell(g, 5, 3)] == 100


def test_zone_schema_rejected():
    with pytest.raises(ValueError):
        zone_from_dict({'schema': 'other', 'zone_id': 'z', 'map_id': 'm', 'map_version': 1, 'boundary': BOUNDARY})
    with pytest.raises(ValueError):
        zone_from_dict({'zone_id': 'z', 'map_id': 'm', 'map_version': 1, 'boundary': [[0, 0], [1, np.nan], [1, 1]]})
