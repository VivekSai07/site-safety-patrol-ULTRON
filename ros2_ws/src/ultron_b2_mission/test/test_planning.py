"""Frontier bounding, coverage gaps, view-pose gates and zone loading."""
import math

from mission_fixtures import room, zone_for
from ultron_b2_mission.coverage_reporter import compute_coverage
from ultron_b2_mission.frontier_explorer import find_frontiers
from ultron_b2_mission.view_pose_planner import plan_views
from ultron_b2_mission.zone import PatrolZone, load_route, map_from_world


def test_frontiers_stay_inside_zone_and_ignore_unknown_outside():
    # Unknown on the right half; the zone only covers x < 5.5, so only the in-zone strip may pull.
    grid = room(unknown_rect=(5.0, 0.1, 9.9, 7.9))
    zone = PatrolZone('Z', [(0, 0), (5.5, 0), (5.5, 8), (0, 8)])
    reach = grid.reachable_from((1.0, 1.0), 0.35)
    fr, unr = find_frontiers(grid, zone, (1.0, 1.0), reach=reach)
    assert fr
    for f in fr + unr:
        assert zone.contains(*f.centroid)
        if f.goal:
            assert zone.contains(f.goal[0], f.goal[1])
    # Zone ending before the unknown region: nothing to explore.
    zone2 = PatrolZone('Z', [(0, 0), (4.9, 0), (4.9, 8), (0, 8)])
    assert find_frontiers(grid, zone2, (1.0, 1.0)) == ([], [])


def test_frontier_keepout_excluded():
    grid = room(unknown_rect=(5.0, 0.1, 9.9, 7.9))
    zone = PatrolZone('Z', [(0, 0), (10, 0), (10, 8), (0, 8)], keepout=[[(4.5, 0), (10, 0), (10, 8), (4.5, 8)]])
    assert find_frontiers(grid, zone, (1.0, 1.0)) == ([], [])


def test_frontier_behind_wall_is_reported_unreachable_not_dropped():
    # Wall at x=5 separates the robot from a pocket containing unknown space.
    grid = room(unknown_rect=(7.0, 0.1, 9.9, 7.9), walls=[(5.0, 0.0, 5.2, 8.0)])
    zone = zone_for()
    fr, unr = find_frontiers(grid, zone, (1.0, 1.0))
    assert not fr
    assert unr and all(not f.reachable and f.goal is None for f in unr)


def test_frontier_blacklist():
    grid = room(unknown_rect=(6.0, 0.1, 9.9, 7.9))
    fr, _ = find_frontiers(grid, zone_for(), (1.0, 1.0))
    g = fr[0].goal
    fr2, unr2 = find_frontiers(grid, zone_for(), (1.0, 1.0), blacklist=[fr[0].centroid])
    assert fr[0].frontier_id in [f.frontier_id for f in unr2]
    assert all(f.frontier_id != fr[0].frontier_id for f in fr2)
    assert g is not None


def test_coverage_reports_unknown_and_unreachable_separately_with_gaps():
    grid = room(unknown_rect=(7.0, 0.1, 9.9, 3.0), walls=[(5.0, 4.0, 5.2, 8.0), (5.0, 4.0, 10.0, 4.2)])
    # Pocket x>5.2, y>4.2 is free but enclosed; unknown strip bottom-right is explorable.
    grid.data  # noqa: B018
    zone = zone_for()
    reach = grid.reachable_from((1.0, 1.0), 0.35)
    cov = compute_coverage(grid, zone, reach, ladders_unreachable=['LAD-002'], ladders_pending=['LAD-003'])
    assert cov.unknown_area_m2 > 0
    assert cov.unreachable_area_m2 > 0
    assert cov.observed_area_m2 + cov.unknown_area_m2 <= cov.zone_area_m2 + 1e-6
    assert not cov.complete
    assert any(g.startswith('GAP-UNK-') for g in cov.gap_ids)
    assert any(g.startswith('GAP-UNR-') for g in cov.gap_ids)
    assert 'LADDER_UNREACHABLE:LAD-002' in cov.gap_ids and 'LADDER_PENDING:LAD-003' in cov.gap_ids
    assert len(cov.gap_polygons) == len([g for g in cov.gap_ids if g.startswith('GAP-')])
    # Explorable gap polygon lies in the unknown strip.
    i = cov.gap_ids.index('GAP-UNK-1')
    xs = [p[0] for p in cov.gap_polygons[i]]
    assert min(xs) >= 6.9


def test_coverage_complete_only_without_gaps():
    grid = room()
    zone = zone_for()
    reach = grid.reachable_from((1.0, 1.0), 0.35)
    cov = compute_coverage(grid, zone, reach)
    assert cov.complete and cov.coverage_fraction == 1.0 and not cov.gap_ids
    cov2 = compute_coverage(grid, zone, reach, ladders_missing=['LAD-001'])
    assert not cov2.complete and cov2.gap_ids == ['LADDER_MISSING:LAD-001']


def test_view_planner_gates_and_reasons():
    grid = room(walls=[(4.0, 0.0, 4.2, 3.2)])
    zone = PatrolZone('Z', [(0, 0), (10, 0), (10, 8), (0, 8)], keepout=[[(6.0, 3.0), (8.0, 3.0), (8.0, 5.0), (6.0, 5.0)]])
    plan = plan_views('L', (5.0, 4.0), 'TRESTLE', grid, zone, (1.0, 6.0), yaw=0.0)
    assert plan.feasible
    reasons = {r for _, r in plan.rejected}
    assert 'OUTSIDE_ZONE_OR_KEEPOUT' in reasons
    for v in plan.feasible:
        assert zone.contains(v.x, v.y)
        assert abs(math.atan2(4.0 - v.y, 5.0 - v.x) - v.yaw) < 1e-9
    # Leaning ladders: only the front sector, never behind the wall side.
    lean = plan_views('L', (5.0, 7.0), 'LEANING_COMBINATION_LADDER', grid, zone, (1.0, 6.0), yaw=math.pi / 2)
    for v in lean.feasible + [p for p, _ in lean.rejected]:
        assert v.angle_deg in (-90, -40, -140)


def test_view_planner_no_line_of_sight():
    grid = room(walls=[(3.0, 3.0, 7.0, 3.2)])
    zone = zone_for()
    plan = plan_views('L', (5.0, 4.0), 'TRESTLE', grid, zone, (5.0, 1.5), yaw=0.0)
    assert any(r == 'NO_LINE_OF_SIGHT' or r == 'UNREACHABLE' for _, r in plan.rejected)
    assert all(v.y > 3.2 for v in plan.feasible)


def test_world_route_conversion_and_zone_config(tmp_path_local):
    route = tmp_path_local / 'r.yaml'
    route.write_text('schema: ultron.sim_route/v1\nwaypoints:\n  - [1.6, 1.5]\n  - [1.6, 3.5]\n')
    pts = load_route(str(route), (1.6, 1.5, math.pi / 2))
    assert abs(pts[0][0]) < 1e-9 and abs(pts[0][1]) < 1e-9
    assert abs(pts[1][0] - 2.0) < 1e-9 and abs(pts[1][1]) < 1e-9
    assert map_from_world([(1.6, 1.5)], (1.6, 1.5, 0.0)) == [(0.0, 0.0)]
