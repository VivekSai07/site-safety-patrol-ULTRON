"""View-pose planning for ladder inspection (port of src/ultron/viewpoints.py to occupancy grids).

Candidates sit on rings of PROPOSED standoff radii around the ladder, facing it. With a known
ladder yaw, free-standing types get four sides (0/90/180/270 deg) and leaning types only the
front sector (no automatic orbit behind wall ladders, as in viewpoints.py). With unknown yaw a
ring of ``unknown_yaw_angles`` is used. Gates: inside map, inside zone (not keep-out), known
free with robot clearance, reachable from the robot, line of sight to the ladder, ladder within
the camera horizontal FOV and its height within vertical FOV at that range. Rejections are
returned with reasons. Validated only on the simulation grid; not proof of physical clearance.
"""
from dataclasses import dataclass, field
import math

FREE_STANDING = ('LAYHER_A_FRAME_STEPLADDER', 'COMBINATION_LADDER_A_FRAME', 'A_FRAME_STEPLADDER', 'TRESTLE')
LEANING = ('LEANING_COMBINATION_LADDER', 'LEANING_LADDER')


@dataclass
class ViewPose:
    x: float
    y: float
    yaw: float
    standoff_m: float
    angle_deg: float
    score: float = 0.0
    validated_for: str = 'SIMULATION_GRID_ONLY'


@dataclass
class ViewPlan:
    asset_id: str
    feasible: list = field(default_factory=list)
    rejected: list = field(default_factory=list)   # (ViewPose, reason)

    @property
    def reachable(self):
        return bool(self.feasible)


@dataclass
class ViewPlannerConfig:
    standoffs_m: tuple = (1.8, 2.2, 1.5)          # PROPOSED
    robot_radius_m: float = 0.45                  # PROPOSED B2 half-width + margin, TO_VERIFY
    camera_height_m: float = 0.55                 # PROPOSED head camera height, TO_VERIFY with F/URDF
    hfov_deg: float = 89.0                        # colour 1280x720 fx 643.6/fy 642.9 (BAG_FACTS) => ~89.7x58.5 deg
    vfov_deg: float = 58.0
    ladder_height_m: float = 2.0
    min_visible_height_fraction: float = 0.6      # PROPOSED
    unknown_yaw_angles: tuple = (0, 45, 90, 135, 180, 225, 270, 315)
    ladder_clear_radius_m: float = 0.6            # ladder footprint ignored for line of sight


def _angles(ladder_type, yaw, cfg):
    if yaw is None or not math.isfinite(yaw):
        return [math.radians(a) for a in cfg.unknown_yaw_angles], cfg.unknown_yaw_angles
    if ladder_type in LEANING:
        rel = (-90, -40, -140)
    else:
        rel = (-90, 90, 0, 180)
    return [yaw + math.radians(a) for a in rel], rel


def vertical_fit(standoff, cfg):
    """Fraction of the ladder height visible at ``standoff`` given the camera vertical FOV."""
    half = math.radians(cfg.vfov_deg) / 2.0
    lo = max(0.0, cfg.camera_height_m - standoff * math.tan(half))
    hi = min(cfg.ladder_height_m, cfg.camera_height_m + standoff * math.tan(half))
    return max(0.0, hi - lo) / cfg.ladder_height_m


def plan_views(asset_id, ladder_xy, ladder_type, grid, zone, robot_xy, *, yaw=None, reach=None,
               cfg=None, blacklist=()):
    cfg = cfg or ViewPlannerConfig()
    lx, ly = ladder_xy
    plan = ViewPlan(asset_id)
    trav = grid.traversable(cfg.robot_radius_m)
    if reach is None:
        reach = grid.reachable_from(robot_xy, cfg.robot_radius_m)
    angles, labels = _angles(ladder_type, yaw, cfg)
    for standoff in cfg.standoffs_m:
        frac = vertical_fit(standoff, cfg)
        for theta, label in zip(angles, labels):
            x, y = lx + standoff * math.cos(theta), ly + standoff * math.sin(theta)
            vp = ViewPose(x, y, math.atan2(ly - y, lx - x), standoff, float(label))
            c = grid.cell(x, y)
            if c is None:
                plan.rejected.append((vp, 'OUTSIDE_MAP'))
            elif not zone.contains(x, y):
                plan.rejected.append((vp, 'OUTSIDE_ZONE_OR_KEEPOUT'))
            elif grid.unknown[c]:
                plan.rejected.append((vp, 'UNKNOWN_SPACE'))
            elif not trav[c]:
                plan.rejected.append((vp, 'NO_CLEARANCE'))
            elif any(math.hypot(x - bx, y - by) < 0.3 for bx, by in blacklist):
                plan.rejected.append((vp, 'PREVIOUSLY_FAILED'))
            elif not reach[c]:
                plan.rejected.append((vp, 'UNREACHABLE'))
            elif not grid.line_of_sight((x, y), (lx, ly), ignore_radius_m=cfg.ladder_clear_radius_m):
                plan.rejected.append((vp, 'NO_LINE_OF_SIGHT'))
            elif frac < cfg.min_visible_height_fraction:
                plan.rejected.append((vp, 'LADDER_NOT_IN_VERTICAL_FOV'))
            else:
                vp.score = math.hypot(x - robot_xy[0], y - robot_xy[1]) - 0.5 * frac
                plan.feasible.append(vp)
    plan.feasible.sort(key=lambda v: v.score)
    return plan
