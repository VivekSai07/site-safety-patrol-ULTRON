"""Collision/path-checked observation candidates; no automatic full orbit for wall ladders."""
import math


def candidate_views(target,ladder_type,grid,*,full_orbit=False,start=None,radius=2.):
    if not math.isfinite(radius) or radius<1.:
        raise ValueError('Observation radius must be finite and at least 1 m')
    x,y=target[:2]
    angles=(-90,-140,-40)
    if full_orbit and ladder_type in ('A_FRAME_STEPLADDER','PLATFORM_PODEST'):
        angles=(-90,-45,0,45,90,135,180,225)
    result=[]
    for degrees in angles:
        theta=math.radians(degrees)
        position=(x+radius*math.cos(theta),y+radius*math.sin(theta))
        if not grid.free(position):
            continue
        if start is not None and not grid.path(start,position):
            continue
        result.append({'position':position,'yaw':math.atan2(y-position[1],x-position[0]),
                       'angle_deg':degrees,'validated_for':'SIMULATION_GRID_ONLY'})
    return result
