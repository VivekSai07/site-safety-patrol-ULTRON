"""Conservative occupancy-snapshot guard for kinematic simulation, NOT a B2 safety controller.

A real adapter must supply measured, synchronized, map-frame observations and an
independent watchdog. Empty snapshots mean observed clear, never missing sensors.
"""
from dataclasses import dataclass
import math
from .planning import Grid


@dataclass(frozen=True)
class ObstacleSnapshot:
    stamp: float
    map_id: str
    rectangles: tuple
    localization_valid: bool = True
    operator_enabled: bool = True


class MotionGuard:
    def __init__(self, grid, map_id, max_age=.3, deceleration=.6, latency=.2):
        self.grid, self.map_id = grid, map_id
        self.max_age, self.deceleration, self.latency = max_age, deceleration, latency
        if not all(math.isfinite(v) and v > 0 for v in (max_age, deceleration, latency)):
            raise ValueError('Positive finite guard parameters required')
        self.last_stamp = -math.inf

    def grid_for(self, snapshot, speed=0.):
        margin = abs(speed) * self.latency + speed * speed / (2 * self.deceleration)
        # Apply braking margin to dynamic geometry, retain static footprint inflation.
        dynamic = [(a-margin, b-margin, c+margin, d+margin) for a,b,c,d in snapshot.rectangles]
        return Grid(self.grid.bounds, self.grid.obstacles + dynamic,
                    radius=self.grid.radius, resolution=self.grid.resolution)

    def check(self, start, end, speed, now, snapshot):
        try:
            if snapshot is None:
                return False, 'NO_OBSTACLE_OBSERVATION'
            if not all(math.isfinite(v) for v in (*start, *end, speed, now, snapshot.stamp)):
                return False, 'INVALID_OBSERVATION'
            if snapshot.map_id != self.map_id or snapshot.localization_valid is not True:
                return False, 'LOCALIZATION_INVALID'
            if snapshot.operator_enabled is not True:
                return False, 'OPERATOR_DISABLED'
            if not 0 <= now - snapshot.stamp <= self.max_age or snapshot.stamp < self.last_stamp:
                return False, 'OBSTACLE_OBSERVATION_STALE_OR_REORDERED'
            for rect in snapshot.rectangles:
                if len(rect) != 4 or not all(math.isfinite(v) for v in rect) or rect[0] > rect[2] or rect[1] > rect[3]:
                    return False, 'INVALID_OBSERVATION'
            self.last_stamp = snapshot.stamp
            grid = self.grid_for(snapshot, speed)
            steps = max(1, math.ceil(math.dist(start[:2], end[:2]) / min(.05, grid.resolution / 2)))
            for i in range(steps + 1):
                point = [start[j] + (end[j]-start[j]) * i/steps for j in (0,1)]
                if not grid.free(point):
                    return False, 'DYNAMIC_OR_STATIC_CLEARANCE'
            return True, 'CLEAR'
        except (TypeError, ValueError, AttributeError, OverflowError):
            return False, 'INVALID_OBSERVATION'


class InjectedObstacleSensor:
    """Deterministic SIMULATION occupancy injection, not a rendered lidar sensor.

Introduces an obstacle ahead of the first translational command. `stale` loses
observations after its first sample. No hardware fallback to this provider exists.
"""
    def __init__(self, mode, map_id):
        self.mode, self.map_id = mode, map_id
        self.rectangle = None
        self.first_stamp = None

    def sample(self, now, pose, command):
        if self.first_stamp is None:
            self.first_stamp = now
        if self.mode == 'crossing' and self.rectangle is None and command.vx > .1:
            x = pose[0] + 1.4 * math.cos(pose[2])
            y = pose[1] + 1.4 * math.sin(pose[2])
            self.rectangle = (x-.15, y-.15, x+.15, y+.15)
        stamp = self.first_stamp if self.mode == 'stale' else now
        return ObstacleSnapshot(stamp, self.map_id, (self.rectangle,) if self.rectangle else ())
