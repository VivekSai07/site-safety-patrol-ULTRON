"""Kinematic base driver for the simulation. Not a walking controller; no hardware path.

Accepts body-frame velocity commands (from ``/ultron/sim/cmd_vel`` or a scripted route),
clamps speed and acceleration, stops immediately when commands go stale, and refuses any
step whose footprint would intersect the scene (the base stops and the event is counted).
True motion follows the clamped command exactly; odometry error is added separately.
"""
from dataclasses import dataclass
import math


@dataclass
class DriverState:
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    wz: float = 0.0
    collisions: int = 0
    timeouts: int = 0
    last_event: str = 'IDLE'


def _approach(current, target, max_delta):
    return current + max(-max_delta, min(max_delta, target - current))


class KinematicDriver:
    def __init__(self, params, start, is_free):
        self.p = params
        self.is_free = is_free
        self.state = DriverState(*map(float, start))
        if not is_free(self.state.x, self.state.y):
            raise ValueError(f'Start pose {start} collides with the scene')
        self.command = (0.0, 0.0)
        self.command_time = None
        self.route = []          # list of (vx, wz, duration) segments
        self.route_left = 0.0

    def set_command(self, vx, wz, now):
        if not all(math.isfinite(v) for v in (vx, wz, now)):
            raise ValueError('Nonfinite command')
        self.command = (float(vx), float(wz))
        self.command_time = float(now)

    def load_route(self, segments):
        """Segments of (vx, wz, duration) executed open-loop; commands are ignored meanwhile."""
        self.route = [tuple(map(float, s)) for s in segments]
        self.route_left = self.route[0][2] if self.route else 0.0

    @property
    def route_active(self):
        return bool(self.route)

    def _target(self, now, dt):
        if self.route:
            vx, wz, _ = self.route[0]
            self.route_left -= dt
            while self.route and self.route_left <= 1e-9:
                self.route.pop(0)
                if self.route:
                    self.route_left += self.route[0][2]
            return vx, wz, 'ROUTE'
        if self.command_time is None:
            return 0.0, 0.0, 'IDLE'
        if now - self.command_time > self.p['command_timeout_s']:
            if self.state.last_event != 'COMMAND_TIMEOUT':
                self.state.timeouts += 1
            return None, None, 'COMMAND_TIMEOUT'
        return (*self.command, 'COMMAND')

    def step(self, now, dt):
        s = self.state
        vx_t, wz_t, source = self._target(now, dt)
        if vx_t is None:
            s.vx = s.wz = 0.0  # stale commands stop the base immediately
            s.last_event = source
            return s
        vx_t = max(-self.p['max_linear_m_s'], min(self.p['max_linear_m_s'], vx_t))
        wz_t = max(-self.p['max_angular_rad_s'], min(self.p['max_angular_rad_s'], wz_t))
        if source == 'ROUTE':
            vx, wz = vx_t, wz_t  # route profiles are already acceleration-limited
        else:
            vx = _approach(s.vx, vx_t, self.p['max_linear_accel_m_s2'] * dt)
            wz = _approach(s.wz, wz_t, self.p['max_angular_accel_rad_s2'] * dt)
        mid = s.yaw + 0.5 * wz * dt
        x, y = s.x + vx * math.cos(mid) * dt, s.y + vx * math.sin(mid) * dt
        if (x, y) != (s.x, s.y) and not self.is_free(x, y):
            s.vx = s.wz = 0.0
            s.collisions += 1
            s.last_event = 'COLLISION_STOP'
            self.route = []
            return s
        s.x, s.y = x, y
        s.yaw = math.atan2(math.sin(s.yaw + wz * dt), math.cos(s.yaw + wz * dt))
        s.vx, s.wz = vx, wz
        s.last_event = source
        return s


def route_segments(start, waypoints, final_yaw):
    """Convert a waypoint route into acceleration-limited (vx, wz, dt) segments.

    Reuses ultron.motion's turn-then-drive trapezoidal profile so scripted mapping runs
    are deterministic. The segment velocities reproduce the profile's poses exactly.
    """
    from ..motion import motion_samples
    segments = []
    previous = tuple(map(float, start))
    for pose, command in motion_samples(start, waypoints, final_yaw):
        dt = command.dt
        dist = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
        dyaw = math.atan2(math.sin(pose[2] - previous[2]), math.cos(pose[2] - previous[2]))
        segments.append((dist / dt, dyaw / dt, dt))
        previous = pose
    return segments
