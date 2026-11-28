"""Pure logic for b2_sport_bridge (no ROS): preflight, clamps, watchdog/arming state machine.

Unitree B2 sport API ids from the pinned unitree_ros2 source (deps.repos @668d1ec5,
example/src/include/common/ros2_b2_sport_client.h): STOPMOVE = 1003, MOVE = 1008; Move parameter
is JSON {"x": vx, "y": vy, "z": vyaw}. R-67: the vendor API may drift between firmware versions;
re-verify against the robot's firmware before any authorized hardware test.
"""
from dataclasses import dataclass, field
import json
import math

API_STOPMOVE = 1003
API_MOVE = 1008
ZERO_EPS = 1e-4


def preflight(*, target, hardware_enable, env, namespace, expected_domain_id=-1, hw_token_ok=None):
    """Return (ok, reason). Runs before any request publisher exists."""
    domain = env.get('ROS_DOMAIN_ID')
    if domain is None or not str(domain).strip().isdigit():
        return False, f'ROS_DOMAIN_ID unset or invalid ({domain!r})'
    domain = int(domain)
    if expected_domain_id is not None and int(expected_domain_id) >= 0 and domain != int(expected_domain_id):
        return False, f'ROS_DOMAIN_ID={domain} != expected_domain_id={expected_domain_id}'
    if target == 'mock':
        if domain == 0:
            return False, 'target mock refuses ROS_DOMAIN_ID=0 (robot network domain)'
        if env.get('ROS_LOCALHOST_ONLY') != '1':
            return False, 'target mock requires ROS_LOCALHOST_ONLY=1'
        if namespace.rstrip('/') == '/ultron/live':
            return False, 'target mock refuses the live namespace'
        return True, f'mock target, domain {domain}, localhost only'
    if target == 'robot':
        if not hardware_enable:
            return False, 'target robot requires hardware_enable:=true'
        if domain != 0:
            return False, f'target robot requires the robot domain 0 (got {domain})'
        if namespace.rstrip('/') != '/ultron/live':
            return False, f'target robot requires namespace /ultron/live (got {namespace})'
        if hw_token_ok is None or not hw_token_ok[0]:
            return False, f'hardware token: {hw_token_ok[1] if hw_token_ok else "not checked"}'
        return True, f'robot target: {hw_token_ok[1]}'
    return False, f'unknown target {target!r} (mock | robot)'


@dataclass
class Limits:
    max_vx: float = 0.6
    min_vx: float = -0.3
    max_vy: float = 0.3
    max_wz: float = 0.8
    max_ax: float = 1.0
    max_ay: float = 1.0
    max_awz: float = 2.0


def clamp_velocity(cmd, lim):
    vx, vy, wz = cmd
    return (min(max(vx, lim.min_vx), lim.max_vx), min(max(vy, -lim.max_vy), lim.max_vy),
            min(max(wz, -lim.max_wz), lim.max_wz))


def limit_accel(target, previous, lim, dt):
    """Limit increases in |v| per axis; decreases (braking, sign change toward zero) pass immediately."""
    out = []
    for t, p, a in zip(target, previous, (lim.max_ax, lim.max_ay, lim.max_awz)):
        if abs(t) <= abs(p) and t * p >= 0:
            out.append(t)
            continue
        base = p if t * p > 0 else 0.0           # crossing zero: brake to 0 first, then accelerate
        step = a * dt
        out.append(base + max(-step, min(step, t - base)))
    return tuple(out)


def move_parameter(vx, vy, wz):
    return json.dumps({'x': float(vx), 'y': float(vy), 'z': float(wz)})


@dataclass
class BridgeCore:
    """Arming + watchdog state machine. tick() returns ('MOVE', (vx, vy, wz)) | ('STOP', reason) | None."""
    limits: Limits = field(default_factory=Limits)
    watchdog_s: float = 0.3
    rate_hz: float = 20.0
    armed: bool = False
    fault: str = ''
    cmd: tuple | None = None
    cmd_stamp: float | None = None
    sent: tuple = (0.0, 0.0, 0.0)
    stopped: bool = True                 # last request was StopMove (or nothing sent yet)
    pending_stop: str = ''

    def arm(self, now):
        if self.cmd is not None and self.cmd_stamp is not None and now - self.cmd_stamp <= self.watchdog_s \
                and any(abs(v) > ZERO_EPS for v in self.cmd):
            return False, 'refused: a non-zero cmd_vel is streaming; arm only at rest'
        self.armed, self.fault = True, ''
        self.sent, self.stopped = (0.0, 0.0, 0.0), True
        return True, 'armed'

    def disarm(self, reason):
        was = self.armed
        self.armed = False
        if was and not self.stopped:
            self.pending_stop = reason
        return was

    def latch_fault(self, reason):
        self.fault = reason
        if self.armed or not self.stopped:
            self.pending_stop = f'FAULT: {reason}'
        self.armed = False

    def on_cmd(self, vx, vy, wz, now):
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self.latch_fault('NONFINITE_CMD_VEL')
            self.cmd = None
            return
        self.cmd, self.cmd_stamp = (float(vx), float(vy), float(wz)), now

    def tick(self, now):
        if self.pending_stop:
            why, self.pending_stop = self.pending_stop, ''
            return self._stop(why)
        if not self.armed:
            return None
        fresh = self.cmd is not None and self.cmd_stamp is not None and now - self.cmd_stamp <= self.watchdog_s
        if not fresh:
            return None if self.stopped else self._stop('WATCHDOG: no fresh cmd_vel')
        target = clamp_velocity(self.cmd, self.limits)
        if all(abs(v) <= ZERO_EPS for v in target):
            return None if self.stopped else self._stop('ZERO_COMMAND')
        out = limit_accel(target, self.sent, self.limits, 1.0 / self.rate_hz)
        self.sent, self.stopped = out, False
        return 'MOVE', out

    def check_watchdog(self, now):
        """Fast path (called at a higher rate than tick): StopMove as soon as the command goes stale."""
        if self.pending_stop:
            why, self.pending_stop = self.pending_stop, ''
            return self._stop(why)
        if self.stopped:
            return None
        if self.cmd_stamp is None or now - self.cmd_stamp > self.watchdog_s:
            return self._stop('WATCHDOG: no fresh cmd_vel')
        return None

    def _stop(self, why):
        self.sent, self.stopped = (0.0, 0.0, 0.0), True
        return 'STOP', why
