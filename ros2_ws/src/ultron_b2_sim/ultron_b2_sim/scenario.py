"""Simulation scenario hooks for ladders (move / hide / show) and ladder ground truth.

SIMULATION ONLY. Ladder poses returned by :meth:`LadderHooks.truth` are simulator ground truth
for evaluation and for the clearly labelled ``oracle_sim`` detector; never estimator input.
"""
import math

PARKED_XY = (-30.0, -30.0)
PARKED_Z = -6.0
TRUTH_SCHEMA = 'ultron.sim_ladder_truth/v1'
GROUND_TRUTH_FRAME = 'ultron_sim_world'


def _quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def _to_local(corners, position, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for x, y in corners:
        dx, dy = x - position[0], y - position[1]
        out.append((c * dx + s * dy, -s * dx + c * dy))
    return out


def _to_world(local, position, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(position[0] + c * x - s * y, position[1] + s * x + c * y) for x, y in local]


class LadderHooks:
    def __init__(self, sim):
        self.sim = sim
        mj, model = sim.mujoco, sim.model
        self.ladders = {}
        for obj in sim.objects:
            if not obj.name.startswith('ladder_'):
                continue
            bid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, obj.name)
            if bid < 0:
                continue
            self.ladders[obj.name] = {
                'object': obj, 'body': bid, 'hidden': False, 'kind': obj.kind,
                'local_footprint': _to_local(obj.footprint, obj.position, obj.yaw),
                'last_pose': (float(obj.position[0]), float(obj.position[1]), float(obj.yaw)),
            }
        self.version = 0

    def _get(self, name):
        if name not in self.ladders:
            raise ValueError(f'unknown ladder {name!r}; known: {sorted(self.ladders)}')
        return self.ladders[name]

    def _apply(self, entry, x, y, yaw, hidden):
        model = self.sim.model
        bid = entry['body']
        obj = entry['object']
        if hidden:
            model.body_pos[bid] = (*PARKED_XY, PARKED_Z)
            obj.footprint = []
        else:
            model.body_pos[bid] = (x, y, 0.0)
            model.body_quat[bid] = _quat(yaw)
            obj.position, obj.yaw = (x, y), yaw
            obj.footprint = _to_world(entry['local_footprint'], (x, y), yaw)
            entry['last_pose'] = (x, y, yaw)
        entry['hidden'] = hidden
        self.sim.mujoco.mj_forward(model, self.sim.data)
        self.version += 1

    def move(self, name, x, y, yaw):
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('nonfinite ladder pose')
        entry = self._get(name)
        self._apply(entry, float(x), float(y), float(yaw), False)

    def hide(self, name):
        self._apply(self._get(name), 0.0, 0.0, 0.0, True)

    def show(self, name):
        entry = self._get(name)
        self._apply(entry, *entry['last_pose'], False)

    def truth(self, labels=None):
        out = []
        for name in sorted(self.ladders):
            e = self.ladders[name]
            x, y, yaw = e['last_pose']
            out.append({'name': name, 'label': (labels or {}).get(name), 'kind': e['kind'],
                        'present': not e['hidden'], 'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5),
                        'footprint': [[round(a, 4), round(b, 4)] for a, b in e['object'].footprint]})
        return out
