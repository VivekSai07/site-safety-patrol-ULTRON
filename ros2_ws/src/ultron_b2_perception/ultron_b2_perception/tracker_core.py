"""Map-scoped ladder candidate tracker (port of the association rules in src/ultron/memory.py and
src/ultron/observations.py).

Rules (radii PROPOSED):
* An observation with exactly one compatible candidate within ``radius`` that is not already
  claimed in the same frame is MATCHED (position fused, ID kept).
* Two observations in one frame that would claim the same candidate cannot be one ladder: the
  extra observation becomes a new AMBIGUOUS candidate and both are marked AMBIGUOUS. Never merged.
* An observation with several candidates in range is matched only if the nearest is clearly
  nearer (``ambiguity_ratio``); otherwise it is recorded on all of them as ambiguous evidence and
  no position is fused.
* No candidate in range => NEW candidate.
* A candidate repeatedly in unoccluded view (``view_checks``) but not observed is MISSING after
  ``missing_after`` frames. Absence elsewhere is not evidence of removal.

Candidates are scoped to (map_id, map_version); changing the map clears the session state.
Ordered history (first/last seen, detection ids) is preserved.
"""
from dataclasses import dataclass, field
import itertools
import math

import numpy as np

NEW, MATCHED, AMBIGUOUS, MISSING = 'NEW', 'MATCHED', 'AMBIGUOUS', 'MISSING'
UNKNOWN_TYPE = 'UNKNOWN'


def types_compatible(a, b):
    return a == b or UNKNOWN_TYPE in (a, b) or not a or not b


@dataclass
class Candidate:
    candidate_id: str
    position: np.ndarray
    covariance: np.ndarray
    ladder_type: str
    state: str
    first_seen: float
    last_seen: float
    asset_id: str = ''
    yaw: float = 0.0
    ambiguous_with: set = field(default_factory=set)
    observation_count: int = 0
    ambiguous_observations: int = 0
    detection_ids: list = field(default_factory=list)
    from_oracle: bool = False
    misses: int = 0
    prior: bool = False
    seen_this_session: bool = False

    def as_dict(self):
        return {'candidate_id': self.candidate_id, 'asset_id': self.asset_id,
                'position': [float(v) for v in self.position], 'covariance': self.covariance.tolist(),
                'ladder_type': self.ladder_type, 'state': self.state, 'yaw': self.yaw,
                'ambiguous_with': sorted(self.ambiguous_with), 'observation_count': self.observation_count,
                'first_seen': self.first_seen, 'last_seen': self.last_seen,
                'detection_ids': list(self.detection_ids), 'from_oracle': self.from_oracle,
                'prior': self.prior, 'seen_this_session': self.seen_this_session}


class LadderTracker:
    def __init__(self, *, radius=0.65, ambiguity_ratio=0.5, missing_after=3, max_detection_ids=20,
                 id_prefix='CAND'):
        self.radius = float(radius)
        self.ambiguity_ratio = float(ambiguity_ratio)
        self.missing_after = int(missing_after)
        self.max_detection_ids = int(max_detection_ids)
        self.id_prefix = id_prefix
        self.map_key = None
        self.candidates = {}
        self._ids = itertools.count(1)

    # ----------------------------------------------------------------- map scope
    def set_map(self, map_id, map_version):
        key = (str(map_id), int(map_version))
        if key == self.map_key:
            return False
        self.map_key = key
        self.candidates = {}
        self._ids = itertools.count(1)
        return True

    def seed(self, priors):
        """Load prior candidates for the current map (e.g. registry assets from earlier patrols)."""
        for p in priors:
            cid = p.get('candidate_id') or p['asset_id']
            self.candidates[cid] = Candidate(
                candidate_id=cid, asset_id=p.get('asset_id', ''), position=np.asarray(p['position'], float),
                covariance=np.asarray(p.get('covariance', np.eye(3) * 0.04), float).reshape(3, 3),
                ladder_type=p.get('ladder_type', UNKNOWN_TYPE), state=p.get('state', MATCHED),
                first_seen=float(p.get('first_seen', 0.0)), last_seen=float(p.get('last_seen', 0.0)),
                ambiguous_with=set(p.get('ambiguous_with', ())), from_oracle=bool(p.get('from_oracle', False)),
                observation_count=int(p.get('observation_count', 0)), prior=True)

    def _new_id(self):
        while True:
            cid = f'{self.id_prefix}-{next(self._ids):04d}'
            if cid not in self.candidates:
                return cid

    # ----------------------------------------------------------------- update
    def update(self, observations, stamp, view_checks=()):
        """Fuse one frame of observations. Returns the set of changed candidate ids."""
        if self.map_key is None:
            raise RuntimeError('tracker has no map scope; call set_map first')
        changed, claimed, uncertain = set(), set(), set()
        for obs in sorted(observations, key=lambda o: (o['position'][0], o['position'][1])):
            pos = np.asarray(obs['position'], float)
            if not np.all(np.isfinite(pos)):
                continue
            near = sorted(((math.hypot(c.position[0] - pos[0], c.position[1] - pos[1]), c)
                           for c in self.candidates.values()
                           if types_compatible(c.ladder_type, obs.get('ladder_type', UNKNOWN_TYPE))),
                          key=lambda dc: dc[0])
            near = [(d, c) for d, c in near if d <= self.radius]
            target = None
            if len(near) == 1:
                target = near[0][1]
            elif len(near) >= 2 and near[0][0] <= self.ambiguity_ratio * near[1][0]:
                target = near[0][1]
            elif len(near) >= 2:
                for _, c in near:
                    c.ambiguous_observations += 1
                    changed.add(c.candidate_id)
                    uncertain.add(c.candidate_id)
                continue
            if target is not None and target.candidate_id not in claimed:
                self._fuse(target, obs, pos, stamp)
                claimed.add(target.candidate_id)
                changed.add(target.candidate_id)
                continue
            cand = self._create(obs, pos, stamp, NEW if target is None else AMBIGUOUS)
            if target is not None:  # same-frame split: two ladders where one was assumed
                cand.ambiguous_with.add(target.candidate_id)
                target.ambiguous_with.add(cand.candidate_id)
                target.state = AMBIGUOUS
                changed.add(target.candidate_id)
            claimed.add(cand.candidate_id)
            changed.add(cand.candidate_id)
        for cid in view_checks:
            cand = self.candidates.get(cid)
            if cand is None or cid in claimed or cid in uncertain:
                continue
            cand.misses += 1
            if cand.misses >= self.missing_after and cand.state != MISSING:
                cand.state = MISSING
                changed.add(cid)
        return changed

    def _create(self, obs, pos, stamp, state):
        cid = self._new_id()
        cand = Candidate(candidate_id=cid, position=pos.copy(),
                         covariance=np.asarray(obs.get('covariance', np.eye(3) * 0.04), float).reshape(3, 3),
                         ladder_type=obs.get('ladder_type', UNKNOWN_TYPE), state=state, first_seen=stamp,
                         last_seen=stamp, yaw=float(obs.get('yaw', 0.0)), observation_count=1,
                         detection_ids=[obs['detection_id']] if obs.get('detection_id') else [],
                         from_oracle=bool(obs.get('from_oracle', False)), seen_this_session=True)
        self.candidates[cid] = cand
        return cand

    def _fuse(self, cand, obs, pos, stamp):
        C_obs = np.asarray(obs.get('covariance', np.eye(3) * 0.04), float).reshape(3, 3)
        try:
            gain = cand.covariance @ np.linalg.inv(cand.covariance + C_obs)
            cand.position = cand.position + gain @ (pos - cand.position)
            cand.covariance = (np.eye(3) - gain) @ cand.covariance
        except np.linalg.LinAlgError:
            n = cand.observation_count
            cand.position = (cand.position * n + pos) / (n + 1)
        # Keep a floor so a long-tracked candidate can still follow small map corrections.
        cand.covariance = cand.covariance + np.eye(3) * 1e-4
        cand.observation_count += 1
        cand.last_seen = stamp
        cand.misses = 0
        cand.seen_this_session = True
        cand.from_oracle = cand.from_oracle or bool(obs.get('from_oracle', False))
        if obs.get('detection_id'):
            cand.detection_ids = (cand.detection_ids + [obs['detection_id']])[-self.max_detection_ids:]
        # NEW (not in prior memory for this map) stays NEW for the session; ambiguity is sticky.
        if cand.ambiguous_with:
            cand.state = AMBIGUOUS
        elif cand.prior or cand.state == MISSING:
            cand.state = MATCHED

    def snapshot(self):
        return [c.as_dict() for c in sorted(self.candidates.values(), key=lambda c: c.candidate_id)]
