"""ladder_tracker node: map-scoped association of localized ladder observations.

Subscribes ``perception/ladder_observations`` and ``state/localization`` (map id/version).
Publishes ``perception/ladder_candidates`` (LadderCandidateArray, latched) on change and at 1 Hz.
Ambiguity is preserved (AMBIGUOUS candidates are never merged). Optional persistence of ladder
identity through ``src/ultron/memory.py`` when ``memory_db`` is set; priors for the current map
are loaded so a stationary ladder keeps its asset ID across patrols.
"""
import sys

import numpy as np

from . import registry_store
from .ros_util import run_node, stamp_to_float
from .tracker_core import LadderTracker


def main(argv=None):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from builtin_interfaces.msg import Time as TimeMsg
    from ultron_b2_interfaces.msg import LadderCandidate, LadderCandidateArray, LocalizationState

    def to_time(seconds):
        sec = int(seconds)
        return TimeMsg(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1_000_000_000)

    class LadderTrackerNode(Node):
        def __init__(self):
            super().__init__('ladder_tracker')
            p = self.declare_parameter
            self.tracker = LadderTracker(radius=float(p('association_radius_m', 0.65).value),          # PROPOSED
                                         ambiguity_ratio=float(p('ambiguity_ratio', 0.5).value),       # PROPOSED
                                         missing_after=int(p('missing_after_views', 3).value))         # PROPOSED
            self.map_frame = p('map_frame', 'map').value
            db = p('memory_db', '').value
            self.registry = None
            if db:
                try:
                    self.registry = registry_store.open_registry(db)
                except Exception as exc:
                    self.get_logger().error(f'memory registry unavailable ({exc}); tracking without persistence')
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(LadderCandidateArray, 'perception/ladder_candidates', latched)
            self.create_subscription(LocalizationState, 'state/localization', self._on_loc, 10)
            self.create_subscription(LadderCandidateArray, 'perception/ladder_observations', self._on_obs, 10)
            self.create_timer(1.0, self._publish)

        def _on_loc(self, msg):
            if msg.map_id and self.tracker.set_map(msg.map_id, msg.map_version):
                self.get_logger().info(f'map scope {msg.map_id}@{msg.map_version}; candidates reset')
                if self.registry is not None:
                    key = registry_store.map_key(msg.map_id, msg.map_version)
                    self.tracker.seed(registry_store.load_priors(self.registry, key))
                self._publish()

        def _on_obs(self, msg):
            if self.tracker.map_key is None:
                return
            obs, checks = [], []
            for c in msg.candidates:
                if (c.map_id, int(c.map_version)) != self.tracker.map_key:
                    continue  # observation from another map: never associated across maps
                if c.association_state == 'VIEW_CHECK':
                    checks.append(c.candidate_id)
                elif c.association_state == 'OBSERVATION':
                    obs.append({'position': (c.position.x, c.position.y, c.position.z),
                                'covariance': np.asarray(c.position_covariance).reshape(3, 3),
                                'ladder_type': c.ladder_type, 'from_oracle': c.from_oracle,
                                'detection_id': c.detection_ids[0] if c.detection_ids else ''})
            changed = self.tracker.update(obs, stamp_to_float(msg.header.stamp), checks)
            if changed:
                self._persist(changed)
                self._publish()

        def _persist(self, changed):
            if self.registry is None:
                return
            key = registry_store.map_key(*self.tracker.map_key)
            for cid in changed:
                cand = self.tracker.candidates[cid]
                try:
                    if not cand.asset_id:
                        cand.asset_id = registry_store.ensure_asset(self.registry, key, cand.as_dict())
                    else:
                        registry_store.update_asset(self.registry, cand.as_dict())
                except Exception as exc:
                    self.get_logger().error(f'registry write failed for {cid}: {exc}')

        def _publish(self):
            out = LadderCandidateArray(schema_version=LadderCandidateArray.SCHEMA_VERSION)
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.map_frame
            if self.tracker.map_key is not None:
                for c in self.tracker.snapshot():
                    m = LadderCandidate(schema_version=LadderCandidate.SCHEMA_VERSION, header=out.header,
                                        candidate_id=c['candidate_id'], map_id=self.tracker.map_key[0],
                                        map_version=self.tracker.map_key[1], yaw=float(c['yaw']),
                                        ladder_type=c['ladder_type'], association_state=c['state'],
                                        asset_id=c['asset_id'], ambiguous_with=c['ambiguous_with'],
                                        observation_count=c['observation_count'],
                                        first_seen=to_time(c['first_seen']), last_seen=to_time(c['last_seen']),
                                        detection_ids=c['detection_ids'], from_oracle=c['from_oracle'])
                    m.position.x, m.position.y, m.position.z = c['position']
                    m.position_covariance = [float(v) for v in np.asarray(c['covariance']).reshape(9)]
                    out.candidates.append(m)
            self.pub.publish(out)

    return run_node(LadderTrackerNode, argv)


if __name__ == '__main__':
    sys.exit(main())
