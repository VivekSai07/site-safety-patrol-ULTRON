"""Map-scoped tracker: stable IDs, NEW, AMBIGUOUS never merged, MISSING only from unoccluded views."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ultron_b2_perception.tracker_core import AMBIGUOUS, MATCHED, MISSING, NEW, LadderTracker  # noqa: E402


def obs(x, y, t='TRESTLE', det='', oracle=True):
    return {'position': [x, y, 0.4], 'ladder_type': t, 'detection_id': det, 'from_oracle': oracle}


def test_stationary_ladder_keeps_id_and_new_is_new():
    tr = LadderTracker()
    tr.set_map('hall', 1)
    changed = tr.update([obs(4.0, 4.0, det='d1')], 1.0)
    (cid,) = changed
    assert tr.candidates[cid].state == NEW
    rng = np.random.default_rng(0)
    for k in range(20):
        n = rng.normal(0, 0.08, 2)
        tr.update([obs(4.0 + n[0], 4.0 + n[1], det=f'd{k}')], 2.0 + k)
    assert list(tr.candidates) == [cid]
    c = tr.candidates[cid]
    assert c.state == NEW and c.observation_count == 21
    assert np.hypot(c.position[0] - 4.0, c.position[1] - 4.0) < 0.1
    assert c.from_oracle


def test_prior_from_memory_is_matched_with_same_id():
    tr = LadderTracker()
    tr.set_map('hall', 1)
    tr.seed([{'asset_id': 'LAD-001', 'position': [4.0, 4.0, 0.0], 'ladder_type': 'TRESTLE'}])
    tr.update([obs(4.2, 3.9)], 1.0)
    assert list(tr.candidates) == ['LAD-001'] and tr.candidates['LAD-001'].state == MATCHED
    # A different map scope never reuses the candidate.
    assert tr.set_map('hall', 2) and not tr.candidates


def test_two_ladders_in_one_frame_near_one_candidate_become_ambiguous_not_merged():
    tr = LadderTracker()
    tr.set_map('hall', 1)
    (a,) = tr.update([obs(4.0, 4.0)], 1.0)
    changed = tr.update([obs(4.1, 4.0), obs(4.5, 4.2)], 2.0)
    assert len(tr.candidates) == 2
    b = (changed - {a}).pop()
    assert tr.candidates[a].state == AMBIGUOUS and tr.candidates[b].state == AMBIGUOUS
    assert tr.candidates[a].ambiguous_with == {b} and tr.candidates[b].ambiguous_with == {a}
    # Later observations keep both identities; ambiguity is sticky.
    tr.update([obs(4.05, 4.0)], 3.0)
    assert len(tr.candidates) == 2 and tr.candidates[a].state == AMBIGUOUS


def test_equidistant_observation_is_not_fused_into_either():
    tr = LadderTracker()
    tr.set_map('hall', 1)
    tr.update([obs(4.0, 4.0), obs(4.8, 4.0)], 1.0)
    before = {k: c.position.copy() for k, c in tr.candidates.items()}
    tr.update([obs(4.4, 4.0)], 2.0)
    assert len(tr.candidates) == 2
    for k, c in tr.candidates.items():
        assert np.allclose(c.position, before[k]) and c.ambiguous_observations == 1


def test_incompatible_type_creates_new_candidate():
    tr = LadderTracker()
    tr.set_map('hall', 1)
    tr.update([obs(4.0, 4.0, 'TRESTLE')], 1.0)
    tr.update([obs(4.1, 4.0, 'LEANING_COMBINATION_LADDER')], 2.0)
    assert len(tr.candidates) == 2


def test_missing_after_repeated_unoccluded_views_and_recovery():
    tr = LadderTracker(missing_after=3)
    tr.set_map('hall', 1)
    (a,) = tr.update([obs(4.0, 4.0)], 1.0)
    tr.update([], 2.0, view_checks=[a])
    tr.update([], 3.0, view_checks=[a])
    assert tr.candidates[a].state == NEW
    tr.update([], 4.0, view_checks=[a])
    assert tr.candidates[a].state == MISSING
    # Not in view (no view check) => no evidence; re-observed => MATCHED again.
    tr.update([obs(4.0, 4.0)], 5.0)
    assert tr.candidates[a].state == MATCHED and tr.candidates[a].misses == 0


def test_absence_without_view_check_is_not_missing():
    tr = LadderTracker(missing_after=1)
    tr.set_map('hall', 1)
    (a,) = tr.update([obs(4.0, 4.0)], 1.0)
    for k in range(10):
        tr.update([], 2.0 + k)
    assert tr.candidates[a].state == NEW
