"""The import surface for the association pipeline: appearance advice at its decision points.

`objects.py` on the association branch calls `advise()` where it asks "is this
sighting a known object?" and "are these two the same?", interprets the scores
with `classify()`, and otherwise keeps its geometric behaviour. `was_inspected`
lets the inspection stage skip objects a mission already inspected.
"""
from typing import Protocol, runtime_checkable


@runtime_checkable
class AssociationAdvisor(Protocol):
    def advise(self, crop, position, kind, candidate_ids=None):
        """Appearance scores (object_id, cosine) for the caller's candidates, best
        first. An empty list means appearance has no say."""
        ...

    def register_sighting(self, object_id, crop, position, kind):
        """Fold one sighting of a (possibly new) object into the memory."""
        ...

    def mark_inspected(self, object_id, mission_id): ...

    def was_inspected(self, object_id):
        """The mission that inspected this object, or None."""
        ...
