"""What the patrol looks for, and what each kind of object needs before its form can be read.

One entry per kind. Adding a kind means adding an entry here and, for a new form,
a checklist in `checklists.py`: no other module names kinds.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class View:
    """A camera view an inspection needs: seen from `min_range`..`max_range` metres."""
    name: str
    min_range: float
    max_range: float


@dataclass(frozen=True)
class Kind:
    name: str
    prefix: str             # object ids: LAD-001, FEX-001, ...
    prompts: tuple          # what the open-vocabulary detector is asked for
    same_object_radius: float  # metres: sightings this close together are one object
    views: tuple            # View entries the form reader wants
    sides: int              # how many different viewing directions are wanted (45 degree steps)
    checklist: str          # form to fill: "ladder" or a key of checklists.CHECKLISTS


OVERVIEW, CLOSE = View("overview", 2.0, 4.5), View("close", 0.6, 2.0)

KINDS = {kind.name: kind for kind in (
    # A free-standing ladder can be walked around; three directions cover rails, rungs and feet.
    Kind("ladder", "LAD", ("ladder", "stepladder"), 1.0, (OVERVIEW, CLOSE), 3, "ladder"),
    # Wall-mounted equipment: one direction, but close enough to read the inspection sticker.
    Kind("fire_extinguisher", "FEX", ("fire extinguisher",), 0.8, (OVERVIEW, CLOSE), 1, "fire_extinguisher"),
    Kind("fire_hydrant", "HYD", ("fire hydrant", "fire hose cabinet"), 0.8, (OVERVIEW, CLOSE), 1, "fire_hydrant"),
    Kind("first_aid_kit", "AID", ("first aid kit", "first aid box"), 0.8, (OVERVIEW, CLOSE), 1, "first_aid_kit"),
)}

KIND_OF_PROMPT = {prompt: kind.name for kind in KINDS.values() for prompt in kind.prompts}
