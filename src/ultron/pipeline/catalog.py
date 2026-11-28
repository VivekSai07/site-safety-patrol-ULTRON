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
    min_score: float        # detector confidence below which a box of this kind is ignored
    size: tuple             # plausible height of the box in metres (min, max): rejects look-alikes
    same_object_radius: float  # metres: sightings this close together are one object
    views: tuple            # View entries the form reader wants
    sides: int              # how many different viewing directions are wanted (45 degree steps)
    checklist: str          # form to fill: "ladder" or a key of checklists.CHECKLISTS


OVERVIEW, CLOSE = View("overview", 2.0, 4.5), View("close", 0.6, 2.0)

# Prompts and thresholds were tuned on the hall recording with YOLOE. Wording matters: "red fire
# extinguisher" scores 0.9 where "fire extinguisher" finds nothing, and near-synonyms of one kind
# ("ladder" and "stepladder") split the score between them, so each kind has one prompt.
# The red kinds get confused with each other from a distance (a 15 cm alarm box scored 0.6 as an
# extinguisher), so each kind also has a plausible physical height, checked against measured range.
# Measured in the hall: extinguisher 0.43-0.57 m, alarm box 0.35-0.38 m, hose cabinet 0.77-0.88 m.
KINDS = {kind.name: kind for kind in (
    # A free-standing ladder can be walked around; three directions cover rails, rungs and feet.
    Kind("ladder", "LAD", ("ladder",), 0.5, (0.5, 8.0), 0.7, (OVERVIEW, CLOSE), 3, "ladder"),
    # Wall-mounted equipment: one direction, but close enough to read the inspection sticker.
    Kind("fire_extinguisher", "FEX", ("red fire extinguisher",), 0.35, (0.42, 0.9), 0.8, (OVERVIEW, CLOSE), 1, "fire_extinguisher"),
    Kind("fire_hydrant", "HYD", ("red fire hose cabinet on the wall",), 0.35, (0.6, 1.5), 0.8, (OVERVIEW, CLOSE), 1, "fire_hydrant"),
    Kind("fire_alarm", "ALM", ("red fire alarm box on the wall",), 0.35, (0.1, 0.41), 0.8, (OVERVIEW, CLOSE), 1, "fire_alarm"),
    Kind("first_aid_kit", "AID", ("first aid kit",), 0.5, (0.15, 0.7), 0.8, (OVERVIEW, CLOSE), 1, "first_aid_kit"),
)}

KIND_OF_PROMPT = {prompt: kind.name for kind in KINDS.values() for prompt in kind.prompts}
