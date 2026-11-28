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
    description: str        # what the vision model is told this kind looks like
    prompts: tuple          # detector prompt candidates; the first is used unless calibration picks another
    min_score: float        # detector confidence below which a box of this kind is ignored
    size: tuple             # plausible height of the box in metres (min, max): rejects look-alikes
    same_object_radius: float  # metres: sightings this close together are one object
    views: tuple            # View entries the form reader wants
    sides: int              # how many different viewing directions are wanted (45 degree steps)
    checklist: str          # form to fill: "ladder" or a key of checklists.CHECKLISTS
    extent: float = 0.4     # metres: largest horizontal size, how far behind its nearest surface it may reach


OVERVIEW, CLOSE = View("overview", 2.0, 4.5), View("close", 0.8, 2.0)
# Near enough to read a label or an inspection sticker: 1500 pixels per metre is about 0.4 m
# with the B2's head camera (focal length 643 pixels). Only a part of the object fits in view.
DETAIL = View("detail", 0.3, 0.5)

# Prompts and thresholds were tuned on the hall recording with YOLOE. Wording matters: "red fire
# extinguisher" scores 0.9 where "fire extinguisher" finds nothing, and near-synonyms of one kind
# ("ladder" and "stepladder") split the score between them, so the detector runs one prompt per
# kind. `calibrate.py` re-does this choice on a new site from the candidates listed here.
# The red kinds get confused with each other from a distance (a 15 cm alarm box scored 0.6 as an
# extinguisher), so each kind also has a plausible physical height, checked against measured range.
# Measured in the hall: extinguisher 0.43-0.57 m, alarm box 0.35-0.38 m, hose cabinet 0.77-0.88 m.
KINDS = {kind.name: kind for kind in (
    # A free-standing ladder can be walked around; three directions cover rails, rungs and feet.
    Kind("ladder", "LAD", "portable ladder or stepladder",
         ("ladder", "stepladder", "aluminium ladder"), 0.5, (0.5, 8.0), 0.7, (OVERVIEW, CLOSE, DETAIL), 3, "ladder",
         extent=1.8),
    # Wall-mounted equipment: one direction, but close enough to read the inspection sticker.
    Kind("fire_extinguisher", "FEX", "portable red cylinder, about 0.5 m tall, with a hose and handle",
         ("red fire extinguisher", "fire extinguisher", "fire extinguisher standing on the floor"), 0.35, (0.42, 0.9), 0.8, (OVERVIEW, CLOSE, DETAIL), 1, "fire_extinguisher"),
    Kind("fire_hydrant", "HYD", "red wall cabinet, 0.6 to 1.5 m tall, holding a fire hose; closed it looks like a plain red box with a small label",
         ("red fire hose cabinet on the wall", "fire hose cabinet", "fire hydrant"), 0.35, (0.6, 1.5), 0.8, (OVERVIEW, CLOSE, DETAIL), 1, "fire_hydrant"),
    Kind("fire_alarm", "ALM", "small red wall box, 0.1 to 0.4 m tall, with a push button or alarm panel",
         ("red fire alarm box on the wall", "fire alarm call point", "fire alarm"), 0.35, (0.1, 0.41), 0.8, (OVERVIEW, CLOSE), 1, "fire_alarm"),
    Kind("first_aid_kit", "AID", "first-aid box or cabinet, usually green or white with a cross",
         ("first aid kit", "first aid box", "green first aid cabinet on the wall"), 0.5, (0.15, 0.7), 0.8, (OVERVIEW, CLOSE), 1, "first_aid_kit"),
)}

DEFAULT_PROMPTS = {kind.name: {"prompt": kind.prompts[0], "min_score": kind.min_score} for kind in KINDS.values()}
