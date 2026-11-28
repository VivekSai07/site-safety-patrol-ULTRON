"""Records passed between pipeline modules. Plain data, JSON-friendly."""
from dataclasses import asdict, dataclass, field
import math

SIDE_STEP = math.pi / 4  # viewing directions are counted in 45 degree steps


@dataclass
class Detection:
    """One object found in one frame. `box` is [x0, y0, x1, y1] in thousandths of the image."""
    kind: str
    box: list
    score: float = 1.0
    track: int = None     # detector's track id: the same number in later frames is the same object
    outline: list = None  # object outline as [[x, y], ...] in thousandths, when the detector segments


@dataclass
class Sighting:
    """A detection placed in the map: where the object is and from where it was seen."""
    detection: Detection
    position: tuple = None   # object (x, y) in the map frame, None if the frame has no pose
    distance: float = None   # metres from camera to object
    approximate: bool = False  # True when no range sensor hit the object and the range was assumed
    height: float = None     # physical height of the box in metres at the measured range
    cells: list = None       # [[i, j], ...] floor cells (CELL metres) the object's visible surface covers
    zband: tuple = None      # (low, high) metres above (+) or below (-) the camera that the box spans

    def side(self, robot_pose):
        """Viewing direction as a 45 degree step, 0..7: the map bearing from object to camera."""
        if self.position is None or robot_pose is None:
            return None
        bearing = math.atan2(robot_pose[1] - self.position[1], robot_pose[0] - self.position[0])
        return int(round(bearing / SIDE_STEP)) % 8


@dataclass
class SiteObject:
    id: str
    kind: str
    position: list = None    # running mean of sighting positions, map frame
    sightings: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    status: str = "FOUND"    # FOUND -> INSPECTED (or UNREACHABLE); REJECTED if the vision model says it is nothing;
                             # MOVED once its place was seen empty
    verified: bool = False   # the vision model looked at a keyframe and agreed with the kind
    result: dict = field(default_factory=dict)  # what the inspection concluded
    apart: list = field(default_factory=list)   # ids seen in the same frame: certainly other objects
    cells: dict = field(default_factory=dict)   # {"i,j": sightings covering that floor cell}: the footprint
    zband: list = None       # mean (low, high) height band relative to the camera: where to look for it
    views: int = 0           # frames that had its place in clear view (near, in the image, not hidden) ...
    detected: int = 0        # ... and of those, the frames the detector found it in

    def to_json(self):
        return asdict(self)


@dataclass
class Keyframe:
    """A stored frame of one object, with the geometry needed to choose between frames."""
    id: str
    object_id: str
    file: str                # JPEG, relative to the workspace
    stamp: float
    box: list
    pose: list = None        # robot (x, y, yaw) when taken
    distance: float = None
    side: int = None
    sharpness: float = 0.0
    score: float = 1.0       # detector confidence

    def to_json(self):
        return asdict(self)


@dataclass
class ViewRequest:
    """Where the robot should stand to get a view an object still lacks."""
    object_id: str
    view: str                # catalog View name
    side: int
    x: float
    y: float
    yaw: float
    reason: str = ""
