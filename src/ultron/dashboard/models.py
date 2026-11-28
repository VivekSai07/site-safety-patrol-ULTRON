"""Data contract between the robot side (sim or ROS) and the dashboard."""
import math
import re
from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .checklist import ALL_KEYS, ASCENT_TYPES, MATERIALS, ItemResult

SAFE_ID = re.compile(r'^[A-Za-z0-9_.-]{1,64}$')


class Verdict(str, Enum):
    FLAGGED = 'FLAGGED'
    MANUAL_REVIEW = 'MANUAL_REVIEW'
    AUTOMATED_PASS = 'AUTOMATED_PASS'


class CheckResult(str, Enum):
    PASS = 'PASS'
    FAIL = 'FAIL'
    UNKNOWN = 'UNKNOWN'


def safe_id(value):
    if not SAFE_ID.match(value):
        raise ValueError('ids may use letters, digits, ".", "_" and "-" only')
    return value


class SiteMap(BaseModel):
    bounds: list[float] = Field(min_length=4, max_length=4, description='x0, y0, x1, y1 in metres')
    keepouts: list[list[float]] = []
    base: list[float] | None = None


class MissionStart(BaseModel):
    mission_id: str
    site_id: str
    map_version: str = ''
    synthetic: bool = True
    map: SiteMap | None = None

    @field_validator('mission_id', 'site_id')
    @classmethod
    def ids_are_safe(cls, value):
        return safe_id(value)


class MissionEnd(BaseModel):
    status: str = Field(min_length=1)
    summary: dict[str, Any] = {}


class Event(BaseModel):
    mission_id: str
    type: str = Field(min_length=1)
    time: float = Field(ge=0, description='Mission time in seconds')
    asset_id: str | None = None
    data: dict[str, Any] = {}


class Pose(BaseModel):
    time: float = Field(ge=0)
    x: float
    y: float
    yaw: float = 0.0
    state: str = ''


class Goal(BaseModel):
    """A navigation goal set on the dashboard map, like RViz's 2D Goal Pose (map frame, yaw in radians)."""
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = 'map'
    # who asked: the operator always wins; the pipeline only drives when nobody else does
    # (docs/EXPLORATION_CONTRACT.md)
    by: Literal['operator', 'pipeline'] = 'operator'


class GoalStatus(str, Enum):
    SENT = 'SENT'            # dashboard has it, no bridge has picked it up yet
    ACCEPTED = 'ACCEPTED'    # the ROS bridge published it to the navigation stack
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


class GoalUpdate(BaseModel):
    seq: int
    status: GoalStatus
    detail: str = ''


class ExplorationState(str, Enum):
    """Mapping the site with the team's frontier explorer (m-explore-ros2), started from the dashboard.

    The operator or pipeline asks (STARTING, PAUSING, RESUMING, FINISHING); only the ROS bridge confirms
    (EXPLORING, PAUSED, MAPPED, FAILED), so the page never shows the robot exploring before the robot
    side says it is.
    """
    IDLE = 'IDLE'            # not explored since the dashboard started
    STARTING = 'STARTING'    # operator pressed Start; no bridge has confirmed yet
    EXPLORING = 'EXPLORING'  # bridge: the explorer is running and drives the robot
    PAUSING = 'PAUSING'      # pause asked (explore/resume=false); no bridge has confirmed yet
    PAUSED = 'PAUSED'        # bridge: the explorer is stopped but alive; goals may drive Nav2
    RESUMING = 'RESUMING'    # resume asked (explore/resume=true); no bridge has confirmed yet
    FINISHING = 'FINISHING'  # operator pressed Finish mapping; no bridge has confirmed yet
    MAPPED = 'MAPPED'        # explorer stopped (operator finished, or no frontiers left); the map is usable
    FAILED = 'FAILED'        # bridge could not start or stop the explorer; detail says why


# The explorer process exists (or is being started): Start is refused.
EXPLORATION_ACTIVE = {ExplorationState.STARTING, ExplorationState.EXPLORING, ExplorationState.PAUSING,
                      ExplorationState.PAUSED, ExplorationState.RESUMING, ExplorationState.FINISHING}
# The explorer drives Nav2 (or is about to): every goal is refused with 409 'exploring'.
EXPLORATION_DRIVING = {ExplorationState.STARTING, ExplorationState.EXPLORING, ExplorationState.RESUMING,
                       ExplorationState.FINISHING}
EXPLORATION_PAUSE = {ExplorationState.PAUSING, ExplorationState.PAUSED}
BRIDGE_REPORTS = {ExplorationState.EXPLORING, ExplorationState.PAUSED, ExplorationState.MAPPED, ExplorationState.FAILED}
MAX_LEASE_S = 180.0


class ExplorationAction(str, Enum):
    START = 'start'
    PAUSE = 'pause'
    RENEW = 'renew'
    RESUME = 'resume'
    FINISH = 'finish'


class ExplorationRequest(BaseModel):
    action: ExplorationAction
    by: Literal['operator', 'pipeline'] = 'operator'
    # a pipeline pause ends by itself after this long unless renewed (the pipeline may crash or hang);
    # longer asks are cut to MAX_LEASE_S. An operator pause has no lease.
    lease_s: float = Field(60.0, gt=0, allow_inf_nan=False)
    reason: str = Field('', max_length=200)

    @field_validator('lease_s')
    @classmethod
    def capped(cls, value):
        return min(value, MAX_LEASE_S)


class ExplorationUpdate(BaseModel):
    """From the bridge, about the latest request (`seq`)."""
    seq: int
    state: ExplorationState
    detail: str = ''
    by: str = ''             # who ended it: 'operator' or 'explorer' (no frontiers left)

    @field_validator('state')
    @classmethod
    def reported_by_bridge(cls, state):
        if state not in BRIDGE_REPORTS:
            raise ValueError(f'the bridge reports {sorted(s.value for s in BRIDGE_REPORTS)}; {state.value} is the operator\'s')
        return state


def _finite_points(points):
    for p in points:
        if not all(math.isfinite(v) for v in p):
            raise ValueError('coordinates must be finite numbers')
    return points


class Fence(BaseModel):
    """Where the robot may go and objects may be: a polygon in the map frame, the room's walls drawn
    slightly inside the glass (lidar sees through windows, so SLAM maps free space beyond them)."""
    frame_id: str = 'map'
    polygon: list[list[float]] = Field(min_length=3, max_length=1000)

    @field_validator('frame_id')
    @classmethod
    def frame_is_safe(cls, value):
        return safe_id(value)

    @field_validator('polygon')
    @classmethod
    def is_a_polygon(cls, value):
        if any(len(p) != 2 for p in value):
            raise ValueError('each polygon point is [x, y] in metres')
        _finite_points(value)
        area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(value, value[1:] + value[:1])) / 2
        if abs(area) < 1e-6:
            raise ValueError('the polygon has no area')
        return value


class Obstacles(BaseModel):
    """Footprints of known objects Nav2 may not see (a ladder lying on the floor): cell centres, replaced
    on every PUT. The bridge republishes them as a point cloud for an obstacle layer."""
    frame_id: str = 'map'
    cell: float = Field(gt=0, le=2.0, allow_inf_nan=False)
    cells: list[list[float]] = Field(max_length=50000)

    @field_validator('frame_id')
    @classmethod
    def frame_is_safe(cls, value):
        return safe_id(value)

    @field_validator('cells')
    @classmethod
    def are_points(cls, value):
        if any(len(p) != 2 for p in value):
            raise ValueError('each cell is [x, y] in metres, its centre')
        return _finite_points(value)


class SiteCheckAnswer(str, Enum):
    """The form's own columns as the pipeline answers them (ultron.pipeline.checklists)."""
    YES = 'ja'
    NO = 'nein'
    OPEN = 'offen'     # the photos could not settle it


class SiteCheckRow(BaseModel):
    answer: SiteCheckAnswer
    remark: str = Field('', max_length=1000)
    confidence: float | None = Field(None, ge=0, le=1, allow_inf_nan=False)
    photo: str | None = None   # an id uploaded to /api/missions/{id}/photos/{photo_id}

    @field_validator('photo')
    @classmethod
    def photo_is_safe(cls, value):
        return None if value is None else safe_id(value)


class SiteCheck(BaseModel):
    """One non-ladder object a patrol found, with its checklist rows (nesseler's other checklists).

    Only kinds the checklists name are taken, and a row only from an object of a kind that row is about.
    """
    object_id: str
    kind: str
    position: list[float] | None = Field(None, min_length=2, max_length=2)
    verified: bool = False          # the vision model confirmed the object is this kind
    rows: dict[str, SiteCheckRow] = {}
    remarks: str = Field('', max_length=2000)
    photos: list[str] = []

    @field_validator('object_id')
    @classmethod
    def object_id_is_safe(cls, value):
        return safe_id(value)

    @field_validator('photos')
    @classmethod
    def photos_are_safe(cls, value):
        return [safe_id(p) for p in value]

    @model_validator(mode='after')
    def rows_fit_the_kind(self):
        from .site_checklists import ROWS, kinds
        if self.kind not in kinds():
            raise ValueError(f'unknown kind {self.kind!r}; known: {", ".join(kinds())}')
        for row_id in self.rows:
            if row_id not in ROWS:
                raise ValueError(f'unknown checklist row {row_id!r}')
            if self.kind not in ROWS[row_id].kinds:
                raise ValueError(f'row {row_id} is not answered by a {self.kind}')
        return self


class LadderData(BaseModel):
    """Page 1 of the nesseler form. Unread fields stay None and print as blank."""
    inventory_no: str | None = None
    ascent_type: str | None = None
    material: str | None = None
    rungs: int | None = Field(default=None, ge=1)
    length_m: float | None = Field(default=None, gt=0)
    manufacturer: str | None = None
    article_no: str | None = None
    sticker_due: str | None = Field(default=None, description='Next inspection from the sticker, YYYY-MM or YYYY-MM-DD')
    sticker_status: str | None = Field(default=None, description='VALID, OVERDUE or UNREADABLE')

    @field_validator('ascent_type')
    @classmethod
    def known_ascent(cls, value):
        if value is not None and value not in ASCENT_TYPES:
            raise ValueError(f'ascent_type must be one of {ASCENT_TYPES}')
        return value

    @field_validator('material')
    @classmethod
    def known_material(cls, value):
        if value is not None and value not in MATERIALS:
            raise ValueError(f'material must be one of {MATERIALS}')
        return value


class Photo(BaseModel):
    id: str
    caption: str = ''
    camera: str = ''

    @field_validator('id')
    @classmethod
    def id_is_safe(cls, value):
        return safe_id(value)


class CriterionDetail(BaseModel):
    reason: str = Field(min_length=1, max_length=4000)
    confidence: float = Field(ge=0, le=1, allow_inf_nan=False, description='Uncalibrated model score, not safe-use probability')


class Inspection(BaseModel):
    """One ladder verdict in one mission. Bulky sim internals (views, geoms) are dropped."""
    model_config = ConfigDict(extra='ignore')

    mission_id: str
    asset_id: str
    ladder_type: str
    status: Verdict
    reason: str
    time: float | None = Field(default=None, ge=0, description='Mission time of the verdict')
    checks: dict[str, CheckResult] = {}
    checklist: dict[str, ItemResult] = {}
    photos: list[Photo] = []
    item_photos: dict[str, list[str]] = {}
    criterion_details: dict[str, CriterionDetail] = {}
    assessment_source: str | None = None
    view_coverage: dict[str, Any] = {}
    retained_findings: dict[str, Any] = {}
    ladder: LadderData = LadderData()
    missing: list[str] = []
    label_status: str | None = None
    position: list[float] = Field(min_length=3, max_length=3)
    safe_to_use: bool = False
    tactile_checks_pending: bool = True
    human_signoff_required: bool = True

    @field_validator('asset_id')
    @classmethod
    def asset_is_safe(cls, value):
        return safe_id(value)

    @field_validator('checklist')
    @classmethod
    def known_items(cls, value):
        unknown = set(value) - set(ALL_KEYS)
        if unknown:
            raise ValueError(f'unknown checklist items: {sorted(unknown)}')
        return value

    @field_validator('safe_to_use')
    @classmethod
    def never_certified(cls, value):
        if value:
            raise ValueError('safe_to_use must be false: the system only pre-screens')
        return value

    @field_validator('tactile_checks_pending', 'human_signoff_required')
    @classmethod
    def human_steps_stay_open(cls, value):
        if not value:
            raise ValueError('tactile checks and human sign-off stay with a person')
        return value

    @model_validator(mode='after')
    def verdict_matches_evidence(self):
        results = set(self.checks.values())
        items = set(self.checklist.values())
        overdue = 'OVERDUE' in (self.label_status, self.ladder.sticker_status)
        if (CheckResult.FAIL in results or ItemResult.DEFECT in items or overdue) and self.status is not Verdict.FLAGGED:
            raise ValueError('a defect or overdue inspection sticker must be FLAGGED')
        if (CheckResult.UNKNOWN in results or ItemResult.UNKNOWN in items) and self.status is Verdict.AUTOMATED_PASS:
            raise ValueError('an unknown check can never be AUTOMATED_PASS')
        if not set(self.criterion_details) <= set(ALL_KEYS):
            raise ValueError('Unknown criterion detail key')
        photo_ids = {p.id for p in self.photos}
        for key, refs in self.item_photos.items():
            if key not in ALL_KEYS or not set(refs) <= photo_ids:
                raise ValueError(f'item_photos[{key}] must reference a known item and listed photos')
        return self
