"""Data contract between the robot side (sim or ROS) and the dashboard."""
import re
from enum import Enum
from typing import Any

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
