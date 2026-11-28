"""Common final motion gate decision logic (no ROS). Port of ``ultron.autonomy.navigation_contract.MotionGate``.

Every velocity source passes through here last (after twist_mux and collision_monitor). Default is
"no motion": any missing, stale, invalid or over-limit input yields a zero command and a reason.

Checks, in order (first failure wins):
  HOLD                      ops/hold lock true (or hold reported as active source)
  OPERATOR_ENABLE_*         heartbeat missing / disabled / older than operator_enable_max_age_s
  SCAN_MISSING_OR_STALE     scan required for simulation mapping teleop
  NO_MAP_LOADED / LOCALIZATION: ...   only for sources that require localization
  NO_COMMAND / COMMAND_STALE          command age > command_max_age_s
  COMMAND_NONFINITE / COMMAND_OVER_LIMIT

Localization is required for ``autonomy`` always and for ``teleop`` when
``teleop_requires_localization`` (default True, fail-closed). Unknown, ``none`` or stale active
source is treated as ``autonomy`` (strictest). A source change toward a less strict source only
takes effect after ``source_hysteresis_s`` of the strict source not being seen.
"""
from dataclasses import dataclass, field
import math

from .ultron_path import ensure_ultron

ensure_ultron()
from ultron.autonomy.localization_state import LocalizationState, accept_pose  # noqa: E402

KNOWN_SOURCES = ('hold', 'teleop', 'autonomy', 'none')
DEFAULTS = {
    'operator_enable_max_age_s': 1.0,
    'command_max_age_s': 0.3,
    'source_max_age_s': 0.5,          # PROPOSED: active_source older than this => autonomy
    'source_hysteresis_s': 0.1,       # PROPOSED
    'teleop_requires_localization': True,
    'teleop_requires_fresh_scan': False,
    'teleop_scan_max_age_s': 0.5,     # PROPOSED simulation-only check
    # PROPOSED limits (sim); real values TO_VERIFY with the B2 sport-mode envelope.
    'max_vx': 0.6, 'min_vx': -0.3, 'max_vy': 0.3, 'max_wz': 0.8,
}


def localization_from_dict(d):
    """Contract dict (ultron_b2_contracts.msg_to_localization_state) -> LocalizationState."""
    fields = set(LocalizationState.__dataclass_fields__)
    return LocalizationState(**{k: v for k, v in d.items() if k in fields})


@dataclass
class Decision:
    open: bool
    reason: str
    source: str                  # effective source used for the decision
    cmd_out: tuple = (0.0, 0.0, 0.0)


@dataclass
class FinalGate:
    params: dict = field(default_factory=lambda: dict(DEFAULTS))
    map_id: str = ''
    map_version: int = 0
    hold: bool = False
    operator_enabled: bool | None = None
    operator_stamp: float | None = None
    localization: LocalizationState | None = None
    source: str | None = None
    source_stamp: float | None = None
    command: tuple | None = None
    command_stamp: float | None = None
    scan_stamp: float | None = None
    _strict_seen: float | None = None

    def __post_init__(self):
        self.params = {**DEFAULTS, **(self.params or {})}

    # ------------------------------------------------------------------ inputs
    def set_command(self, vx, vy, wz, now):
        self.command, self.command_stamp = (float(vx), float(vy), float(wz)), now

    def set_operator(self, enabled, now):
        self.operator_enabled, self.operator_stamp = bool(enabled), now

    def set_source(self, source, now):
        self.source, self.source_stamp = str(source), now

    # ------------------------------------------------------------------ decision
    def effective_source(self, now):
        s = self.source
        if s is None or self.source_stamp is None or now - self.source_stamp > self.params['source_max_age_s']:
            s = 'autonomy'                         # stale / missing => strictest
        elif s not in KNOWN_SOURCES or s == 'none':
            s = 'autonomy'                         # unknown => strictest
        if s in ('autonomy', 'hold'):
            self._strict_seen = now
        elif self._strict_seen is not None and now - self._strict_seen < self.params['source_hysteresis_s']:
            s = 'autonomy'
        return s

    def requires_localization(self, source):
        if source == 'teleop':
            return bool(self.params['teleop_requires_localization'])
        return True

    def decide(self, now):
        source = self.effective_source(now)
        reason = self._reason(source, now)
        if reason:
            return Decision(False, reason, source)
        return Decision(True, '', source, self.command)

    def _reason(self, source, now):
        p = self.params
        if self.hold or source == 'hold':
            return 'HOLD'
        if self.operator_enabled is None or self.operator_stamp is None:
            return 'OPERATOR_ENABLE_MISSING'
        if not self.operator_enabled:
            return 'OPERATOR_DISABLED'
        if now - self.operator_stamp > p['operator_enable_max_age_s']:
            return 'OPERATOR_ENABLE_STALE'
        if source == 'teleop' and p['teleop_requires_fresh_scan']:
            if self.scan_stamp is None or not math.isfinite(self.scan_stamp) \
                    or not 0 <= now - self.scan_stamp <= p['teleop_scan_max_age_s']:
                return 'SCAN_MISSING_OR_STALE'
        if self.requires_localization(source):
            if not self.map_id:
                return 'NO_MAP_LOADED'
            ok, why = accept_pose(self.localization, self.map_id, self.map_version, now)
            if not ok:
                return f'LOCALIZATION: {why}'
        if self.command is None or self.command_stamp is None:
            return 'NO_COMMAND'
        if now - self.command_stamp > p['command_max_age_s']:
            return 'COMMAND_STALE'
        if not all(math.isfinite(v) for v in self.command):
            return 'COMMAND_NONFINITE'
        vx, vy, wz = self.command
        if vx > p['max_vx'] + 1e-6 or vx < p['min_vx'] - 1e-6 or abs(vy) > p['max_vy'] + 1e-6 \
                or abs(wz) > p['max_wz'] + 1e-6:
            return f'COMMAND_OVER_LIMIT: ({vx:.3f}, {vy:.3f}, {wz:.3f})'
        return None
