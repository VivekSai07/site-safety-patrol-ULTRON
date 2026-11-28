"""Run-mode isolation table (docs/b2_ros_stack/FLEET_COMMON.md). PROPOSED; verify on site."""

MODES = {
    'sim': {'namespace': '/ultron/sim', 'domain_id': 71, 'localhost_only': True,
            'control': True, 'hardware_allowed': False, 'use_sim_time': True,
            'odom_sources': ('sim', 'mola', 'kiss'), 'default_odom_source': 'sim',
            'calibration_profile': 'sim'},
    'replay': {'namespace': '/ultron/replay', 'domain_id': 72, 'localhost_only': True,
               'control': False, 'hardware_allowed': False, 'use_sim_time': True,
               'odom_sources': ('mola', 'kiss'), 'default_odom_source': 'mola',
               'calibration_profile': 'real'},
    # Live uses the robot network domain. Only with explicit user/operator authorization.
    'live': {'namespace': '/ultron/live', 'domain_id': 0, 'localhost_only': False,
             'control': True, 'hardware_allowed': True, 'use_sim_time': False,
             'odom_sources': ('mola', 'kiss', 'b2'), 'default_odom_source': 'mola',
             'calibration_profile': 'real'},
}
VIZ_DOMAIN_ID = 42

STACK_MODES = ('mapping', 'localization', 'nav', 'mission')
ODOM_SOURCES = ('sim', 'mola', 'kiss', 'b2')
VIZ_OPTIONS = ('none', 'foxglove', 'rviz_bridge', 'both')
RECORD_OPTIONS = ('none', 'minimal', 'debug', 'full')

# Sub-launch files in launch order, with the owning fleet agent.
SUB_LAUNCHES = (
    ('description', 'A'),
    ('drivers', 'B (live/replay) / F (sim)'),
    ('preprocess', 'B'),
    ('state_estimation', 'C'),
    ('slam', 'C'),
    ('nav', 'D'),
    ('control', 'D'),
    ('mission', 'E'),
    ('viz', 'F'),
    ('record', 'F'),
)
# Sub-launches that can produce or forward velocity commands. Never launched in replay.
CONTROL_SUB_LAUNCHES = ('nav', 'control', 'mission')

GROUND_TRUTH_FRAMES = ('ultron_sim_world', 'ground_truth', 'gt', 'world_gt')

DEFAULT_HW_TOKEN = '/etc/ultron/ULTRON_HW_AUTHORIZED'
HW_TOKEN_ENV = 'ULTRON_HW_AUTHORIZED_FILE'


def sub_launches_for(run_mode):
    """Sub-launch names included for ``run_mode``; replay never includes command-producing launches."""
    control = MODES[run_mode]['control']
    return tuple(name for name, _ in SUB_LAUNCHES if control or name not in CONTROL_SUB_LAUNCHES)
