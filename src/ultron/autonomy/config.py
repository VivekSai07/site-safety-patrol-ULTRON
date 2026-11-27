"""Strict loaders for the simulation lidar profile and SLAM configuration.

Unknown or missing keys are errors so that a typo cannot silently fall back to a
default. Values are simulation choices unless their recorded source says otherwise.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parents[3] / 'config' / 'autonomy'
SOURCES = {'datasheet', 'organizer', 'measured', 'assumed'}
# Only simulation-namespaced topics plus ROS infrastructure may be configured.
ALLOWED_TOPIC_PREFIXES = ('/ultron/sim/',)
ALLOWED_TOPICS = {'/clock'}


class ConfigError(ValueError):
    pass


def _load_yaml(path):
    import yaml
    path = Path(path)
    text = path.read_text()
    data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ConfigError(f'{path}: top level must be a mapping')
    return data, hashlib.sha256(text.encode()).hexdigest()


def _keys(section, expected, where):
    if not isinstance(section, dict):
        raise ConfigError(f'{where}: expected a mapping')
    missing = set(expected) - set(section)
    unknown = set(section) - set(expected)
    if missing or unknown:
        raise ConfigError(f'{where}: missing {sorted(missing)} unknown {sorted(unknown)}')


def _number(value, where, *, minimum=None, maximum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f'{where}: finite number required, got {value!r}')
    if positive and value <= 0:
        raise ConfigError(f'{where}: must be positive')
    if minimum is not None and value < minimum:
        raise ConfigError(f'{where}: must be >= {minimum}')
    if maximum is not None and value > maximum:
        raise ConfigError(f'{where}: must be <= {maximum}')
    return float(value)


def _vector(value, n, where):
    if not isinstance(value, list) or len(value) != n:
        raise ConfigError(f'{where}: list of {n} numbers required')
    return tuple(_number(v, where) for v in value)


@dataclass(frozen=True)
class LidarProfile:
    name: str
    variant: str
    channels: int
    vertical_angles_deg: tuple
    horizontal_resolution_deg: float
    scan_rate_hz: float
    range_min_m: float
    range_max_m: float
    range_noise_std_m: float
    dropout_probability: float
    mount_xyz_m: tuple
    mount_rpy_rad: tuple
    sources: dict
    sha256: str

    @property
    def columns(self):
        return int(round(360.0 / self.horizontal_resolution_deg))

    @property
    def unconfirmed(self):
        """Parameters still lacking organizer, datasheet or measured provenance."""
        return sorted(k for k, s in self.sources.items() if s == 'assumed')


LIDAR_KEYS = ('schema', 'name', 'variant', 'channels', 'vertical_fov_deg', 'vertical_distribution',
              'horizontal_fov_deg', 'horizontal_resolution_deg', 'scan_rate_hz', 'range_min_m',
              'range_max_m', 'range_noise_std_m', 'dropout_probability', 'mount_xyz_m', 'mount_rpy_rad')


def load_lidar_profile(path=CONFIG_DIR / 'lidar_helios32.yaml'):
    data, digest = _load_yaml(path)
    _keys(data, LIDAR_KEYS, str(path))
    if data['schema'] != 'ultron.lidar_profile/v1':
        raise ConfigError(f'Unsupported lidar schema {data["schema"]!r}')
    values, sources = {}, {}
    for key in LIDAR_KEYS[2:]:
        entry = data[key]
        _keys(entry, ('value', 'source'), f'{path}:{key}')
        if entry['source'] not in SOURCES:
            raise ConfigError(f'{key}: source must be one of {sorted(SOURCES)}')
        values[key], sources[key] = entry['value'], entry['source']
    channels = values['channels']
    if isinstance(channels, bool) or not isinstance(channels, int) or not 1 <= channels <= 256:
        raise ConfigError('channels: integer 1..256 required')
    low, high = _vector(values['vertical_fov_deg'], 2, 'vertical_fov_deg')
    if not -90 <= low < high <= 90:
        raise ConfigError('vertical_fov_deg: need -90 <= low < high <= 90')
    if values['vertical_distribution'] != 'uniform':
        raise ConfigError('vertical_distribution: only "uniform" is implemented')
    if _number(values['horizontal_fov_deg'], 'horizontal_fov_deg') != 360.0:
        raise ConfigError('horizontal_fov_deg: only 360 is implemented')
    resolution = _number(values['horizontal_resolution_deg'], 'horizontal_resolution_deg', positive=True, maximum=10)
    if abs(360.0 / resolution - round(360.0 / resolution)) > 1e-6:
        raise ConfigError('horizontal_resolution_deg must divide 360')
    rmin = _number(values['range_min_m'], 'range_min_m', positive=True)
    rmax = _number(values['range_max_m'], 'range_max_m', positive=True)
    if rmin >= rmax:
        raise ConfigError('range_min_m must be below range_max_m')
    step = (high - low) / (channels - 1) if channels > 1 else 0.0
    return LidarProfile(
        name=str(data['name']), variant=str(values['variant']), channels=channels,
        vertical_angles_deg=tuple(low + i * step for i in range(channels)),
        horizontal_resolution_deg=resolution,
        scan_rate_hz=_number(values['scan_rate_hz'], 'scan_rate_hz', positive=True, maximum=50),
        range_min_m=rmin, range_max_m=rmax,
        range_noise_std_m=_number(values['range_noise_std_m'], 'range_noise_std_m', minimum=0, maximum=1),
        dropout_probability=_number(values['dropout_probability'], 'dropout_probability', minimum=0, maximum=1),
        mount_xyz_m=_vector(values['mount_xyz_m'], 3, 'mount_xyz_m'),
        mount_rpy_rad=_vector(values['mount_rpy_rad'], 3, 'mount_rpy_rad'),
        sources=sources, sha256=digest)


SIM_KEYS = {
    'ros': ('domain_id', 'localhost_only', 'use_sim_time'),
    'frames': ('map', 'odom', 'base', 'lidar'),
    'self_filter': ('min_xyz_m', 'max_xyz_m'),
    'topics': ('clock', 'points', 'scan', 'odom', 'ground_truth', 'cmd_vel', 'localization_state', 'status'),
    'scan_projection': ('z_min_m', 'z_max_m', 'angle_increment_deg', 'range_min_m', 'range_max_m'),
    'odometry': ('rate_hz', 'linear_scale_error', 'yaw_rate_bias_rad_s', 'linear_noise_std',
                 'angular_noise_std', 'seed'),
    'driver': ('max_linear_m_s', 'max_angular_rad_s', 'max_linear_accel_m_s2',
               'max_angular_accel_rad_s2', 'command_timeout_s', 'footprint_radius_m'),
}


@dataclass(frozen=True)
class SimSlamConfig:
    ros: dict
    frames: dict
    self_filter: dict
    topics: dict
    scan_projection: dict
    odometry: dict
    driver: dict
    lidar: LidarProfile
    sha256: str


def load_sim_slam_config(path=CONFIG_DIR / 'sim_slam.yaml'):
    path = Path(path)
    data, digest = _load_yaml(path)
    _keys(data, ('schema', 'lidar_profile', *SIM_KEYS), str(path))
    if data['schema'] != 'ultron.sim_slam/v1':
        raise ConfigError(f'Unsupported sim_slam schema {data["schema"]!r}')
    for section, keys in SIM_KEYS.items():
        _keys(data[section], keys, f'{path}:{section}')
    ros = data['ros']
    if isinstance(ros['domain_id'], bool) or not isinstance(ros['domain_id'], int) or not 0 <= ros['domain_id'] <= 101:
        raise ConfigError('ros.domain_id: integer 0..101 required')
    if ros['localhost_only'] is not True or ros['use_sim_time'] is not True:
        raise ConfigError('Simulation requires localhost_only and use_sim_time to be true')
    frames = data['frames']
    if len(set(frames.values())) != len(frames) or not all(isinstance(f, str) and f for f in frames.values()):
        raise ConfigError('frames: distinct non-empty names required')
    for name, topic in data['topics'].items():
        if not isinstance(topic, str) or not (topic in ALLOWED_TOPICS or topic.startswith(ALLOWED_TOPIC_PREFIXES)):
            raise ConfigError(f'topics.{name}: {topic!r} is outside the simulation namespace')
    numeric = {}
    for section in ('scan_projection', 'odometry', 'driver'):
        numeric[section] = {}
        for key, value in data[section].items():
            if key == 'seed':
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ConfigError('odometry.seed: integer required')
                numeric[section][key] = value
                continue
            negative_ok = key in ('z_min_m', 'z_max_m', 'yaw_rate_bias_rad_s', 'linear_scale_error')
            numeric[section][key] = _number(value, f'{section}.{key}', minimum=None if negative_ok else 0)
    projection = numeric['scan_projection']
    if projection['z_min_m'] >= projection['z_max_m'] or projection['range_min_m'] >= projection['range_max_m']:
        raise ConfigError('scan_projection: min must be below max')
    for key in ('rate_hz',):
        if numeric['odometry'][key] <= 0:
            raise ConfigError(f'odometry.{key}: must be positive')
    for key, value in numeric['driver'].items():
        if value <= 0:
            raise ConfigError(f'driver.{key}: must be positive')
    low = _vector(data['self_filter']['min_xyz_m'], 3, 'self_filter.min_xyz_m')
    high = _vector(data['self_filter']['max_xyz_m'], 3, 'self_filter.max_xyz_m')
    if not all(a < b for a, b in zip(low, high)):
        raise ConfigError('self_filter: min must be below max on every axis')
    profile_path = data['lidar_profile']
    profile_path = Path(profile_path) if Path(profile_path).is_absolute() else path.parent / profile_path
    return SimSlamConfig(ros=dict(ros), frames=dict(frames), topics=dict(data['topics']),
                         self_filter={'min_xyz_m': low, 'max_xyz_m': high},
                         lidar=load_lidar_profile(profile_path), sha256=digest, **numeric)


@dataclass(frozen=True)
class Route:
    name: str
    scene: str
    start_pose: tuple
    final_yaw: float
    waypoints: tuple
    sha256: str


def load_route(path):
    path = Path(path)
    data, digest = _load_yaml(path)
    _keys(data, ('schema', 'scene', 'start_pose', 'final_yaw', 'waypoints'), str(path))
    if data['schema'] != 'ultron.sim_route/v1':
        raise ConfigError(f'Unsupported route schema {data["schema"]!r}')
    if not isinstance(data['waypoints'], list) or not data['waypoints']:
        raise ConfigError('waypoints: non-empty list required')
    return Route(name=path.stem, scene=str(data['scene']), start_pose=_vector(data['start_pose'], 3, 'start_pose'),
                 final_yaw=_number(data['final_yaw'], 'final_yaw'),
                 waypoints=tuple(_vector(w, 2, f'waypoints[{i}]') for i, w in enumerate(data['waypoints'])),
                 sha256=digest)
