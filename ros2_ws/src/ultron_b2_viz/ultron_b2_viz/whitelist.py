"""Single source of truth for what visualization may see (Agent F). PROPOSED; verify on site.

Used by: foxglove.launch.py (topic_whitelist regexes), gen_configs (domain_bridge YAML per mode,
zenoh allow-lists, Foxglove layouts) and the tests that keep the generated files in sync.

Rules (PLAN.md §3.2, AGENT_F.md): never raw ``sensors/lidar/points`` or raw images; clouds only
via ``viz_throttle`` (voxelized, rate-limited); images only JPEG-compressed and rate-limited.
"""
import re

MODE_NAMESPACES = {'sim': '/ultron/sim', 'replay': '/ultron/replay', 'live': '/ultron/live'}
MODE_DOMAINS = {'sim': 71, 'replay': 72, 'live': 0}
VIZ_DOMAIN = 42
VIZ_NAMESPACE = '/ultron/viz'   # mode-independent names in domain 42 (laptop RViz)
FOXGLOVE_PORT = 8765            # PROPOSED

# Relative topic -> (type, purpose). Bridged to domain 42 and allowed for Foxglove.
VIZ_TOPICS = {
    'viz/points': ('sensor_msgs/msg/PointCloud2', 'lidar, voxelized <=30k pts, <=2 Hz'),
    'viz/map_cloud': ('sensor_msgs/msg/PointCloud2', '3D map, voxelized, <=1 Hz'),
    'viz/cam_head/color/image/compressed': ('sensor_msgs/msg/CompressedImage', 'head color JPEG <=5 Hz'),
    'viz/map': ('nav_msgs/msg/OccupancyGrid', '2D map, <=1 Hz, latched'),
    'viz/global_costmap': ('nav_msgs/msg/OccupancyGrid', '<=1 Hz'),
    'viz/local_costmap': ('nav_msgs/msg/OccupancyGrid', '<=2 Hz'),
    'viz/plan': ('nav_msgs/msg/Path', '<=2 Hz'),
    'sensors/scan': ('sensor_msgs/msg/LaserScan', '2D scan (~7 kB at 10 Hz)'),
    'state/odom': ('nav_msgs/msg/Odometry', 'odometry'),
    'state/localization': ('ultron_b2_interfaces/msg/LocalizationState', 'localization state'),
    'robot_description': ('std_msgs/msg/String', 'URDF (latched)'),
    'control/active_source': ('std_msgs/msg/String', 'active command source'),
    'safety/motion_gate': ('ultron_b2_interfaces/msg/MotionGateStatus', 'gate status'),
    'perception/ladder_detections': ('ultron_b2_interfaces/msg/LadderDetectionArray', 'detections'),
    'perception/ladder_candidates': ('ultron_b2_interfaces/msg/LadderCandidateArray', 'tracked candidates'),
    'mission/status': ('ultron_b2_interfaces/msg/MissionStatus', 'mission state'),
    'mission/events': ('ultron_b2_interfaces/msg/MissionEvent', 'mission events'),
}
# Absolute topics (not namespaced).
GLOBAL_TOPICS = {
    '/tf': ('tf2_msgs/msg/TFMessage', 'TF'),
    '/tf_static': ('tf2_msgs/msg/TFMessage', 'static TF (transient local)'),
}
# Additional relative patterns Foxglove may subscribe to on the Orin (local tunnel, not bridged).
FOXGLOVE_EXTRA = (r'viz/.*', r'map', r'map_metadata', r'state/.*', r'mission/.*', r'safety/.*',
                  r'diagnostics', r'sim/bridge_status')
# Never visible to viz consumers, whatever the patterns above say.
DENY = (r'.*sensors/lidar/points(_filtered)?$', r'.*/image_raw$', r'.*/image_rect_raw$',
        r'.*/ladder_labels$', r'.*sim/ground_truth/.*')

# rviz_bridge/zenoh rate caps in Hz (zenoh `pub_max_frequencies`); viz_throttle enforces them first.
MAX_HZ = {'viz/points': 2, 'viz/map_cloud': 1, 'viz/cam_head/color/image/compressed': 5, 'viz/map': 1,
          'viz/global_costmap': 1, 'viz/local_costmap': 2, 'viz/plan': 2, 'sensors/scan': 10, 'state/odom': 10}


def _ns_pattern():
    return '/ultron/(?:' + '|'.join(MODE_NAMESPACES) + ')/'


def foxglove_whitelist():
    """Regexes for foxglove_bridge ``topic_whitelist`` (full-match, all modes)."""
    rel = sorted({re.escape(t) for t in VIZ_TOPICS} | set(FOXGLOVE_EXTRA))
    return [_ns_pattern() + '(?:' + '|'.join(rel) + ')'] + [re.escape(t) for t in GLOBAL_TOPICS]


def is_denied(topic):
    return any(re.fullmatch(p, topic) for p in DENY)


def foxglove_allows(topic):
    """What foxglove_bridge exposes (it full-matches each whitelist regex; it has no deny list)."""
    return any(re.fullmatch(p, topic) for p in foxglove_whitelist())


def viz42_name(rel):
    """Domain-42 name for a relative stack topic: ``viz/points`` -> ``/ultron/viz/points``."""
    return f'{VIZ_NAMESPACE}/{rel[4:] if rel.startswith("viz/") else rel}'


def bridge_topics(mode):
    """(source topic, type, destination topic) bridged from the mode domain to domain 42."""
    ns = MODE_NAMESPACES[mode]
    out = [(f'{ns}/{rel}', typ, viz42_name(rel)) for rel, (typ, _) in VIZ_TOPICS.items()]
    out += [(t, typ, t) for t, (typ, _) in GLOBAL_TOPICS.items()]
    for src, _, _ in out:
        assert not is_denied(src), src
    return out
