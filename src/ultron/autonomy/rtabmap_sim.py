"""ROS 2 names for the isolated MuJoCo RTAB-Map experiment."""

from .config import CONFIG_DIR

PARAMS = CONFIG_DIR / 'rtabmap_lidar3d.yaml'
ODOM = '/ultron/sim/rtabmap/odom'
CLOUD_MAP = '/ultron/sim/rtabmap/cloud_map'
# Keep the simulated braking time within the existing stop budget after a stale-pose decision.
LINEAR_SPEED_M_S = 0.28
