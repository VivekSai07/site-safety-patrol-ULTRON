"""Sub-launch `viz` (owner: Agent F): viz:={none,foxglove,rviz_bridge,both}.

foxglove    -> viz_throttle + foxglove_bridge (127.0.0.1:8765, whitelist only)
rviz_bridge -> viz_throttle + domain_bridge (stack domain -> 42, whitelist only)
both        -> all three; none -> nothing. Extra args: foxglove_port, foxglove_address, viz_zenoh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ultron_b2_bringup.launch_common import SUB_ARGS


def _viz(context):
    cfg = context.launch_configurations
    viz = cfg.get('viz', 'none') or 'none'
    if viz == 'none':
        return []
    share = get_package_share_directory('ultron_b2_viz')

    def include(name, **args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
                                        launch_arguments=list(args.items()))
    actions = [include('viz_throttle.launch.py')]
    if viz in ('foxglove', 'both'):
        actions.append(include('foxglove.launch.py', foxglove_port=cfg.get('foxglove_port', '8765'),
                               foxglove_address=cfg.get('foxglove_address', '127.0.0.1')))
    if viz in ('rviz_bridge', 'both'):
        actions.append(include('rviz_bridge.launch.py', run_mode=cfg['run_mode'],
                               zenoh=cfg.get('viz_zenoh', 'false')))
    return actions


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    return LaunchDescription(declared + [OpaqueFunction(function=_viz)])
