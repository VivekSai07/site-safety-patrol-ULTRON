"""Sub-launch `record` (owner: Agent F): rosbag2 mcap + zstd with profiles minimal/debug/full.

Profiles: config/record/profiles.yaml; storage options: config/record/mcap_zstd.yaml.
Output: runs/bags/<run_mode>_<profile>_<UTC stamp>/ ($ULTRON_RUNS_DIR overrides runs/).
`record:=none` starts nothing.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction

from ultron_b2_bringup.launch_common import SUB_ARGS
from ultron_b2_bringup.modes import MODES
from ultron_b2_bringup.record_profiles import bag_output, load_profiles, profile_regex


def _record(context):
    cfg = context.launch_configurations
    profile = cfg.get('record', 'none') or 'none'
    if profile == 'none':
        return []
    run_mode = cfg['run_mode']
    share = os.path.join(get_package_share_directory('ultron_b2_bringup'), 'config', 'record')
    use_sim_time = cfg.get('use_sim_time', 'false') == 'true'
    regex = profile_regex(load_profiles(os.path.join(share, 'profiles.yaml')), profile,
                          MODES[run_mode]['namespace'], use_sim_time)
    out = bag_output(run_mode, profile)
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '--storage-config-file', os.path.join(share, 'mcap_zstd.yaml'),
           '-o', str(out), '-e', regex, '--node-name', f'ultron_recorder_{run_mode}', '-p', '500']
    if use_sim_time:
        cmd.append('--use-sim-time')
    return [LogInfo(msg=f'[record] profile={profile} -> {out}'),
            ExecuteProcess(name='ultron_recorder', cmd=cmd, output='screen', sigterm_timeout='10')]


def generate_launch_description():
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    return LaunchDescription(declared + [OpaqueFunction(function=_record)])
