"""Shared launch construction for sim / replay / live (owner: Agent A).

Order: preflight (plain process, no DDS participant) -> on exit code 0 the namespaced stack
starts; otherwise the launch shuts down. Replay never includes command-producing sub-launches.
All stack nodes are pushed into the mode namespace and use relative topics; TF stays on /tf.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

from .modes import (GROUND_TRUTH_FRAMES, MODES, ODOM_SOURCES, RECORD_OPTIONS, STACK_MODES, SUB_LAUNCHES,
                    VIZ_OPTIONS, sub_launches_for)

PKG = 'ultron_b2_bringup'
# Arguments every sub-launch receives (the sub-launch contract for agents B-F).
SUB_ARGS = ('run_mode', 'mode', 'odom_source', 'viz', 'record', 'bag', 'hardware_enable', 'use_sim_time',
            'calibration_profile', 'publish_description')


def declare_arguments(run_mode):
    spec = MODES[run_mode]
    args = [
        DeclareLaunchArgument('mode', default_value='mapping', choices=list(STACK_MODES),
                              description='Stack mode'),
        DeclareLaunchArgument('odom_source', default_value=spec['default_odom_source'], choices=list(ODOM_SOURCES),
                              description=f'odom->base_link authority; allowed here: {spec["odom_sources"]}'),
        DeclareLaunchArgument('viz', default_value='none', choices=list(VIZ_OPTIONS)),
        DeclareLaunchArgument('record', default_value='none', choices=list(RECORD_OPTIONS)),
        DeclareLaunchArgument('hardware_enable', default_value='false', choices=['true', 'false'],
                              description='Live only; also needs the operator hardware token'),
        DeclareLaunchArgument('hw_token', default_value='',
                              description='Hardware token path (default $ULTRON_HW_AUTHORIZED_FILE or /etc/ultron/...)'),
        DeclareLaunchArgument('bag', default_value='', description='rosbag2 path (replay)'),
        DeclareLaunchArgument('namespace', default_value=spec['namespace'],
                              description='Must equal the mode namespace; preflight refuses otherwise'),
        DeclareLaunchArgument('use_sim_time', default_value=str(spec['use_sim_time']).lower(),
                              choices=['true', 'false']),
        DeclareLaunchArgument('calibration_profile', default_value=spec['calibration_profile'],
                              choices=['sim', 'real']),
        DeclareLaunchArgument('publish_description', default_value='true', choices=['true', 'false'],
                              description='robot_state_publisher for base_link->sensor frames'),
        DeclareLaunchArgument('tf_check', default_value='true', choices=['true', 'false'],
                              description='Run tf_authority_check; a violation shuts the launch down'),
    ]
    if run_mode == 'sim':
        args.append(DeclareLaunchArgument('sim_teleop_without_localization', default_value='false',
                                          choices=['true', 'false'],
                                          description='SIM mapping only: allow gated teleop without a saved map'))
    return args


def _include(name, run_mode):
    source = PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(PKG), 'launch', 'sub',
                                                                 f'{name}.launch.py']))
    args = {k: LaunchConfiguration(k) for k in SUB_ARGS if k != 'run_mode'}
    args['run_mode'] = run_mode
    include = IncludeLaunchDescription(source, launch_arguments=list(args.items()))
    include.ultron_sub_launch = name
    return include


def _shutdown_on_failure(reason):
    def handler(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg=f'[{PKG}] {reason} (exit code {event.returncode}); shutting down'),
                    EmitEvent(event=Shutdown(reason=reason))]
        return None
    return handler


def build_stack(run_mode):
    """Namespaced stack (everything after preflight)."""
    tf_check = Node(package=PKG, executable='tf_authority_check', name='tf_authority_check', output='screen',
                    parameters=[{'forbidden_frames': list(GROUND_TRUTH_FRAMES), 'fail_on_violation': True}],
                    condition=IfCondition(LaunchConfiguration('tf_check')))
    actions = [PushRosNamespace(LaunchConfiguration('namespace')),
               SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))]
    actions += [_include(name, run_mode) for name in sub_launches_for(run_mode)]
    actions += [tf_check,
                RegisterEventHandler(OnProcessExit(target_action=tf_check,
                                                   on_exit=_shutdown_on_failure('TF authority violation')))]
    return GroupAction(actions, scoped=True)


def generate(run_mode):
    spec = MODES[run_mode]
    preflight = ExecuteProcess(
        name='preflight',
        cmd=[PathJoinSubstitution([FindPackagePrefix(PKG), 'lib', PKG, 'preflight']),
             '--mode', run_mode,
             '--namespace', LaunchConfiguration('namespace'),
             '--hardware-enable', LaunchConfiguration('hardware_enable'),
             '--odom-source', LaunchConfiguration('odom_source'),
             '--launch-control', str(spec['control']).lower(),
             '--hw-token', LaunchConfiguration('hw_token')],
        output='screen')
    stack = build_stack(run_mode)

    def after_preflight(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg=f'[{PKG}] preflight passed; starting {run_mode} stack'), stack]
        return [LogInfo(msg=f'[{PKG}] preflight REFUSED {run_mode} (exit {event.returncode}); nothing started'),
                EmitEvent(event=Shutdown(reason='preflight refused'))]

    return LaunchDescription(declare_arguments(run_mode) + [
        LogInfo(msg=f'[{PKG}] {run_mode}: sub-launches {", ".join(sub_launches_for(run_mode))}'),
        preflight,
        RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=after_preflight)),
    ])


def placeholder_launch(component, owner):
    """Stub sub-launch: declares the sub-launch contract args and starts one silent placeholder node."""
    declared = [DeclareLaunchArgument(k, default_value='') for k in SUB_ARGS]
    node = Node(package=PKG, executable='placeholder', name=f'{component}_placeholder', output='screen',
                parameters=[{'component': component, 'owner': owner}])
    return LaunchDescription(declared + [node])


OWNERS = dict(SUB_LAUNCHES)


def saved_map_paths(cfg):
    """(map_yaml, map_meta) for nav/control from ``map:=`` (C's map store) unless given explicitly.

    Keeps Nav2's static map, the zone masks and the gate's map id on the same saved map that slam_toolbox
    localizes against. Returns empty strings when no ``map:=`` is set (Nav2 then uses the live ``map``).
    """
    map_yaml, map_meta, ref = cfg.get('map_yaml', ''), cfg.get('map_meta', ''), cfg.get('map', '')
    if ref and not (map_yaml and map_meta):
        import os
        from ultron_b2_localization import map_store
        from ultron_b2_localization.map_manager_node import default_map_root, parse_map_ref
        map_id, version, path = parse_map_ref(ref)
        if path and os.path.isdir(path):
            path = os.path.join(path, 'map.yaml')
        root = os.path.expanduser(cfg.get('map_root', '') or default_map_root())
        directory = map_store.resolve(root, map_id, version, path)
        map_yaml = map_yaml or str(directory / 'map.yaml')
        map_meta = map_meta or str(directory)
    return map_yaml, map_meta
