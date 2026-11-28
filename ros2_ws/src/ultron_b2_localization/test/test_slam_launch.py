"""Verify simulation and replay scan-buffer tuning is isolated from live localization."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


@pytest.mark.parametrize(('run_mode', 'expected'), [('sim', {'scan_queue_size': 10}),
                                                   ('replay', {'scan_queue_size': 10}), ('live', {}), ('', {})])
def test_scan_queue_override_is_simulation_only(monkeypatch, run_mode, expected):
    path = Path(__file__).resolve().parents[1] / 'launch' / 'slam.launch.py'
    spec = importlib.util.spec_from_file_location('test_slam_launch_module', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _pkg: '/package')
    monkeypatch.setattr(module, 'resolve_map', lambda *_args: ('/map', {
        'posegraph_path': '/map/graph', 'yaml_path': '/map/map.yaml', 'map_id': 'map', 'map_version': 1}))
    nodes = []
    monkeypatch.setattr(module, 'Node', lambda **kwargs: nodes.append(kwargs))
    context = LaunchContext()
    context.launch_configurations.update({'mode': 'nav', 'run_mode': run_mode, 'map': '/map', 'map_root': '',
                                          'initial_pose': '0,0,0', 'use_sim_time': 'true',
                                          'provenance': 'test', 'odom_source': 'mola'})

    module._nodes(context)
    slam = next(node for node in nodes if node['name'] == 'slam_toolbox')
    overrides = [value for value in slam['parameters'] if isinstance(value, dict)]
    assert next((value for value in overrides if 'scan_queue_size' in value), {}) == expected
