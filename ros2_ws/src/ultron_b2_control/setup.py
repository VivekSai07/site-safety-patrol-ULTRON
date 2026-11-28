from glob import glob

from setuptools import setup

package_name = 'ultron_b2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ULTRON team',
    maintainer_email='ultron@example.invalid',
    description='twist_mux chain, active_source_monitor, teleop, sim_driver, b2_sport_bridge (mock-tested only) (Agent D)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'active_source_monitor = ultron_b2_control.active_source_monitor:main',
        'sim_driver = ultron_b2_control.sim_driver:main',
        'b2_sport_bridge = ultron_b2_control.b2_sport_bridge:main',
        'mock_sport_server = ultron_b2_control.mock_sport_server:main',
    ]},
)
