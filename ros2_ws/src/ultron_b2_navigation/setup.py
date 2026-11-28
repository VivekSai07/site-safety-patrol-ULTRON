from glob import glob

from setuptools import setup

package_name = 'ultron_b2_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/bt', glob('config/bt/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ULTRON team',
    maintainer_email='ultron@example.invalid',
    description='Nav2 configs, patrol-zone masks, common motion_gate and NavigateToView contract server (Agent D)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'motion_gate = ultron_b2_navigation.motion_gate:main',
        'nav_contract_server = ultron_b2_navigation.nav_contract_server:main',
        'zone_mask_server = ultron_b2_navigation.zone_mask_server:main',
        'zone_masks_tool = ultron_b2_navigation.zone_masks_tool:main',
        'sim_glue = ultron_b2_navigation.sim_glue:main',
        'sim_nav_cases = ultron_b2_navigation.sim_nav_cases:main',
    ]},
)
