from glob import glob

from setuptools import setup

package_name = 'ultron_b2_localization'

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
    description='ULTRON B2 state estimation, SLAM configs, LocalizationState monitor and map manager (Agent C)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'kiss_icp_odometry = ultron_b2_localization.kiss_icp_node:main',
        'b2_odom_adapter = ultron_b2_localization.b2_odom_adapter:main',
        'localization_monitor = ultron_b2_localization.localization_monitor_node:main',
        'map_manager = ultron_b2_localization.map_manager_node:main',
        'odom_planarizer = ultron_b2_localization.odom_planarizer:main',
    ]},
)
