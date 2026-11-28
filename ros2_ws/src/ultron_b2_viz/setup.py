from glob import glob

from setuptools import setup

package_name = 'ultron_b2_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json5')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ] + [('share/' + package_name + '/foxglove/' + mode, glob(f'foxglove/{mode}/*.json'))
         for mode in ('sim', 'replay', 'live')],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ULTRON team',
    maintainer_email='ultron@example.invalid',
    description='ULTRON B2 visualization: viz_throttle, Foxglove whitelist/layouts, domain_bridge to 42, zenoh, RViz',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'viz_throttle = ultron_b2_viz.viz_throttle:main',
        'topic_probe = ultron_b2_viz.topic_probe:main',
        'foxglove_probe = ultron_b2_viz.foxglove_probe:main',
        'gen_viz_configs = ultron_b2_viz.gen_configs:main',
    ]},
)
