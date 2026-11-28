from glob import glob

from setuptools import setup

package_name = 'ultron_b2_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ULTRON team',
    maintainer_email='ultron@example.invalid',
    description='Ladder detection (ORACLE in sim / learned with local weights), localization and map-scoped tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'ladder_detector = ultron_b2_perception.ladder_detector:main',
        'ladder_localizer = ultron_b2_perception.ladder_localizer:main',
        'ladder_tracker = ultron_b2_perception.ladder_tracker:main',
    ]},
)
