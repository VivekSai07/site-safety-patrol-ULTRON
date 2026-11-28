from glob import glob

from setuptools import setup

package_name = 'ultron_b2_preprocess'

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
    description='Lidar preprocessing: self box, range/height crop, voxel, optional deskew; laserscan config',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['cloud_filter = ultron_b2_preprocess.cloud_filter:main']},
)
