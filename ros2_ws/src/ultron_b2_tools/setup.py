from setuptools import setup

package_name = 'ultron_b2_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ULTRON team',
    maintainer_email='ultron@example.invalid',
    description='Bag inspection and stream measurement tools for the ULTRON B2 stack',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'stream_probe = ultron_b2_tools.stream_probe:main',
        'bag_inspect = ultron_b2_tools.bag_inspect:main',
    ]},
)
