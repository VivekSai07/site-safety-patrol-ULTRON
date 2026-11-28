from setuptools import setup

package_name = 'ultron_b2_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ULTRON team',
    maintainer_email='ultron@example.invalid',
    description='STUB owned by Agent F',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
