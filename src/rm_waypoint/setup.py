from setuptools import find_packages, setup

package_name = 'rm_waypoint'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dev Patel',
    maintainer_email='devsp2003@gmail.com',
    description=(
        'ROBOT. Go-to-waypoint controller for repositioning between runs. '
        'Publishes /teleop/cmd_vel only — an input to rm_safety, never /cmd_vel.'
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_node = rm_waypoint.waypoint_node:main',
        ],
    },
)
