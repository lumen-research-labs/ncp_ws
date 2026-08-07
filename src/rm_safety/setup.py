from setuptools import find_packages, setup

package_name = 'rm_safety'

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
    description='ROBOT. Safety layer. The sole publisher of /cmd_vel: clamping, lidar e-stop, kill switch, staleness watchdog.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # populated as nodes are written
        ],
    },
)
