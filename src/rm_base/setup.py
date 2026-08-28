from setuptools import find_packages, setup

package_name = 'rm_base'

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
    description='ROBOT. Open-loop mecanum base driver: raw PWM, bypasses the firmware speed PID.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rm_base_node = rm_base.base_node:main',
        ],
    },
)
