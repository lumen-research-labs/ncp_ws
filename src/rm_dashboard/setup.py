from setuptools import find_packages, setup

package_name = 'rm_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # The web assets are packaged INSIDE the python package (not into
    # share/) and read relative to __file__, so the node works identically
    # from a colcon install, a --symlink-install tree, and a plain
    # `python3 -m rm_dashboard.dashboard_node` run on the robot. Serving from
    # share/ would have made the last of those fail, which is exactly the
    # situation you are in when debugging a robot that will not build.
    package_data={package_name: ['static/*.html']},
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dev Patel',
    maintainer_email='devsp2003@gmail.com',
    description=(
        'ROBOT. Live web dashboard for the NCP pipeline. Subscribe-only on '
        'topics; never publishes. Stdlib HTTP + SSE, no third-party deps.'
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = rm_dashboard.dashboard_node:main',
        ],
    },
)
