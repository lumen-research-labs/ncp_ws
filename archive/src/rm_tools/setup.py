from setuptools import find_packages, setup

package_name = 'rm_tools'

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
    description='WORKSTATION ONLY. Offline bag analysis, model export, schema checking. Never a runtime dependency of rm_safety or rm_policy.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # populated as nodes are written
        ],
    },
)
