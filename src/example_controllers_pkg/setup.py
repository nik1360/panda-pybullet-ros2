
import os
from glob import glob 
from setuptools import setup

package_name = 'example_controllers_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma][xml]*'))) 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikolas',
    maintainer_email='nikolas.sacchi01@universitadipavia.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'torque_control_node = example_controllers_pkg.torque_control_node:main',
            'external_force_node = example_controllers_pkg.external_force_node:main',
            'cartesian_velocity_controller_node = example_controllers_pkg.cartesian_velocity_controller_node:main'
        ],
    },
)
