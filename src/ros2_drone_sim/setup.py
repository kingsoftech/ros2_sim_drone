from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_drone_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name,'models'), glob('models/*')),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdullah',
    maintainer_email='asoyero@student.lautech.edu.ng',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'drone_info_node = ros2_drone_sim.drone_info:main',
        ],
    },
)
