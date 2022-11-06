from glob import glob
import os
from setuptools import setup

package_name = 'ros2_berrygps'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(share_dir, 'launch'), glob('launch/*.launch.py')),
        (os.path.join(share_dir, 'config'), glob('config/*.yaml')),
        (share_dir, glob('config/*.ini')),
    ],
    install_requires=['setuptools',
                      'pyserial',
                      'numpy',
                      'pyyaml'],
    zip_safe=True,
    author='Matt Murray',
    maintainer='Matt Murray',
    keywords=['ROS2'],
    maintainer_email='mattanimation@gmail.com',
    description='A package to publish GPS and IMU message from the BerryGPS Hat.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'berry_gps = ros2_berrygps.berrygps_node:main',
            'imu_node = ros2_berrygps.imu_node:main',
            'gps_node = ros2_berrygps.gps_node:main'
        ],
    },
)
