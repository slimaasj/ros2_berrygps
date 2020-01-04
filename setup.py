
from glob import glob
import os
from setuptools import setup

package_name = 'ros2_berrygps'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('config/*.ini')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matt Murray',
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
