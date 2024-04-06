from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/mapping.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='ikemurakei2001@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'naive_mapping_node = slam.naive_mapping_node:main',
            'ekf_slam_node = slam.ekf_slam_node:main',
            'tune_odom_node = slam.tune_odom_node:main',
            'ekf_slam_v2_node = slam.ekf_slam_v2_node:main',
        ],
    },
)
