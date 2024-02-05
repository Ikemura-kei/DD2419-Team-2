from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dd2419_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'world'), glob('./world/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*_launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('./rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='rosuser@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)