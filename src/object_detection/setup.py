from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team2',
    maintainer_email='team2@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = object_detection.object_detection:main',
            'display_markers = object_detection.display_markers:main',
            'dl_detection = object_detection.dl_detection:main',
            'dl_object_classifier = object_detection.dl_object_classifier:main',
            'dl_postprocessor = object_detection.dl_postprocessor:main',
            'dl_object_classifier_on_arm_camera = object_detection.dl_object_classifier_on_arm_camera:main',
            'detection_by_threshold_node = object_detection.detection_by_threshold_node:main',
        ],
    },
)
