from setuptools import find_packages, setup

package_name = 'decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team2',
    maintainer_email='mael.arrive@insa-lyon.fr',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ms1_go_to_obj_node = decision.ms1_go_to_obj_node:main',
            'ms1_go_to_obj_node_v2 = decision.ms1_go_to_obj_node_v2:main',
            'ms1_pick_and_place_node = decision.ms1_pick_and_place_node:main'
        ],
    },
)
