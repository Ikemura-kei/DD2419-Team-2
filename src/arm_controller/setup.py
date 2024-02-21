from setuptools import find_packages, setup

package_name = 'arm_controller'

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
    maintainer_email='ikemurakei2001@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_controller = arm_controller.open_loop_controller:main',
            'joystick_controller = arm_controller.joystick_controller:main',
            'forward_kinematics = arm_controller.forward_kinematics:main',
            'rrr = arm_controller.rrr:main'
        ],
    },
)
