import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'chess_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=['chess_robot_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandov22',
    maintainer_email='sandov22@todo.todo',
    description='Top-level launch and config for the chess robot stack.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'publish_finger_states = chess_robot_bringup.publish_finger_states:main',
        ],
    },
)
