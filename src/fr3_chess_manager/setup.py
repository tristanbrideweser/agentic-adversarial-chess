from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fr3_chess_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    maintainer_email='tristan@todo.todo',
    description='Game manager for the fr3-embodied-ai-chess project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_manager = fr3_chess_manager.game_manager:main',
            'move_planner = fr3_chess_manager.move_planner:main',
        ],
    },
)
