from setuptools import find_packages, setup

package_name = 'chess_engine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tristan',
    entry_points={
        'console_scripts': [
            'board_state_node = chess_engine.board_state_node:main',
            'stockfish_node = chess_engine.stockfish_node:main', # Added this entry
        ],
    },
)