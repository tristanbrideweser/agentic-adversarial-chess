from setuptools import find_packages, setup

package_name = 'fr3_chess_logic'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-chess'],
    zip_safe=True,
    maintainer='Karthik Sivachandran',
    maintainer_email='ksivacha@purdue.edu',
    description='Chess logic and validation package for the fr3-embodied-ai-chess project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_state_node = fr3_chess_logic.board_state_node:main',
            'stockfish_node = fr3_chess_logic.stockfish_node:main',
        ],
    },
)
