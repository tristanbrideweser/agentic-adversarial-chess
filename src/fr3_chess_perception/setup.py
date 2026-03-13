from setuptools import find_packages, setup

package_name = 'fr3_chess_perception'

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
    maintainer='tristan',
    maintainer_email='tristan@todo.todo',
    description='Perception package for the fr3-embodied-ai-chess project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_detector = fr3_chess_perception.board_detector:main',
            'piece_classifier = fr3_chess_perception.piece_classifier:main',
        ],
    },
)
