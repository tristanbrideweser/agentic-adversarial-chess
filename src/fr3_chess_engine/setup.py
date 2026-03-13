from setuptools import find_packages, setup

package_name = 'fr3_chess_engine'

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
    description='Engine interface for the fr3-embodied-ai-chess project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'engine_node = fr3_chess_engine.engine_node:main',
        ],
    },
)
