from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandov22',
    maintainer_email='sandov22@todo.todo',
    description='Board state and piece perception for the chess robots.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_verifier = perception.board_verifier:main',
        ],
    },
)
