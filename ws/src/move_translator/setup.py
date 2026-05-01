from setuptools import setup, find_packages
import os
from glob import glob

package_name = "move_translator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "chess",
    ],
    zip_safe=True,
    maintainer="Tristan",
    maintainer_email="tristan@example.com",
    description=(
        "Translates FEN + UCI chess moves into world-frame pick-and-place "
        "task queues for the Franka Panda arms."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move_translator_node = move_translator.move_translator_node:main",
        ],
    },
)