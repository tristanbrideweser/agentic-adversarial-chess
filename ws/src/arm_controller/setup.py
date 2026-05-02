from setuptools import setup, find_packages
import os
from glob import glob

package_name = "arm_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join(os.path.dirname(__file__), "config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tristan",
    maintainer_email="tristan@example.com",
    description=(
        "Pick-and-place action servers for both Franka Panda arms. "
        "Orchestrates MoveIt 2 motion planning, Cartesian path execution, "
        "and gripper control for chess piece manipulation."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_place_server_white = arm_controller.pick_place_server:main_white",
            "pick_place_server_black = arm_controller.pick_place_server:main_black",
        ],
    },
)