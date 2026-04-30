from setuptools import setup, find_packages
import os
from glob import glob

package_name = "grasp_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tristan",
    maintainer_email="tristan@example.com",
    description=(
        "Piece-type-aware grasp planner for chess robot arms. "
        "Supports GPD (live point cloud) and precomputed lookup-table modes."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_planner_node = grasp_planner.grasp_planner_node:main",
        ],
    },
)