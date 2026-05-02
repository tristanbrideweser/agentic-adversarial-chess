from setuptools import setup, find_packages
import os
from glob import glob

package_name = "board_localization"

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
        "Board coordinate system, TF tree broadcaster, and square lookup "
        "service for the chess robot system."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "board_tf_broadcaster  = board_localization.tf_broadcaster:main",
            "square_lookup_service = board_localization.square_lookup_service:main",
        ],
    },
)