import os
from glob import glob

from setuptools import setup

package_name = "chess_engine"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Karthik Sivachandran",
    maintainer_email="sivachandrankarthik@gmail.com",
    description="Chess engine package: authoritative board state and per-color Stockfish service.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "board_state_node = chess_engine.board_state_node:main",
            "stockfish_node = chess_engine.stockfish_node:main",
        ],
    },
)
