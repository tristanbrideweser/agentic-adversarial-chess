from setuptools import setup

package_name = "game_coordinator"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Karthik Sivachandran",
    maintainer_email="sivachandrankarthik@gmail.com",
    description="Top-level py_trees behavior tree orchestrating the chess game loop.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "coordinator_node = game_coordinator.coordinator_node:main",
        ],
    },
)
