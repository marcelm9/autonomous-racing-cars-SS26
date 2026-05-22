from setuptools import setup, find_packages
from glob import glob
import os

package_name = "launch_orchestrator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # old path copy i dont know if we still need this -matthias
        # (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        # (os.path.join('share', 'launch_orchestrator', 'diagnostics', 'config'), glob('config/*.yaml')),
        # Include all launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # Include all config files (like your yaml)
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        # Include all map files
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="matthias",
    maintainer_email="matthias@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
)
