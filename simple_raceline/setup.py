#!/usr/bin/env python3
# setup.py
"""Setup file for colcon."""

import os

from glob import glob

from setuptools import (
    find_packages,
    setup,
)

package_name = "simple_raceline"

setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [
                os.path.join("resource", package_name),
            ],
        ),
        (
            os.path.join("share", package_name),
            [
                "package.xml",
            ],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(
                os.path.join("launch", "*launch.[yxpr][amyv]*"),
            ),
        ),
    ],
    install_requires=[
        "setuptools",
        "scikit-image",
        "numpy",
    ],
    zip_safe=True,
    maintainer="Jaroslav Klapálek",
    maintainer_email="jaroslav.klapalek@tuwien.ac.at",
    description=(
        "A simple raceline generator used as a planner example for ARC course."
    ),
    license="GPLv3",
    entry_points={
        "console_scripts": ["simple_raceline = simple_raceline.simple_raceline:main"],
    },
)
