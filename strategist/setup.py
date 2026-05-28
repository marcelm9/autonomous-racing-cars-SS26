from setuptools import find_packages, setup

package_name = "strategist"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # CRITICAL FOR ROS2: This line registers the package with the ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="marcel.menzel@student.tuwien.ac.at",
    description="Publishes racing strategies and RViz markers",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "strategist = strategist.strategist:main",
        ],
    },
)
