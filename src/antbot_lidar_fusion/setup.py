from glob import glob
import os
from setuptools import find_packages, setup

package_name = "antbot_lidar_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Fuse AntBot's two diagonal 2D lidars in base_link.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lidar_fusion_node = antbot_lidar_fusion.lidar_fusion_node:main",
            "topic_check_node = antbot_lidar_fusion.topic_check_node:main",
        ],
    },
)
