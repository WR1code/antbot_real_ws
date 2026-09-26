from glob import glob
import os

from setuptools import find_packages, setup


package_name = "antbot_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Preflight-gated MID360S mapping integration for AntBot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lio_odom_adapter = antbot_mapping.lio_odom_adapter:main",
            "mapping_preflight = antbot_mapping.mapping_preflight:main",
        ],
    },
)
