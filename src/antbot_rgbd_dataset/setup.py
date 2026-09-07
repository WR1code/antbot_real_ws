from glob import glob
import os

from setuptools import find_packages, setup


package_name = "antbot_rgbd_dataset"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Exact-stamp RGB-D keyframe dataset recorder for AntBot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rgbd_keyframe_recorder = antbot_rgbd_dataset.recorder_node:main",
            "phase4b_capture_node = antbot_rgbd_dataset.phase4b_capture_node:main",
            "offline_preview_publisher = "
            "antbot_rgbd_dataset.offline_preview_node:main",
            "rebuild_rgbd_preview = antbot_rgbd_dataset.rebuild_preview:main",
        ],
    },
)
