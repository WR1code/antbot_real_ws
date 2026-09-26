from glob import glob
import os

from setuptools import find_packages, setup


package_name = "meridian_hand_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    scripts=["scripts/run_hand_depth_viewer.sh"],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description=(
        "Display MediaPipe hand landmarks, registered Orbbec depth, "
        "and a wrist-guided arm contour."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hand_depth_viewer = meridian_hand_vision.hand_depth_viewer:main",
        ],
    },
)
