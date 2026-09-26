from glob import glob
import os

from setuptools import find_packages, setup

package_name = "rebot_xbox_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Guarded Xbox control and desktop selector for real robot arms.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "arm_selector = rebot_xbox_hardware.arm_selector:main",
            "active_arm_manager = rebot_xbox_hardware.active_arm_manager:main",
            "hardware_gripper = rebot_xbox_hardware.hardware_gripper:main",
            "instance_guard = rebot_xbox_hardware.instance_guard:main",
            "forbidden_zone_manager = rebot_xbox_hardware.forbidden_zone_manager:main",
        ],
    },
)
