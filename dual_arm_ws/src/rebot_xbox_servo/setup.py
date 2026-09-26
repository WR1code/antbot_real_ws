from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rebot_xbox_servo"


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
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Shared safety-latched Xbox MoveIt Servo input for reBotArm.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "rebot_xbox_twist = rebot_xbox_servo.xbox_twist:main",
            "inspect_joy = rebot_xbox_servo.inspect_joy:main",
            "sim_gripper = rebot_xbox_servo.sim_gripper:main",
            "arm_initializer = rebot_xbox_servo.arm_initializer:main",
        ],
    },
)
