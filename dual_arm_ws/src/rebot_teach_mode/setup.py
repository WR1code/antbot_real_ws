from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rebot_teach_mode"

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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "action_groups"),
            glob("action_groups/*.md"),
        ),
        (
            os.path.join("share", package_name, "action_groups", "dm"),
            glob("action_groups/dm/*.md"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Guarded gravity-compensation teaching and replay for reBotArm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teach_mode_node = rebot_teach_mode.teach_node:main",
            "teach_xbox_bridge = rebot_teach_mode.xbox_bridge:main",
        ],
    },
)
