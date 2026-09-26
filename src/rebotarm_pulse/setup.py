from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rebotarm_pulse"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Safe vision-guided pre-contact positioning for pulse demos.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "apriltag_board_pose = rebotarm_pulse.apriltag_board_pose:main",
            "charuco_board_pose = rebotarm_pulse.charuco_board_pose:main",
            "auto_handeye_sequence = rebotarm_pulse.auto_handeye_sequence:main",
            "joint_target_move = rebotarm_pulse.joint_target_move:main",
            "pulse_approach = rebotarm_pulse.pulse_approach:main",
            "pressure_serial_bridge = rebotarm_pulse.pressure_serial_bridge:main",
            "pulse_web_gateway = rebotarm_pulse.pulse_web_gateway:main",
            "piper_pulse_target = rebotarm_pulse.piper_pulse_target:main",
            "piper_pulse_align = rebotarm_pulse.piper_pulse_align:main",
            "piper_tool_geometry = rebotarm_pulse.piper_tool_geometry:main",
        ],
    },
)
