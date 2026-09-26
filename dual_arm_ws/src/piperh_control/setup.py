from glob import glob
from setuptools import find_packages, setup


package_name = "piperh_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "joint_state_udp_bridge = piperh_control.joint_state_udp_bridge:main",
            "hardware_adapter = piperh_control.hardware_adapter:main",
            "gravity_audit = piperh_control.gravity_audit:main",
            "gravity_compensation_dry_run = piperh_control.gravity_dry_run:main",
        ]
    },
)
