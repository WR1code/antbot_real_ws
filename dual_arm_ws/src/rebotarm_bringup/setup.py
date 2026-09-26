from glob import glob
from setuptools import setup

package_name = "rebotarm_bringup"
config_files = [
    "config/driver_params.yaml",
    "config/rebotarm_hardware.yaml",
]

setup(
    name=package_name,
    version="0.3.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", config_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Hardware launch and configuration files for reBotArm.",
    license="Apache-2.0",
)
