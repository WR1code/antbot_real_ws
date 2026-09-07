from setuptools import find_packages, setup


package_name = "antbot_h743_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="WR",
    maintainer_email="wr@example.invalid",
    description="Safe UART bridge and tools for the AntBot H743 chassis.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "h743_cmd_vel_bridge = antbot_h743_bridge.bridge:main",
            "h743_control = antbot_h743_bridge.control_tool:main",
            "h743_dashboard = antbot_h743_bridge.chassis_dashboard:main",
            "h743_uart_debug = antbot_h743_bridge.uart_debug_tool:main",
        ],
    },
)
