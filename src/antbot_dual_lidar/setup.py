from glob import glob
import os

from setuptools import find_packages, setup


package_name = "antbot_dual_lidar"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"] + glob("*.md")),
        ("share/" + package_name, glob("*.csv")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "rviz"), glob("config/rviz/*.rviz")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
        (os.path.join("share", package_name, "datasets"), glob("datasets/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="w",
    maintainer_email="w@example.com",
    description="Independent preprocessing and diagnostics for AntBot's dual RTX 3D lidar.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cloud_preprocessor = antbot_dual_lidar.cloud_preprocessor:main",
            "livox_frame_relay = antbot_dual_lidar.livox_frame_relay:main",
            "dual_lidar_diagnostics = antbot_dual_lidar.diagnostics:main",
            "dual_cloud_synchronizer = antbot_dual_lidar.synchronizer:main",
            "performance_probe = antbot_dual_lidar.performance_probe:main",
            "coverage_probe = antbot_dual_lidar.coverage_probe:main",
            "ground_truth_deskew_validator = antbot_dual_lidar.ground_truth_deskew_validator:main",
            "validate_lio_dataset = antbot_dual_lidar.dataset_validation:main",
            "phase2a_motion_experiment = antbot_dual_lidar.phase2a_motion_experiment:main",
            "phase2a_analyze_bag = antbot_dual_lidar.phase2a_bag_analysis:main",
            "phase2a_minimal_manifest = antbot_dual_lidar.phase2a_minimal_manifest:main",
            "fixed_frame_mapper = antbot_dual_lidar.fixed_frame_mapper:main",
            "save_pointcloud = antbot_dual_lidar.save_pointcloud:main",
            "offline_pointcloud_publisher = antbot_dual_lidar.offline_pointcloud_publisher:main",
            "create_map_bundle = antbot_dual_lidar.create_map_bundle:main",
            "dynamic_obstacle_monitor = antbot_dual_lidar.dynamic_obstacle_monitor:main",
        ],
    },
)
