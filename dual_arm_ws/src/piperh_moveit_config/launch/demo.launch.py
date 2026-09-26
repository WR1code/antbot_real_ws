"""Start Piper-H MoveIt with simulation-only mock ros2_control."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("piperh", package_name="piperh_moveit_config")
        .robot_description(file_path="config/piperh.urdf.xacro")
        .robot_description_semantic(file_path="config/piperh.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    return generate_demo_launch(config)
