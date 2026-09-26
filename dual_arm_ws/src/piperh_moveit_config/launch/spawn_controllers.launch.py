from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    return generate_spawn_controllers_launch(
        MoveItConfigsBuilder("piperh", package_name="piperh_moveit_config").to_moveit_configs()
    )
