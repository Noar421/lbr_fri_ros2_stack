import os

from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("iiwa14_2f85", package_name="iiwa14_2f85_moveit_config")
        .robot_description(
            os.path.join(
                get_package_share_directory("lbr_description"),
                "urdf/iiwa14_2f85/iiwa14_2f85.xacro",
            )
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
