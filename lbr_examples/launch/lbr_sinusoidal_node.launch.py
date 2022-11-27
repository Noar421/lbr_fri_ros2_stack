import math

from launch.launch_description import DeclareLaunchArgument, LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch_arg = DeclareLaunchArgument(
        name="sim",
        default_value="true",
        description="Whether to launch in simulation."
    )

    command_rate_launch_arg = DeclareLaunchArgument(
        name="command_rate",
        default_value=f"{100}",
        description="Command rate in Hz."
    )

    amplitude_launch_arg = DeclareLaunchArgument(
        name="amplitude",
        default_value=f"{math.pi/4.}",
        description="Rotation amplitude of joint 6 in radians."
    )

    period_launch_arg = DeclareLaunchArgument(
        name="period",
        default_value=f"{10.}",
        description="Rotation period in seconds."
    )

    sinusoidal_node = Node(
        package="lbr_examples",
        executable="lbr_sinusoidal_node.py",
        parameters=[
            {"sim": LaunchConfiguration("sim")},
            {"command_rate": LaunchConfiguration("command_rate")},
            {"amplitude": LaunchConfiguration("amplitude")},
            {"period": LaunchConfiguration("period")}
        ]
    )

    return LaunchDescription(
        [
            sim_launch_arg,
            command_rate_launch_arg,
            amplitude_launch_arg,
            period_launch_arg,
            sinusoidal_node
        ]
    )
