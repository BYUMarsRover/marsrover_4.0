# Created by Nelson Durrant, Feb 2025
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    control_dir = get_package_share_directory("rover_control")
    params = os.path.join(control_dir, "config/control_params.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                # https://docs.ros.org/en/humble/p/joy/
                package="joy",
                executable="joy_node",
                name="joy_node_rover",
                output="screen",
            ),
            # Note: teleop_twist_joy runs on base station, not rover
            # The rover only needs joy_node for local elevator control
            launch_ros.actions.Node(
                package="rover_control",
                executable="drive_mux",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rover_control",
                executable="mega_wrapper",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rover_control",
                executable="nano_wrapper",
                output="screen",
            ),
        ]
    )
