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
            launch_ros.actions.Node(
                # https://github.com/ros2/teleop_twist_joy
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[params],
                output="screen",
                remappings=[
                    ("cmd_vel", "cmd_vel_teleop"),
                ],
            ),
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
