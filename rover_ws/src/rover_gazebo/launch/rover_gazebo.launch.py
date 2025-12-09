# Created by Nelson Durrant, Feb 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    gz_dir = get_package_share_directory("rover_gazebo")
    gz_launch_dir = os.path.join(gz_dir, "launch")

    world = os.path.join(gz_dir, "worlds", "provo_canyon.world")
    models_dir = os.path.join(gz_dir, "models")
    # Don't add trailing separator - it can cause path resolution issues
    set_gz_model_path_cmd = None

    if "GAZEBO_MODEL_PATH" in os.environ:
        gz_model_path = os.environ["GAZEBO_MODEL_PATH"] + os.pathsep + models_dir.rstrip(os.pathsep)
        set_gz_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gz_model_path
        )
    else:
        set_gz_model_path_cmd = SetEnvironmentVariable("GAZEBO_MODEL_PATH", models_dir.rstrip(os.pathsep))

    # Spawn the rover in the simulation
    # Use TimerAction to delay spawn until Gazebo server is ready
    spawn_rover_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "rover",
            "-x", "0.0",        # Change from -54.5 to start at center
            "-y", "0.0",        # Change from 127.8 to start at center
            "-z", "0.0",        # Spawn slightly above terrain
            "-Y", "0.0",
        ],
    )
    
    # Delay rover spawn to wait for Gazebo server to be ready
    spawn_rover_delayed = TimerAction(
        period=5.0,  # Wait 5 seconds for server to initialize
        actions=[spawn_rover_cmd]
    )

    # Start the Gazebo server
    # Explicitly pass GAZEBO_MODEL_PATH to ensure it's available
    gz_env = os.environ.copy()
    if "GAZEBO_MODEL_PATH" in gz_env:
        gz_env["GAZEBO_MODEL_PATH"] = gz_env["GAZEBO_MODEL_PATH"] + os.pathsep + models_dir.rstrip(os.pathsep)
    else:
        gz_env["GAZEBO_MODEL_PATH"] = models_dir.rstrip(os.pathsep)
    
    start_gz_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[gz_launch_dir],
        output="both",
        env=gz_env,
    )

    # Start the Gazebo client
    # Delay client start to wait for server to be ready
    # Pass same environment variables
    start_gz_client_cmd = ExecuteProcess(
        cmd=["gzclient"], cwd=[gz_launch_dir], output="both", env=gz_env
    )
    
    # Delay Gazebo client start to wait for server
    start_gz_client_delayed = TimerAction(
        period=2.0,  # Wait 2 seconds for server to initialize
        actions=[start_gz_client_cmd]
    )

    ld = LaunchDescription(
        [
            Node(
                package="rover_gazebo",
                executable="sim_obj_detect",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "enable_mallet": True,
                        "enable_bottle": False,
                    }
                ],
            ),
            # Remap the point cloud to what we'd see in real life
            Node(
                package="topic_tools",
                executable="relay",
                name="gz_points_relay",
                output="screen",
                arguments=[
                    "/intel_realsense_r200_depth/points",
                    "/zed/zed_node/point_cloud/cloud_registered",
                ],
            ),
            # Remap the camera feed to what we'd see in real life (for experimental MCP server)
            Node(
                package="topic_tools",
                executable="relay",
                name="gz_image_relay",
                output="screen",
                arguments=[
                    "/intel_realsense_r200_depth/image_raw",
                    "/zed/zed_node/rgb/image_rect_color",
                ],
            ),
        ]
    )

    ld.add_action(set_gz_model_path_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_gz_client_delayed)  # Use delayed client start
    ld.add_action(spawn_rover_delayed)  # Use delayed rover spawn

    return ld
