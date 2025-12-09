# Created by Nelson Durrant, Jun 2025
# AUTONOMY MISSION ROVER LAUNCH FILE
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")
    sim_mode = LaunchConfiguration("sim_mode")
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RVIZ"
    )
    declare_use_mapviz_cmd = DeclareLaunchArgument(
        "use_mapviz", default_value="False", description="Whether to start mapviz"
    )
    declare_sim_mode_cmd = DeclareLaunchArgument(
        "sim_mode",
        default_value="False",
        description="Whether to start in simulation mode",
    )

    bringup_dir = get_package_share_directory("nav2_bringup")
    nav_dir = get_package_share_directory("rover_navigation")
    gui_dir = get_package_share_directory("rover_gui")
    loc_dir = get_package_share_directory("rover_localization")
    per_dir = get_package_share_directory("rover_perception")
    gz_dir = get_package_share_directory("rover_gazebo")
    des_dir = get_package_share_directory("rover_description")
    ublox_dir = get_package_share_directory("ublox_read_2")

    nav_launch_dir = os.path.join(nav_dir, "launch")
    loc_launch_dir = os.path.join(loc_dir, "launch")
    gui_launch_dir = os.path.join(gui_dir, "launch")
    per_launch_dir = os.path.join(per_dir, "launch")
    gz_launch_dir = os.path.join(gz_dir, "launch")
    des_launch_dir = os.path.join(des_dir, "launch")
    ublox_launch_dir = os.path.join(ublox_dir, "launch")

    nav_params_dir = os.path.join(nav_dir, "config")
    nav2_params = os.path.join(nav_params_dir, "navigation_params.yaml")
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(des_launch_dir, "rover_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    gazebo_cmd = IncludeLaunchDescription(
        # This only launches in simulation
        PythonLaunchDescriptionSource(
            os.path.join(gz_launch_dir, "rover_gazebo.launch.py")
        ),
        condition=IfCondition(sim_mode),
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_launch_dir, "rover_localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
            "params_file": configured_nav2_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "rviz.launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "mapviz.launch.py")),
        condition=IfCondition(use_mapviz),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    gui_cmd = IncludeLaunchDescription(
        # This only launches in simulation
        PythonLaunchDescriptionSource(
            os.path.join(gui_launch_dir, "autonomy_gui.launch.py")
        ),
        condition=IfCondition(sim_mode),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    aruco_opencv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(per_launch_dir, "rover_perception.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    ublox_cmd = IncludeLaunchDescription(
        # This only launches in real life
        XMLLaunchDescriptionSource(os.path.join(ublox_launch_dir, "rover_launch.xml")),
        condition=UnlessCondition(sim_mode),
    )

    state_machine_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, "rover_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_mode_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)

    # Only launched in simulation
    ld.add_action(gazebo_cmd)
    ld.add_action(gui_cmd)

    # Only launched in real life
    # ld.add_action(gps_cmd) # We launch the GPS individually right now

    # Both sim and real
    ld.add_action(description_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(aruco_opencv_cmd)
    ld.add_action(state_machine_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(mapviz_cmd)

    return ld
