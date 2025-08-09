import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = get_package_share_directory("isa_afa_3")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_spawn_model_launch_source = os.path.join(
        ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"
    )
    default_model_path = os.path.join(
        pkg_share, "src", "description", "robot.xacro"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
    world_path = os.path.join(pkg_share, "worlds", "streets_with_walls_world.sdf")
    bridge_config_path = os.path.join(pkg_share, "config", "bridge_config.yaml")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    gz_server = GzServer(
        world_sdf_file=world_path,
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file=bridge_config_path,
        extra_bridge_params=[{}],
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            "world": "steets_with_walls",
            "topic": "/robot_description",
            "entity_name": "robot",
            "z": "10.0",
        }.items(),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot model file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen"),
            robot_state_publisher_node,
            robot_localization_node,
            rviz_node,
            gz_server,
            ros_gz_bridge,
            spawn_entity,
        ]
    )
