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

"""
What was missing in the tutorial https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html:
- from ament_index_python.packages import get_package_share_directory
- ExecuteProcess, IncludeLaunchDescription after from launch.actions import DeclareLaunchArgument
- from launch.launch_description_sources import PythonLaunchDescriptionSource
- from ros_gz_bridge.actions import RosGzBridge
- from ros_gz_sim.actions import GzServer
- ...get_package_share_directory('sam_bot_description') instead of ...FindPackageShare(package='sam_bot_description')...
- ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
- gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
- - ros_topic_name: "/tf"
  gz_topic_name: "/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
- z = 0.65
"""


def generate_launch_description():
    pkg_share = get_package_share_directory("sam_bot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_spawn_model_launch_source = os.path.join(
        ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"
    )
    default_model_path = os.path.join(
        pkg_share, "src", "description", "sam_bot_description.sdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
    world_path = os.path.join(pkg_share, "world", "my_world.sdf")
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
        container_name="ros_gz_container",
        create_own_container="True",
        use_composition="True",
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file=bridge_config_path,
        container_name="ros_gz_container",
        create_own_container="False",
        use_composition="True",
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            "world": "my_world",
            "topic": "/robot_description",
            "entity_name": "sam_bot",
            "z": "0.65",
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
