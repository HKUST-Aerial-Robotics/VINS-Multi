from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("vins_estimator_ros2"),
            "config",
            "multi_rs_color",
            "multi_l515_d435_color.yaml",
        ]),
        description="Path to the VINS configuration YAML file."
    )

    node = Node(
        package="vins_estimator_ros2",
        executable="vins_estimator_ros2_node",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
        }]
    )

    return LaunchDescription([
        config_arg,
        node,
    ])
