from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=TextSubstitution(text="vins_multi"),
        description="Node namespace (prefix for topics)."
    )

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

    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value=TextSubstitution(text=""),
        description="Optional launch prefix (e.g. 'gdb -ex run --args')."
    )

    node = Node(
        package="vins_estimator_ros2",
        executable="vins_estimator_ros2_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        prefix=LaunchConfiguration("prefix"),
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
        }]
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        prefix_arg,
        node,
    ])
