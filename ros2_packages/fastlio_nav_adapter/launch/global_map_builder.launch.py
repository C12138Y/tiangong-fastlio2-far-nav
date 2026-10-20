from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("fastlio_nav_adapter"),
        "config",
        "global_map_builder.yaml",
    ])

    return LaunchDescription([
        Node(
            package="fastlio_nav_adapter",
            executable="global_map_builder_node",
            namespace="my_nav",
            name="global_map_builder",
            output="screen",
            parameters=[config_file],
        ),
    ])