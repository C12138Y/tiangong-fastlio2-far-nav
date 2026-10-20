from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("fastlio_nav_adapter"),
        "config",
        "local_costmap.yaml",
    ])

    return LaunchDescription([
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            namespace="my_nav",
            name="local_costmap",
            output="screen",
            parameters=[config_file],
        ),
    ])
    