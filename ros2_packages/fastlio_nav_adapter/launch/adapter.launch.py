from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("fastlio_nav_adapter"), "config", "adapter.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="fastlio_nav_adapter",
                executable="fastlio_nav_adapter_node",
                name="fastlio_nav_adapter",
                output="screen",
                parameters=[config],
            )
        ]
    )