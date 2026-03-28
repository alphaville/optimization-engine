from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="{{ros.package_name}}",
            executable="{{ros.node_name}}",
            name="{{ros.node_name}}",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("{{ros.package_name}}"),
                "config",
                "open_params.yaml",
            ])],
        )
    ])
