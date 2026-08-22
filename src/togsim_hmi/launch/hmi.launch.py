from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="8080"),
            Node(
                package="togsim_hmi",
                executable="hmi_server",
                name="hmi_bridge",
                output="screen",
                parameters=[{"port": LaunchConfiguration("port"), "use_sim_time": True}],
            ),
        ]
    )
