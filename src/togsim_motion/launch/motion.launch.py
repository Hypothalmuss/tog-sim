from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare("togsim_motion"), "config", "motion.yaml"])
    return LaunchDescription(
        [
            Node(
                package="togsim_motion",
                executable="motion_server",
                parameters=[cfg, {"use_sim_time": True}],
                output="screen",
            )
        ]
    )
