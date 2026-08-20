"""Controller spawners + vacuum bridge (used by sim.launch.py; kept separate for reuse)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="togsim_control",
                executable="vacuum_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
