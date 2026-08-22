from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("togsim_motion")
    cfg = PathJoinSubstitution([share, "config", "motion.yaml"])
    # profile_<name>.yaml overrides the acceleration / jerk limits: "smooth" (demos) or "fast" (benchmarks)
    profile = PathJoinSubstitution([share, "config", ["profile_", LaunchConfiguration("profile"), ".yaml"]])
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="smooth", description="motion profile: smooth | fast"),
            Node(
                package="togsim_motion",
                executable="motion_server",
                parameters=[cfg, profile, {"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
