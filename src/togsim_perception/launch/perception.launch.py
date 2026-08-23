import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source = LaunchConfiguration("source")
    weights = LaunchConfiguration("weights")
    sim = {"use_sim_time": True}
    return LaunchDescription(
        [
            DeclareLaunchArgument("source", default_value="yolo", description="yolo | gt"),
            DeclareLaunchArgument("weights", default_value="~/togsim_data/weights/togsim_seg.pt"),
            DeclareLaunchArgument(
                "tray_models",
                default_value="",
                description="csv of tray models in the cell, e.g. tray_2x4,tray_bar_2x3",
            ),
            Node(
                package="togsim_perception",
                executable="segmentation_node",
                output="screen",
                parameters=[sim, {"source": source, "weights": weights}],
            ),
            Node(package="togsim_perception", executable="pick_pose_node", output="screen", parameters=[sim]),
            Node(
                package="togsim_perception",
                executable="tray_vacancy_node",
                output="screen",
                parameters=[
                    os.path.join(get_package_share_directory("togsim_perception"), "config", "perception_tuning.yaml"),
                    sim,
                    {"tray_models_csv": LaunchConfiguration("tray_models")},
                ],
            ),
        ]
    )
