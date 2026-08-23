"""Everything needed for a running cell: Gazebo cell + controllers + vacuum bridge + motion server.

Pass-through args: gui, products, infeed_speed, outfeed_speed, enclosure, rtf, cameras, cam_width, cam_height, cam_rate
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PASS = [
    "motion_profile",
    "spawn_seed",
    "product_rate",
    "product_classes",
    "tray_models",
    "tray_pitch",
    "max_products",
    "segmentation",
    "gui",
    "products",
    "infeed_speed",
    "outfeed_speed",
    "enclosure",
    "rtf",
    "cameras",
    "cam_width",
    "cam_height",
    "cam_rate",
    "conveyor_visuals",
]
DEFAULTS = {
    "motion_profile": "smooth",
    "spawn_seed": "0",
    "segmentation": "false",
    "product_rate": "24.0",
    "product_classes": "product_bar,product_carton",
    "tray_models": "",
    "tray_pitch": "0.55",
    "max_products": "12",
    "gui": "true",
    "products": "true",
    "infeed_speed": "0.10",
    "outfeed_speed": "0.08",
    "enclosure": "true",
    "rtf": "1.0",
    "cameras": "true",
    "cam_width": "848",
    "cam_height": "480",
    "cam_rate": "30",
    "conveyor_visuals": "true",
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in DEFAULTS.items()]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("togsim_gazebo"), "launch", "sim.launch.py"])
        ),
        launch_arguments={k: LaunchConfiguration(k) for k in PASS}.items(),
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("togsim_control"), "launch", "control.launch.py"])
        )
    )
    motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("togsim_motion"), "launch", "motion.launch.py"])
        ),
        launch_arguments={"profile": LaunchConfiguration("motion_profile")}.items(),
    )
    return LaunchDescription([*args, sim, control, motion])
