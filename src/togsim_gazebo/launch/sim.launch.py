"""Bring up the tog-sim cell in Gazebo Fortress: world, robot, ros2_control, bridges, spawner.

Arguments:
  gui:=true|false          Gazebo GUI client (server always runs)
  headless:=true|false     shortcut for gui:=false plus no rendering-heavy extras
  products:=true|false     spawn products/trays automatically
  infeed_speed, outfeed_speed (m/s), rtf (0 = unthrottled), enclosure:=true|false
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _expand_world(context, *args, **kwargs):
    share = get_package_share_directory("togsim_gazebo")
    world_xacro = os.path.join(share, "worlds", "cell.sdf.xacro")
    out = os.path.join(tempfile.gettempdir(), "togsim_cell.sdf")
    cmd = [
        "xacro",
        world_xacro,
        f"infeed_speed:={LaunchConfiguration('infeed_speed').perform(context)}",
        f"outfeed_speed:={LaunchConfiguration('outfeed_speed').perform(context)}",
        f"enclosure:={LaunchConfiguration('enclosure').perform(context)}",
        f"rtf:={LaunchConfiguration('rtf').perform(context)}",
        f"cameras:={LaunchConfiguration('cameras').perform(context)}",
        f"conveyor_visuals:={LaunchConfiguration('conveyor_visuals').perform(context)}",
        f"cam_width:={LaunchConfiguration('cam_width').perform(context)}",
        f"cam_height:={LaunchConfiguration('cam_height').perform(context)}",
        f"cam_rate:={LaunchConfiguration('cam_rate').perform(context)}",
    ]
    with open(out, "w") as f:
        subprocess.run(cmd, check=True, stdout=f)
    gui = LaunchConfiguration("gui").perform(context) == "true"
    gz_args = f"-r -v 2 {'' if gui else '-s '}{out}"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        )
    ]


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " '",
                PathJoinSubstitution([FindPackageShare("togsim_description"), "urdf", "togsim_robot.urdf.xacro"]),
                "' sim:=true",
            ]
        ),
        value_type=str,
    )
    bridge_cfg = PathJoinSubstitution([FindPackageShare("togsim_gazebo"), "config", "bridge.yaml"])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}, use_sim_time],
        output="screen",
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "togsim_robot", "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_cfg}, use_sim_time],
        output="screen",
    )
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cam_pick/image", "/cam_pick/depth_image", "/cam_place/image", "/cam_place/depth_image"],
        parameters=[use_sim_time],
        output="screen",
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_pos = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_traj = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager", "--inactive"],
        output="screen",
    )
    spawner = Node(
        package="togsim_gazebo",
        executable="product_spawner",
        parameters=[
            use_sim_time,
            {
                "products.enabled": LaunchConfiguration("products"),
                "trays.enabled": LaunchConfiguration("products"),
                "belts.infeed_speed": LaunchConfiguration("infeed_speed"),
                "belts.outfeed_speed": LaunchConfiguration("outfeed_speed"),
                "products.rate_per_min": LaunchConfiguration("product_rate"),
                "products.max_alive": LaunchConfiguration("max_products"),
            },
        ],
        output="screen",
    )
    # camera frames: world -> <cam>_optical (z forward, x right, y down). The sensor in the SDF is at
    # (0.35, ±0.35, 1.62) pitched +90 deg; its optical frame is that pose rotated by (-90, 0, -90) deg.
    cam_tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.35",
                "--y",
                y,
                "--z",
                "1.62",
                "--roll",
                "0",
                "--pitch",
                "1.5708",
                "--yaw",
                "0",
                "--frame-id",
                "world",
                "--child-frame-id",
                f"{name}_link",
            ],
            parameters=[use_sim_time],
        )
        for name, y in (("cam_pick", "-0.35"), ("cam_place", "0.35"))
    ] + [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "-1.5708",
                "--pitch",
                "0",
                "--yaw",
                "-1.5708",
                "--frame-id",
                f"{name}_link",
                "--child-frame-id",
                f"{name}_optical",
            ],
            parameters=[use_sim_time],
        )
        for name in ("cam_pick", "cam_place")
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("products", default_value="true"),
            DeclareLaunchArgument("infeed_speed", default_value="0.10"),
            DeclareLaunchArgument("outfeed_speed", default_value="0.08"),
            DeclareLaunchArgument("enclosure", default_value="true"),
            DeclareLaunchArgument("rtf", default_value="1.0"),
            DeclareLaunchArgument("cameras", default_value="true"),
            DeclareLaunchArgument("conveyor_visuals", default_value="true"),
            DeclareLaunchArgument("product_rate", default_value="24.0", description="products spawned per minute"),
            DeclareLaunchArgument("max_products", default_value="12", description="max products alive on the infeed"),
            DeclareLaunchArgument("cam_width", default_value="848"),
            DeclareLaunchArgument("cam_height", default_value="480"),
            DeclareLaunchArgument("cam_rate", default_value="30"),
            # Gazebo sensors must render on the GPU; some shells export LIBGL_ALWAYS_SOFTWARE=1 which cripples them
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "0"),
            # package:// meshes (realsense, togsim_description) and model:// lookups
            AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", "/opt/ros/humble/share"),
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", PathJoinSubstitution([FindPackageShare("togsim_gazebo"), "models"])
            ),
            # robot meshes: package://togsim_description/... becomes model://togsim_description/... inside Gazebo
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", PathJoinSubstitution([FindPackageShare("togsim_description"), ".."])
            ),
            OpaqueFunction(function=_expand_world),
            rsp,
            bridge,
            image_bridge,
            *cam_tfs,
            TimerAction(period=3.0, actions=[spawn_robot]),
            RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm_pos, arm_traj])),
            TimerAction(period=8.0, actions=[spawner]),
        ]
    )
