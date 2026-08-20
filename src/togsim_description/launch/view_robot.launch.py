"""Visualise the tog-sim robot in RViz with joint sliders (no simulation)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " '",
                PathJoinSubstitution([FindPackageShare("togsim_description"), "urdf", "togsim_robot.urdf.xacro"]),
                "' sim:=false",
            ]
        ),
        value_type=str,
    )
    rviz_config = PathJoinSubstitution([FindPackageShare("togsim_description"), "rviz", "view_robot.rviz"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start joint_state_publisher_gui + RViz"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(gui)
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="log",
                condition=IfCondition(gui),
            ),
        ]
    )
