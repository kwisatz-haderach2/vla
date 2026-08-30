#!/usr/bin/env python3
"""Start the ROS 2 force gateway used by the optional UR5e force controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("ros2_force_control_interface"), "config", "force_interface.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="ros2_force_control_interface",
                executable="force_control_gateway.py",
                name="force_control_gateway",
                output="screen",
                parameters=[config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
