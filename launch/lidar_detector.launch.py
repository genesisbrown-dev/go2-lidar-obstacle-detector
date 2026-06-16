"""
lidar_detector.launch.py
------------------------
ROS2 launch file for the go2-lidar-obstacle-detector node.

Usage (from workspace root after building):
    ros2 launch go2_lidar_detector lidar_detector.launch.py

You can override the lidar topic at launch time:
    ros2 launch go2_lidar_detector lidar_detector.launch.py \
        lidar_topic:=/custom/topic
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Build and return the LaunchDescription for the detector node.

    A LaunchDescription is the ROS2 way of specifying which nodes
    to start, with what parameters, and on which topics.
    """

    # ── Declare overridable launch arguments ─────────────────
    # Users can pass lidar_topic:=<value> on the command line
    # without needing to edit source code.
    lidar_topic_arg = DeclareLaunchArgument(
        name="lidar_topic",
        default_value="/unitree_go2_0/lidar/point_cloud",
        description="PointCloud2 topic published by Isaac Sim or the Go2 driver",
    )

    # ── Define the detector node ──────────────────────────────
    detector_node = Node(
        package="go2_lidar_detector",          # ROS2 package name (must match setup.py)
        executable="lidar_detector",            # Entry point name (must match setup.py)
        name="lidar_detector_node",             # Runtime node name
        output="screen",                        # Print logs to the terminal
        parameters=[
            {"lidar_topic": LaunchConfiguration("lidar_topic")}
        ],
    )

    return LaunchDescription([
        lidar_topic_arg,
        detector_node,
    ])
