"""Start GNSS localization and route recording with output disabled by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("hl_ku_core"), "config", "system.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_file",
                default_value="",
                description="Local output CSV path; set explicitly before recording.",
            ),
            Node(
                package="hl_ku_core",
                executable="gnss_localizer",
                name="gnss_localizer",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="route_recorder",
                name="route_recorder",
                parameters=[
                    config,
                    {"output_file": LaunchConfiguration("output_file")},
                ],
                output="screen",
            ),
        ]
    )
