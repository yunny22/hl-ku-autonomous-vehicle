"""Start the non-actuating public route-following graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("hl_ku_core"), "config", "system.yaml"]
    )
    route = PathJoinSubstitution(
        [FindPackageShare("hl_ku_core"), "routes", "example_route.csv"]
    )
    route_arg = DeclareLaunchArgument(
        "route_file",
        default_value=route,
        description="Synthetic route for graph preview; replace only in a local profile.",
    )
    return LaunchDescription(
        [
            route_arg,
            Node(
                package="hl_ku_core",
                executable="gnss_localizer",
                name="gnss_localizer",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="path_tracker",
                name="path_tracker",
                parameters=[config, {"route_file": LaunchConfiguration("route_file")}],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="mission_manager",
                name="mission_manager",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="perception_aggregator",
                name="perception_aggregator",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="safety_supervisor",
                name="safety_supervisor",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="vehicle_controller",
                name="vehicle_controller",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="hl_ku_core",
                executable="nucleo_serial_bridge",
                name="nucleo_serial_bridge",
                parameters=[config],
                output="screen",
            ),
        ]
    )
