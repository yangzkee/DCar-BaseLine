from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("dcaron_bridge")) / "config" / "bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("port", default_value="COM3"),
        DeclareLaunchArgument("telemetry", default_value="velpos"),
        DeclareLaunchArgument("enable_cmd_vel", default_value="false"),
        Node(
            package="dcaron_bridge", executable="dcaron_bridge", name="dcaron_bridge",
            namespace=LaunchConfiguration("namespace"), output="screen",
            parameters=[LaunchConfiguration("params_file"), {
                "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
                "telemetry": ParameterValue(LaunchConfiguration("telemetry"), value_type=str),
                "enable_cmd_vel": ParameterValue(LaunchConfiguration("enable_cmd_vel"), value_type=bool),
            }],
        ),
    ])
