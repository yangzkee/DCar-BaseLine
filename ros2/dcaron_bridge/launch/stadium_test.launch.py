"""One command starts the serial bridge and continuous stadium acceptance run."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('dcaron_bridge'))
    test = Node(package='dcaron_bridge', executable='stadium_test', output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='COM3'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/bridge.launch.py')),
                                 launch_arguments={'port': LaunchConfiguration('port'),
                                                   'enable_motion_actions': 'true'}.items()),
        RegisterEventHandler(OnProcessExit(target_action=test,
                                          on_exit=[EmitEvent(event=Shutdown(reason='Stadium test ended'))])),
        test,
    ])
