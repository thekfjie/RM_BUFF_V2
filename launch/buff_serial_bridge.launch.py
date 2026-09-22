from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("rm_buff_tracker"), "config", "lab", "buff_serial_bridge.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_config),
        Node(package="rm_buff_tracker", executable="buff_serial_bridge",
             name="buff_serial_bridge", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
    ])
