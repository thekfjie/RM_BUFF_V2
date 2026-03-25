from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("rm_buff_tracker"), "config", "buff_node.yaml"]
    )
    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to ROS2 parameter yaml for buff_tracker_node",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="Upstream camera image topic",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional ROS namespace",
            ),
            Node(
                package="rm_buff_tracker",
                executable="buff_node",
                name="buff_tracker_node",
                namespace=namespace,
                parameters=[params_file],
                remappings=[("~/image_raw", image_topic)],
                output="screen",
            ),
        ]
    )
