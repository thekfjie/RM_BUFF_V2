from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("rm_buff_tracker"), "config", "lab", "buff_spatial_channel.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    observation_topic = LaunchConfiguration("observation_topic")
    target_topic = LaunchConfiguration("target_topic")
    target_frame = LaunchConfiguration("target_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to BUFF spatial channel parameter yaml",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="Upstream camera image topic",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera_info",
                description="Upstream camera info topic",
            ),
            DeclareLaunchArgument(
                "observation_topic",
                default_value="/buff/detector/observation",
                description="Independent BUFF detector observation topic",
            ),
            DeclareLaunchArgument(
                "target_topic",
                default_value="/buff/tracker/target",
                description="Independent BUFF tracker target topic",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="odom",
                description="Frame used by the independent BUFF tracker output",
            ),
            Node(
                package="rm_buff_tracker",
                executable="buff_detector_node",
                name="buff_detector_node",
                parameters=[
                    params_file,
                    {
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "observation_topic": observation_topic,
                    },
                ],
                output="screen",
            ),
            Node(
                package="rm_buff_tracker",
                executable="buff_tracker_node",
                name="buff_spatial_tracker_node",
                parameters=[
                    params_file,
                    {
                        "observation_topic": observation_topic,
                        "target_topic": target_topic,
                        "target_frame": target_frame,
                    },
                ],
                output="screen",
            ),
        ]
    )
