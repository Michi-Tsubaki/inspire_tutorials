"""Real Inspire hand teleoperation from MediaPipe landmarks.

This launch assumes the G1 hand controllers are started elsewhere, for example
from g1_bringup with right_hand_controller and/or left_hand_controller.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hand = LaunchConfiguration("hand")
    image_topic = LaunchConfiguration("image_topic")
    topic_prefix = LaunchConfiguration("topic_prefix")
    frame_id = LaunchConfiguration("frame_id")
    num_hands = LaunchConfiguration("num_hands")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = str(
        Path(get_package_share_directory("inspire_tutorials"))
        / "rviz"
        / "inspire_hand_teleop.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand",
                default_value="both",
                choices=["right", "left", "both"],
                description="Robot hand side to command.",
            ),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("topic_prefix", default_value="/mediapipe"),
            DeclareLaunchArgument("frame_id", default_value="world"),
            DeclareLaunchArgument("num_hands", default_value="2"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument("mirror_handedness", default_value="false"),
            DeclareLaunchArgument("allow_missing_controllers", default_value="true"),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                output="screen",
                parameters=[
                    {
                        "video_device": LaunchConfiguration("video_device"),
                    }
                ],
                remappings=[("image_raw", image_topic)],
                condition=IfCondition(use_camera),
            ),
            Node(
                package="mediapipe_ros2_py",
                executable="mp_node",
                name="mediapipe_hand_node",
                output="screen",
                parameters=[
                    {
                        "model": "hand",
                        "image_topic": image_topic,
                        "topic_prefix": topic_prefix,
                        "frame_id": frame_id,
                        "use_gesture": False,
                        "num_hands": ParameterValue(num_hands, value_type=int),
                        "publish_debug_image": ParameterValue(
                            publish_debug_image,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="inspire_tutorials",
                executable="inspire_hand_teleop",
                name="inspire_hand_teleop",
                output="screen",
                parameters=[
                    {
                        "hand": hand,
                        "landmarks_topic": ParameterValue(
                            [topic_prefix, "/hand/landmarks"],
                            value_type=str,
                        ),
                        "target_joint_states_topic": "/inspire_hand/target_joint_states",
                        "right_action_name": "/right_hand_controller/follow_joint_trajectory",
                        "left_action_name": "/left_hand_controller/follow_joint_trajectory",
                        "mirror_handedness": ParameterValue(
                            LaunchConfiguration("mirror_handedness"),
                            value_type=bool,
                        ),
                        "allow_missing_controllers": ParameterValue(
                            LaunchConfiguration("allow_missing_controllers"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    rviz_config,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
