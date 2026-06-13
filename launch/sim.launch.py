"""RViz/ros2_control simulation for MediaPipe Inspire hand teleoperation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inspire_tutorials.hand_mapping import normalize_hand_selection
from inspire_tutorials.urdf_builder import build_inspire_hands_robot_description


def _launch_setup(context, *args, **kwargs):
    hand = LaunchConfiguration("hand").perform(context)
    sides = normalize_hand_selection(hand)
    share_dir = Path(get_package_share_directory("inspire_tutorials"))
    description_dir = share_dir / "description" / "inspire_hand"
    robot_description = build_inspire_hands_robot_description(description_dir, hand)
    controllers_yaml = str(share_dir / "config" / "inspire_hand_controllers.yaml")
    rviz_config = str(share_dir / "rviz" / "inspire_hand_teleop.rviz")

    image_topic = LaunchConfiguration("image_topic")
    topic_prefix = LaunchConfiguration("topic_prefix")
    frame_id = LaunchConfiguration("frame_id")
    num_hands = LaunchConfiguration("num_hands")
    video_file = LaunchConfiguration("video_file")
    video_frame_id = LaunchConfiguration("video_frame_id")
    video_playback_speed = LaunchConfiguration("video_playback_speed")
    video_loop = LaunchConfiguration("video_loop")

    nodes = [
        Node(
            package="inspire_tutorials",
            executable="video_file_image_publisher",
            name="video_file_image_publisher",
            output="screen",
            parameters=[
                {
                    "video_file": video_file,
                    "image_topic": image_topic,
                    "frame_id": video_frame_id,
                    "playback_speed": ParameterValue(
                        video_playback_speed,
                        value_type=float,
                    ),
                    "loop": ParameterValue(video_loop, value_type=bool),
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_video_file")),
        ),
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
            condition=IfCondition(LaunchConfiguration("use_camera")),
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
                        LaunchConfiguration("publish_debug_image"),
                        value_type=bool,
                    ),
                }
            ],
            condition=IfCondition(LaunchConfiguration("use_mediapipe")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controllers_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
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
                    "server_timeout_sec": 5.0,
                    "mirror_handedness": ParameterValue(
                        LaunchConfiguration("mirror_handedness"),
                        value_type=bool,
                    ),
                    "allow_missing_controllers": False,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]

    for side in sides:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[f"{side}_hand_controller"],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    default_video_file = str(
        Path(get_package_share_directory("inspire_tutorials"))
        / "test_data"
        / "test.mp4"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand",
                default_value="both",
                choices=["right", "left", "both"],
                description="Simulated hand side to start and command.",
            ),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("topic_prefix", default_value="/mediapipe"),
            DeclareLaunchArgument("frame_id", default_value="world"),
            DeclareLaunchArgument("num_hands", default_value="2"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_video_file", default_value="false"),
            DeclareLaunchArgument("video_file", default_value=default_video_file),
            DeclareLaunchArgument("video_frame_id", default_value="camera"),
            DeclareLaunchArgument("video_playback_speed", default_value="0.5"),
            DeclareLaunchArgument("video_loop", default_value="true"),
            DeclareLaunchArgument("use_mediapipe", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            DeclareLaunchArgument("mirror_handedness", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
