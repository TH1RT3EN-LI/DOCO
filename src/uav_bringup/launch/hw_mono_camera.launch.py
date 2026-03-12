import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from uav_bringup.profile_defaults import DEFAULT_CAMERA_INTRINSICS, DEFAULT_CAMERA_TO_BODY


def generate_launch_description():
    device = LaunchConfiguration("device")
    fourcc = LaunchConfiguration("fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    fps = LaunchConfiguration("fps")
    camera_name = LaunchConfiguration("camera_name")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    base_frame_id = LaunchConfiguration("base_frame_id")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")
    fx = LaunchConfiguration("fx")
    fy = LaunchConfiguration("fy")
    cx = LaunchConfiguration("cx")
    cy = LaunchConfiguration("cy")
    camera_hfov_rad = LaunchConfiguration("camera_hfov_rad")

    mono_camera_source = Node(
        package="uav_bridge",
        executable="mono_camera_source_node",
        name="mono_camera_source",
        output="screen",
        parameters=[
            {
                "device": device,
                "fourcc": fourcc,
                "camera_name": camera_name,
                "image_width": image_width,
                "image_height": image_height,
                "fps": fps,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "frame_id": camera_frame_id,
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "camera_hfov_rad": camera_hfov_rad,
            }
        ],
    )

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="uav_camera_static_tf",
        output="screen",
        arguments=[
            "--x",
            camera_x,
            "--y",
            camera_y,
            "--z",
            camera_z,
            "--roll",
            camera_roll,
            "--pitch",
            camera_pitch,
            "--yaw",
            camera_yaw,
            "--frame-id",
            base_frame_id,
            "--child-frame-id",
            camera_frame_id,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value=EnvironmentVariable("UAV_MONO_CAMERA_DEVICE", default_value="/dev/video0"),
                description="Mono camera video device path, e.g. /dev/video0",
            ),
            DeclareLaunchArgument(
                "fourcc",
                default_value=EnvironmentVariable("UAV_MONO_CAMERA_FOURCC", default_value=""),
                description="Optional V4L2 fourcc such as YUYV or MJPG",
            ),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument("camera_name", default_value="uav_mono_camera"),
            DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument("camera_frame_id", default_value="uav_camera_optical_frame"),
            DeclareLaunchArgument("camera_x", default_value=DEFAULT_CAMERA_TO_BODY["x"]),
            DeclareLaunchArgument("camera_y", default_value=DEFAULT_CAMERA_TO_BODY["y"]),
            DeclareLaunchArgument("camera_z", default_value=DEFAULT_CAMERA_TO_BODY["z"]),
            DeclareLaunchArgument("camera_roll", default_value=str(-math.pi / 2.0)),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value=str(-math.pi / 2.0)),
            DeclareLaunchArgument("fx", default_value=DEFAULT_CAMERA_INTRINSICS["fx"]),
            DeclareLaunchArgument("fy", default_value=DEFAULT_CAMERA_INTRINSICS["fy"]),
            DeclareLaunchArgument("cx", default_value=DEFAULT_CAMERA_INTRINSICS["cx"]),
            DeclareLaunchArgument("cy", default_value=DEFAULT_CAMERA_INTRINSICS["cy"]),
            DeclareLaunchArgument("camera_hfov_rad", default_value="1.3962634"),
            camera_static_tf,
            mono_camera_source,
        ]
    )
