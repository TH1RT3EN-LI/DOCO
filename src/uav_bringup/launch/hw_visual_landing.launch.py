import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")
    visual_landing_share = get_package_share_directory("uav_visual_landing")

    camera_device = LaunchConfiguration("camera_device")
    camera_fourcc = LaunchConfiguration("camera_fourcc")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    camera_fps = LaunchConfiguration("camera_fps")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    base_frame_id = LaunchConfiguration("base_frame_id")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    fx = LaunchConfiguration("fx")
    fy = LaunchConfiguration("fy")
    cx = LaunchConfiguration("cx")
    cy = LaunchConfiguration("cy")
    camera_hfov_rad = LaunchConfiguration("camera_hfov_rad")
    fmu_namespace = LaunchConfiguration("fmu_namespace")

    visual_landing_config = os.path.join(visual_landing_share, "config", "visual_landing_stable.yaml")
    hw_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "hw_mono_camera.launch.py")),
        launch_arguments={
            "device": camera_device,
            "fourcc": camera_fourcc,
            "image_width": image_width,
            "image_height": image_height,
            "fps": camera_fps,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "base_frame_id": base_frame_id,
            "camera_frame_id": camera_frame_id,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "camera_hfov_rad": camera_hfov_rad,
        }.items(),
    )

    vehicle_local_position_topic = [fmu_namespace, "/out/vehicle_local_position"]
    vehicle_odometry_topic = [fmu_namespace, "/out/vehicle_odometry"]
    vehicle_status_topic = [fmu_namespace, "/out/vehicle_status"]
    offboard_mode_topic = [fmu_namespace, "/in/offboard_control_mode"]
    trajectory_setpoint_topic = [fmu_namespace, "/in/trajectory_setpoint"]
    vehicle_command_topic = [fmu_namespace, "/in/vehicle_command"]
    distance_sensor_topic = [fmu_namespace, "/in/distance_sensor"]

    uav_state_bridge = Node(
        package="uav_bridge",
        executable="uav_state_bridge_node",
        name="uav_state_bridge",
        output="screen",
        parameters=[
            {"vehicle_local_position_topic": vehicle_local_position_topic},
            {"vehicle_odometry_topic": vehicle_odometry_topic},
            {"output_odometry_topic": "/uav/state/odometry"},
            {"map_frame_id": "uav_map"},
            {"odom_frame_id": "uav_odom"},
            {"base_frame_id": "uav_base_link"},
            {"publish_rate_hz": 50.0},
            {"publish_odometry": True},
            {"publish_tf": True},
            {"publish_map_to_odom_tf": True},
        ],
    )

    uav_control = Node(
        package="uav_bridge",
        executable="uav_control_node",
        name="uav_control",
        output="screen",
        parameters=[
            {"velocity_body_topic": "/uav/control/setpoint/velocity_body"},
            {"state_topic": "/uav/state/odometry"},
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
            {"takeoff_service": "/uav/control/command/takeoff"},
            {"hold_service": "/uav/control/command/hold"},
            {"land_service": "/uav/control/command/land"},
        ],
    )

    aruco_detector = Node(
        package="uav_visual_landing",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[
            {"target_marker_id": 0},
            {"tag_size_m": 0.1625},
            {"image_topic": image_topic},
            {"camera_info_topic": camera_info_topic},
            {"target_observation_topic": "/uav/visual_landing/target_observation"},
            {"controller_state_topic": "/uav/visual_landing/controller_state"},
            {"debug_image_topic": "/uav/visual_landing/debug_image"},
        ],
    )

    visual_landing = Node(
        package="uav_visual_landing",
        executable="visual_landing_node",
        name="visual_landing_node",
        output="screen",
        parameters=[
            visual_landing_config,
            {"target_observation_topic": "/uav/visual_landing/target_observation"},
            {"state_topic": "/uav/state/odometry"},
            {"velocity_body_topic": "/uav/control/setpoint/velocity_body"},
            {"controller_state_topic": "/uav/visual_landing/controller_state"},
            {"hold_service": "/uav/control/command/hold"},
            {"land_service": "/uav/control/command/land"},
            {"range_topic": distance_sensor_topic},
            {"start_service": "/uav/visual_landing/command/start"},
            {"stop_service": "/uav/visual_landing/command/stop"},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_fourcc", default_value=""),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("camera_fps", default_value="30.0"),
        DeclareLaunchArgument("image_topic", default_value="/uav/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/uav/camera/camera_info"),
        DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
        DeclareLaunchArgument("camera_frame_id", default_value="uav_camera_optical_frame"),
        DeclareLaunchArgument("fx", default_value="387.229248046875"),
        DeclareLaunchArgument("fy", default_value="387.229248046875"),
        DeclareLaunchArgument("cx", default_value="321.04638671875"),
        DeclareLaunchArgument("cy", default_value="243.44969177246094"),
        DeclareLaunchArgument("camera_hfov_rad", default_value="1.3962634"),
        DeclareLaunchArgument("fmu_namespace", default_value="/uav/fmu"),
        hw_camera_launch,
        uav_state_bridge,
        uav_control,
        aruco_detector,
        visual_landing,
    ])
