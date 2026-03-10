import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

def _world_name_substitution(world_value):
    return PythonExpression([
        '"', world_value, '".rsplit("/", 1)[-1].rsplit(".sdf", 1)[0]'
    ])


def _world_file_substitution(world_value):
    return PythonExpression([
        '"', world_value, '" if "', world_value, '".endswith(".sdf") '
        'else "', world_value, '" + ".sdf"'
    ])


def _topic_in_namespace(namespace_value, suffix):
    return PythonExpression([
        '"', namespace_value, '".rstrip("/") + "', suffix, '"'
    ])


def generate_launch_description():
    frame_default = "sim_uav"
    frame_prefix = "uav_"
    map_frame = "map"
    base_frame = "uav_base_link"
    base_link_name = "base_link"
    mono_camera_frame = "uav_camera_optical_frame"
    stereo_camera_frame = "uav_stereo_camera_optical_frame"

    bringup_share = get_package_share_directory("uav_bringup")
    description_share = get_package_share_directory("uav_description")
    pkg_prefix = get_package_prefix("uav_bringup")
    script_path = os.path.join(pkg_prefix, "lib", "uav_bringup", "run_px4_gz_uav.sh")
    default_model_name = "uav"
    default_model_pose = "1.5,0.0,0.3,0,0,0"
    xacro_file = os.path.join(description_share, "urdf", "uav.urdf.xacro")

    default_gz_partition = f"uav_{os.getpid()}"

    world = LaunchConfiguration("world")
    frame = LaunchConfiguration("frame")
    model_name = LaunchConfiguration("model_name")
    pose = LaunchConfiguration("pose")
    headless = LaunchConfiguration("headless")
    launch_gz = LaunchConfiguration("launch_gz")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")
    px4_start_delay = LaunchConfiguration("px4_start_delay")
    use_initial_pose_as_map_origin = LaunchConfiguration("use_initial_pose_as_map_origin")
    use_offboard_bridge = LaunchConfiguration("use_offboard_bridge")
    uxrce_agent_port = LaunchConfiguration("uxrce_agent_port")
    fmu_namespace = LaunchConfiguration("fmu_namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    sim_world_name = _world_name_substitution(world)
    sim_world_file = _world_file_substitution(world)
    offboard_mode_topic = _topic_in_namespace(fmu_namespace, "/in/offboard_control_mode")
    trajectory_setpoint_topic = _topic_in_namespace(fmu_namespace, "/in/trajectory_setpoint")
    vehicle_command_topic = _topic_in_namespace(fmu_namespace, "/in/vehicle_command")
    distance_sensor_topic = _topic_in_namespace(fmu_namespace, "/in/distance_sensor")
    sensor_optical_flow_topic = _topic_in_namespace(fmu_namespace, "/in/sensor_optical_flow")
    vehicle_local_position_topic = _topic_in_namespace(fmu_namespace, "/out/vehicle_local_position")
    vehicle_status_topic = _topic_in_namespace(fmu_namespace, "/out/vehicle_status")
    gz_pose_topic = LaunchConfiguration("gz_pose_topic")
    gz_image_topic = LaunchConfiguration("gz_image_topic")
    px4_headless = PythonExpression(['"1" if "', headless, '" == "true" else "0"'])
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "sitl_uav.rviz")
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "world": sim_world_file,
            "headless": headless,
            "render_engine": render_engine,
            "gz_partition": gz_partition,
        }.items(),
        condition=IfCondition(launch_gz),
    )

    px4_proc = ExecuteProcess(
        cmd=[script_path],
        additional_env={
            "PX4_GZ_MODEL_NAME": model_name,
            "PX4_GZ_MODEL_POSE": pose,
            "PX4_GZ_WORLD": sim_world_name,
            "PX4_SIM_MODEL": "gz_uav",
            "UAV_PX4_FRAME": frame,
            "GZ_PARTITION": gz_partition,
            "HEADLESS": px4_headless,
        },
        output="screen",
        emulate_tty=True,
    )
    px4_proc_delayed = TimerAction(period=px4_start_delay, actions=[px4_proc])


    gz_pose_tf_bridge = Node(
        package="uav_bridge",
        executable="tf_bridge_node",
        name="gz_pose_tf_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gz_pose_topic": gz_pose_topic},
            {"world_name": sim_world_name},
            {"model_name": model_name},
            {"map_frame": map_frame},
            {"base_frame": base_frame},
            {"base_link_name": base_link_name},
            {"odom_topic": "/uav/odom"},
            {"publish_odometry": True},
            {"use_initial_pose_as_map_origin": use_initial_pose_as_map_origin},
        ],
    )
    
    mono_camera_bridge = Node(
        package="uav_bridge",
        executable="mono_camera_bridge_node",
        name="mono_camera_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gz_image_topic": gz_image_topic},
            {"world_name": sim_world_name},
            {"model_name": model_name},
            {"link_name": "base_link"},
            {"sensor_name": "mono_camera"},
            {"ros_image_topic": "/uav/camera/image_raw"},
            {"frame_id": mono_camera_frame},
        ],
    )

    stereo_camera_bridge = Node(
        package="uav_bridge",
        executable="stereo_camera_bridge_node",
        name="stereo_camera_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"world_name": sim_world_name},
            {"model_name": model_name},
            {"link_name": "base_link"},
            {"rgb_sensor_name": "stereo_depth_camera_rgb"},
            {"depth_sensor_name": "stereo_depth_camera_depth"},
            {"ros_rgb_topic": "/uav/stereo/rgb/image_raw"},
            {"ros_depth_topic": "/uav/stereo/depth/image_raw"},
            {"ros_points_topic": "/uav/stereo/depth/points"},
            {"frame_id": stereo_camera_frame},
            {"points_downsample_step": 4},
        ],
    )

    fmu_topic_namespace_bridge = Node(
        package="uav_bridge",
        executable="fmu_topic_namespace_bridge_node",
        name="fmu_topic_namespace_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"namespaced_fmu_prefix": fmu_namespace},
            {"global_fmu_prefix": "/fmu"},
        ],
    )
    
    offboard_bridge = Node(
        package="uav_bridge",
        executable="offboard_bridge_node",
        name="offboard_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"px4_timestamp_source": "gz_sim"},
            {"world_name": sim_world_name},
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
        ],
        condition=IfCondition(use_offboard_bridge),
    )

    uxrce_agent_node = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", uxrce_agent_port],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_offboard_bridge),
    )

    rangefinder_bridge = Node(
        package="uav_bridge",
        executable="rangefinder_bridge_node",
        name="rangefinder_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"world_name": sim_world_name},
            {"model_name": model_name},
            {"link_name": "base_link"},
            {"sensor_name": "distance_sensor"},
            {"ros_topic": distance_sensor_topic},
        ],
    )

    optical_flow_bridge = Node(
        package="uav_bridge",
        executable="optical_flow_bridge_node",
        name="optical_flow_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"world_name": sim_world_name},
            {"model_name": model_name},
            {"link_name": "base_link"},
            {"sensor_name": "optical_flow"},
            {"camera_hfov": 0.25},
            {"image_width": 64},
            {"image_height": 64},
            {"quality_threshold": 6},
            {"phase_corr_min_response": 0.03},
            {"lk_quality_level": 0.0008},
            {"ros_topic": sensor_optical_flow_topic},
        ],
    )

    aruco_detector = Node(
        package="uav_visual_landing",
        executable="aruco_detector_node",
        name="aruco_detector_node",
        output="screen",
        parameters=[
            {"target_marker_id":0},
        ],
    )
    robot_description = Command(["xacro", " ", xacro_file])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="uav_robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
            {"frame_prefix": frame_prefix},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("UAV_WORLD", default_value="test"),
                description="World name",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value=default_model_name,
                description="Gazebo model name shared by PX4 and bridge nodes",
            ),
            DeclareLaunchArgument("pose", default_value=default_model_pose),
            DeclareLaunchArgument(
                "frame",
                default_value=EnvironmentVariable("UAV_PX4_FRAME", default_value=frame_default),
                description="PX4 frame params profile: sim_uav, default, or simple_quad_x",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "launch_gz",
                default_value="true",
                description="Whether to launch Gazebo simulation process",
            ),
            DeclareLaunchArgument(
                "gz_pose_topic",
                default_value="",
                description="Optional override for Gazebo pose topic",
            ),
            DeclareLaunchArgument(
                "gz_image_topic",
                default_value="",
                description="Optional override for Gazebo image topic",
            ),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("GZ_PARTITION", default_value=default_gz_partition),
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            SetEnvironmentVariable(name="GZ_PARTITION", value=gz_partition),
            DeclareLaunchArgument(
                "px4_start_delay",
                default_value="4.0",
                description="Delay (s) before starting PX4 after Gazebo launch",
            ),
            DeclareLaunchArgument(
                "use_initial_pose_as_map_origin",
                default_value="false",
                description="Use UAV initial pose as map frame origin",
            ),
            DeclareLaunchArgument(
                "use_offboard_bridge",
                default_value="true",
                description="Whether to launch offboard bridge and MicroXRCEAgent",
            ),
            DeclareLaunchArgument(
                "uxrce_agent_port",
                default_value="8888",
                description="MicroXRCEAgent UDP port",
            ),
            DeclareLaunchArgument(
                "fmu_namespace",
                default_value="/uav/fmu",
                description="Namespaced ROS topic prefix used inside the stack for PX4 FMU topics",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to launch RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to RViz2 config file",
            ),
            gz_launch,
            gz_pose_tf_bridge,
            mono_camera_bridge,
            stereo_camera_bridge,
            fmu_topic_namespace_bridge,
            uxrce_agent_node,
            offboard_bridge,
            rangefinder_bridge,
            optical_flow_bridge,
            aruco_detector,
            robot_state_publisher,
            rviz,
            px4_proc_delayed,
        ]
    )
