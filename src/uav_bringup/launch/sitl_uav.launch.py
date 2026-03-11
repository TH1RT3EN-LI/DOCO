import os
import shutil
import subprocess
from functools import partial

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
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

from sim_worlds.launch_common import (
    create_launch_summary_action,
    create_static_transform_node,
    is_true,
    normalize_clock_mode,
    parse_six_dof,
    resolve_world_launch_configurations,
)
from uav_bringup.profile_defaults import DEFAULT_PX4_FRAME

SIM_WORLDS_SHARE = get_package_share_directory("sim_worlds")


def generate_launch_description():
    px4_frame = DEFAULT_PX4_FRAME
    frame_prefix = "uav_"
    base_frame = "uav_base_link"
    mono_camera_frame = "uav_camera_optical_frame"
    stereo_camera_frame = "uav_stereo_camera_optical_frame"
    mono_camera_sensor = "uav_mono_camera"
    stereo_rgb_sensor = "uav_stereo_depth_camera_rgb"
    stereo_depth_sensor = "uav_stereo_depth_camera_depth"
    optical_flow_sensor = "uav_optical_flow_camera"
    optical_flow_range_sensor = "uav_optical_flow_range"
    imu_sensor = "uav_imu_sensor"

    bringup_share = get_package_share_directory("uav_bringup")
    description_share = get_package_share_directory("uav_description")
    pkg_prefix = get_package_prefix("uav_bringup")
    script_path = os.path.join(pkg_prefix, "lib", "uav_bringup", "run_px4_gz_uav.sh")
    default_sim_model = "uav"
    default_px4_instance = "0"
    default_model_pose = "0,0.0,0.3,0,0,0"
    xacro_file = os.path.join(description_share, "urdf", "uav.urdf.xacro")

    default_gz_partition = f"uav_{os.getpid()}"

    world = LaunchConfiguration("world")
    resolved_world_id = LaunchConfiguration("resolved_world_id")
    resolved_world_sdf_path = LaunchConfiguration("resolved_world_sdf_path")
    resolved_gz_world_name = LaunchConfiguration("resolved_gz_world_name")
    sim_model = LaunchConfiguration("sim_model")
    px4_instance = LaunchConfiguration("px4_instance")
    model_name = LaunchConfiguration("model_name")
    pose = LaunchConfiguration("pose")
    headless = LaunchConfiguration("headless")
    launch_gz = LaunchConfiguration("launch_gz")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")
    global_frame = LaunchConfiguration("global_frame")
    uav_map_frame = LaunchConfiguration("uav_map_frame")
    uav_odom_frame = LaunchConfiguration("uav_odom_frame")
    global_to_uav_map = LaunchConfiguration("global_to_uav_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")
    px4_start_delay = LaunchConfiguration("px4_start_delay")
    use_offboard_bridge = LaunchConfiguration("use_offboard_bridge")
    uxrce_agent_port = LaunchConfiguration("uxrce_agent_port")
    fmu_namespace = LaunchConfiguration("fmu_namespace")
    attach_existing_model = LaunchConfiguration("attach_existing_model")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    clock_mode = LaunchConfiguration("clock_mode")
    effective_clock_mode = LaunchConfiguration("effective_clock_mode")
    fmu_topic_prefix = PythonExpression(['"', fmu_namespace, '".rstrip("/")'])
    offboard_mode_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/in/offboard_control_mode"'])
    trajectory_setpoint_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/in/trajectory_setpoint"'])
    vehicle_command_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/in/vehicle_command"'])
    distance_sensor_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/in/distance_sensor"'])
    sensor_optical_flow_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/in/sensor_optical_flow"'])
    vehicle_local_position_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/out/vehicle_local_position"'])
    vehicle_status_topic = PythonExpression(['"', fmu_topic_prefix, '" + "/out/vehicle_status"'])
    gz_pose_topic = LaunchConfiguration("gz_pose_topic")
    gz_image_topic = LaunchConfiguration("gz_image_topic")
    default_entity_name = PythonExpression(['"', sim_model, '" + "_" + "', px4_instance, '"'])
    px4_headless = PythonExpression(['"1" if "', headless, '" == "true" else "0"'])
    px4_sim_model = PythonExpression(['"gz_" + "', sim_model, '"'])
    px4_model_name = PythonExpression(
        ['"', model_name, '" if "', attach_existing_model, '".lower() in ("1", "true", "yes", "on") else ""']
    )
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "sitl_uav.rviz")

    resolve_world_action = OpaqueFunction(
        function=partial(
            resolve_world_launch_configurations,
            package_share=SIM_WORLDS_SHARE,
        )
    )

    def resolve_effective_clock_mode(context):
        requested_clock_mode = clock_mode.perform(context)
        if requested_clock_mode.strip():
            return [
                SetLaunchConfiguration(
                    "effective_clock_mode",
                    normalize_clock_mode(
                        requested_clock_mode,
                        default_value="internal" if is_true(launch_gz.perform(context)) else "external",
                    ),
                )
            ]

        return [
            SetLaunchConfiguration(
                "effective_clock_mode",
                "internal" if is_true(launch_gz.perform(context)) else "external",
            )
        ]

    resolve_clock_mode_action = OpaqueFunction(function=resolve_effective_clock_mode)

    def validate_model_configuration(context):
        sim_model_value = sim_model.perform(context).strip()
        px4_instance_value = px4_instance.perform(context).strip()
        model_name_value = model_name.perform(context).strip()

        def fail(message):
            raise RuntimeError(message)

        if not sim_model_value:
            fail("[uav_sitl] sim_model must not be empty.")

        if not model_name_value:
            fail("[uav_sitl] model_name must not be empty.")

        try:
            px4_instance_int = int(px4_instance_value)
        except ValueError:
            fail(
                f"[uav_sitl] px4_instance must be a non-negative integer: "
                f"got '{px4_instance_value}'."
            )

        if px4_instance_int < 0:
            fail(
                f"[uav_sitl] px4_instance must be a non-negative integer: "
                f"got '{px4_instance_value}'."
            )

        if is_true(attach_existing_model.perform(context)):
            return []

        expected_entity_name = f"{sim_model_value}_{px4_instance_int}"
        if model_name_value != expected_entity_name:
            fail(
                f"[uav_sitl] attach_existing_model=false requires model_name to match "
                f"the PX4-spawned Gazebo entity name. expected='{expected_entity_name}', "
                f"configured='{model_name_value}', sim_model='{sim_model_value}', "
                f"px4_instance='{px4_instance_int}'."
            )

        return []

    validate_model_configuration_action = OpaqueFunction(function=validate_model_configuration)

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "world": world,
            "resolved_world_id": resolved_world_id,
            "resolved_world_sdf_path": resolved_world_sdf_path,
            "resolved_gz_world_name": resolved_gz_world_name,
            "headless": headless,
            "render_engine": render_engine,
            "gz_partition": gz_partition,
        }.items(),
        condition=IfCondition(launch_gz),
    )

    internal_clock_condition = IfCondition(
        PythonExpression(['"', effective_clock_mode, '" == "internal"'])
    )
    sim_clock_bridge_config = os.path.join(SIM_WORLDS_SHARE, "config", "ros_gz_bridge_clock.yaml")
    sim_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sim_clock_bridge",
        namespace="sim_clock",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "lazy": False,
                "config_file": sim_clock_bridge_config,
            }
        ],
        remappings=[
            ("/sim/clock_raw", "/clock"),
        ],
        condition=internal_clock_condition,
    )

    def create_global_to_uav_map_tf(context):
        if not is_true(publish_global_map_tf.perform(context)):
            return []

        return [
            create_static_transform_node(
                node_name="global_to_uav_map_tf",
                transform_values=parse_six_dof(global_to_uav_map.perform(context), "global_to_uav_map"),
                parent_frame=global_frame.perform(context),
                child_frame=uav_map_frame.perform(context),
            )
        ]

    global_to_uav_map_tf_action = OpaqueFunction(function=create_global_to_uav_map_tf)

    px4_proc = ExecuteProcess(
        cmd=[script_path],
        additional_env={
            "PX4_GZ_MODEL_NAME": px4_model_name,
            "PX4_GZ_MODEL_POSE": pose,
            "PX4_GZ_WORLD": resolved_gz_world_name,
            "PX4_SIM_MODEL": px4_sim_model,
            "PX4_INSTANCE": px4_instance,
            "UAV_PX4_FRAME": px4_frame,
            "GZ_PARTITION": gz_partition,
            "HEADLESS": px4_headless,
        },
        output="screen",
        emulate_tty=True,
    )
    px4_proc_delayed = TimerAction(
        period=px4_start_delay,
        actions=[
            LogInfo(msg="[uav_sitl] starting PX4 after px4_start_delay."),
            px4_proc,
        ],
    )

    gz_pose_tf_bridge = Node(
        package="uav_bridge",
        executable="tf_bridge_node",
        name="gz_pose_tf_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gz_pose_topic": gz_pose_topic},
            {"gz_world_name": resolved_gz_world_name},
            {"model_name": model_name},
            {"map_frame": uav_map_frame},
            {"odom_frame": uav_odom_frame},
            {"base_frame": base_frame},
            {"odom_topic": "/uav/odom"},
            {"publish_odometry": True},
            {"use_initial_pose_as_map_origin": False},
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
            {"gz_world_name": resolved_gz_world_name},
            {"model_name": model_name},
            {"link_name": base_frame},
            {"sensor_name": mono_camera_sensor},
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
            {"gz_world_name": resolved_gz_world_name},
            {"model_name": model_name},
            {"link_name": base_frame},
            {"rgb_sensor_name": stereo_rgb_sensor},
            {"depth_sensor_name": stereo_depth_sensor},
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
            {"gz_world_name": resolved_gz_world_name},
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
            {"command_frame_id": uav_map_frame},
        ],
        condition=IfCondition(use_offboard_bridge),
    )

    uxrce_agent_node = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", uxrce_agent_port],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_offboard_bridge),
    )

    offboard_wait_state = {"logged": False}

    def launch_offboard_bridge_when_agent_ready(context):
        if not is_true(use_offboard_bridge.perform(context)):
            return []

        port_value = uxrce_agent_port.perform(context)
        try:
            socket_listeners = subprocess.run(
                ["ss", "-lun"],
                capture_output=True,
                text=True,
                check=False,
            ).stdout
        except FileNotFoundError:
            return [
                LogInfo(msg="[uav_sitl] 'ss' unavailable; starting offboard bridge without UDP readiness check."),
                offboard_bridge,
            ]

        if f":{port_value} " in socket_listeners or socket_listeners.rstrip().endswith(f":{port_value}"):
            return [
                LogInfo(msg=f"[uav_sitl] MicroXRCEAgent UDP port ready on {port_value}; starting offboard bridge."),
                offboard_bridge,
            ]

        actions = []
        if not offboard_wait_state["logged"]:
            offboard_wait_state["logged"] = True
            actions.extend([
                LogInfo(msg=f"[uav_sitl] waiting for MicroXRCEAgent UDP port: {port_value}"),
            ])
        actions.append(
            TimerAction(period=0.5, actions=[OpaqueFunction(function=launch_offboard_bridge_when_agent_ready)])
        )
        return actions

    offboard_bridge_ready_action = OpaqueFunction(function=launch_offboard_bridge_when_agent_ready)

    rangefinder_bridge = Node(
        package="uav_bridge",
        executable="rangefinder_bridge_node",
        name="rangefinder_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gz_world_name": resolved_gz_world_name},
            {"model_name": model_name},
            {"link_name": base_frame},
            {"sensor_name": optical_flow_range_sensor},
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
            {"gz_world_name": resolved_gz_world_name},
            {"model_name": model_name},
            {"link_name": base_frame},
            {"sensor_name": optical_flow_sensor},
            {"imu_sensor_name": imu_sensor},
            {"camera_hfov": 0.25},
            {"image_width": 64},
            {"image_height": 64},
            {"range_sensor_name": optical_flow_range_sensor},
            {"range_sample_timeout_ms": 100.0},
            {"range_min_valid_m": 0.08},
            {"range_max_valid_m": 8.0},
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
            {"target_marker_id": 0},
            {"marker_size_m": 0.2},
            {"landing_error_topic": "/uav/visual_landing/landing_error"},
        ],
    )
    visual_landing = Node(
        package="uav_visual_landing",
        executable="visual_landing_node",
        name="visual_landing_node",
        output="screen",
    )
    def create_robot_state_publisher(context):
        if shutil.which("xacro") is None:
            return [
                LogInfo(msg="[uav_sitl] xacro executable not found; skipping robot_state_publisher.")
            ]

        robot_description = Command(
            [
                "xacro",
                " ",
                xacro_file,
                " ",
                "prefix:=",
                frame_prefix,
                " ",
                "base_frame:=base_link",
            ]
        )
        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="uav_robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"robot_description": robot_description},
                ],
                remappings=[
                    ("/robot_description", "/uav/robot_description"),
                ],
            )
        ]

    robot_state_publisher_action = OpaqueFunction(function=create_robot_state_publisher)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    summary_action = create_launch_summary_action(
        "uav_sitl",
        items=[
            ("clock_mode", effective_clock_mode),
            ("px4_frame", px4_frame),
            ("sim_model", sim_model),
            ("px4_instance", px4_instance),
            ("model_name", model_name),
            ("gz_partition", gz_partition),
            ("world", resolved_world_id),
            ("gz_world", resolved_gz_world_name),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("UAV_WORLD", default_value="test"),
                description="Registered sim_worlds world id",
            ),
            DeclareLaunchArgument("resolved_world_id", default_value=""),
            DeclareLaunchArgument("resolved_world_sdf_path", default_value=""),
            DeclareLaunchArgument("resolved_gz_world_name", default_value=""),
            DeclareLaunchArgument(
                "sim_model",
                default_value=default_sim_model,
                description="PX4 Gazebo simulation model base name used for spawn naming",
            ),
            DeclareLaunchArgument(
                "px4_instance",
                default_value=default_px4_instance,
                description="PX4 instance id used for Gazebo entity naming and PX4 process instance selection",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value=default_entity_name,
                description="Gazebo entity name used by bridge nodes and by PX4 when attach_existing_model=true",
            ),
            DeclareLaunchArgument(
                "attach_existing_model",
                default_value=EnvironmentVariable("PX4_GZ_ATTACH_EXISTING_MODEL", default_value="false"),
                description="Attach PX4 to an already spawned Gazebo model named by model_name",
            ),
            DeclareLaunchArgument("pose", default_value=default_model_pose),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "launch_gz",
                default_value="true",
                description="Whether to launch Gazebo simulation process",
            ),
            DeclareLaunchArgument(
                "clock_mode",
                default_value="",
                description="Clock source selection: empty=auto, internal=bridge Gazebo /clock, external=use an existing /clock publisher",
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
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("uav_map_frame", default_value="uav_map"),
            DeclareLaunchArgument("uav_odom_frame", default_value="uav_odom"),
            DeclareLaunchArgument(
                "global_to_uav_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to uav_map_frame",
            ),
            DeclareLaunchArgument(
                "publish_global_map_tf",
                default_value="true",
                description="Whether to publish static transform from global_frame to uav_map_frame",
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
            resolve_world_action,
            resolve_clock_mode_action,
            validate_model_configuration_action,
            summary_action,
            gz_launch,
            sim_clock_bridge,
            global_to_uav_map_tf_action,
            gz_pose_tf_bridge,
            mono_camera_bridge,
            stereo_camera_bridge,
            fmu_topic_namespace_bridge,
            uxrce_agent_node,
            offboard_bridge_ready_action,
            rangefinder_bridge,
            optical_flow_bridge,
            aruco_detector,
            visual_landing,
            robot_state_publisher_action,
            rviz,
            px4_proc_delayed,
        ]
    )
