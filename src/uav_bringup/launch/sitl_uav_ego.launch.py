import os
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from sim_worlds.launch_common import (
    create_launch_summary_action,
    create_rviz_conditions,
    create_true_only_notice_action,
    normalize_clock_mode,
    resolve_world_launch_configurations,
)
from uav_bringup.profile_defaults import (
    VEHICLE_PROFILES,
    profile_expression,
)

SIM_WORLDS_SHARE = get_package_share_directory("sim_worlds")


def _topic_in_namespace(namespace_value, suffix):
    return PythonExpression([
        '"', namespace_value, '".rstrip("/") + "', suffix, '"'
    ])


def generate_launch_description():
    bringup_share = get_package_share_directory("uav_bringup")

    default_gz_partition = f"uav_{os.getpid()}"
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "sitl_uav_ego.rviz")

    world = LaunchConfiguration("world")
    resolved_world_id = LaunchConfiguration("resolved_world_id")
    resolved_world_sdf_path = LaunchConfiguration("resolved_world_sdf_path")
    resolved_gz_world_name = LaunchConfiguration("resolved_gz_world_name")
    vehicle_profile = LaunchConfiguration("vehicle_profile")
    model_name = LaunchConfiguration("model_name")
    frame = LaunchConfiguration("frame")
    headless = LaunchConfiguration("headless")
    gz_pose_topic = LaunchConfiguration("gz_pose_topic")
    gz_image_topic = LaunchConfiguration("gz_image_topic")
    gz_partition = LaunchConfiguration("gz_partition")
    render_engine = LaunchConfiguration("render_engine")
    px4_start_delay = LaunchConfiguration("px4_start_delay")
    global_frame = LaunchConfiguration("global_frame")
    uav_map_frame = LaunchConfiguration("uav_map_frame")
    uav_odom_frame = LaunchConfiguration("uav_odom_frame")
    global_to_uav_map = LaunchConfiguration("global_to_uav_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")
    enable_dynamic_global_alignment = LaunchConfiguration("enable_dynamic_global_alignment")
    use_initial_pose_as_map_origin = LaunchConfiguration("use_initial_pose_as_map_origin")
    use_offboard_bridge = LaunchConfiguration("use_offboard_bridge")
    uxrce_agent_port = LaunchConfiguration("uxrce_agent_port")
    fmu_namespace = LaunchConfiguration("fmu_namespace")
    attach_existing_model = LaunchConfiguration("attach_existing_model")
    clock_mode = LaunchConfiguration("clock_mode")
    effective_clock_mode = LaunchConfiguration("effective_clock_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_gl = LaunchConfiguration("rviz_software_gl")
    rviz_config = LaunchConfiguration("rviz_config")
    offboard_mode_topic = _topic_in_namespace(fmu_namespace, "/in/offboard_control_mode")
    trajectory_setpoint_topic = _topic_in_namespace(fmu_namespace, "/in/trajectory_setpoint")
    vehicle_command_topic = _topic_in_namespace(fmu_namespace, "/in/vehicle_command")
    vehicle_local_position_topic = _topic_in_namespace(fmu_namespace, "/out/vehicle_local_position")
    vehicle_status_topic = _topic_in_namespace(fmu_namespace, "/out/vehicle_status")

    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    ground_height = LaunchConfiguration("ground_height")
    max_vel = LaunchConfiguration("max_vel")
    max_acc = LaunchConfiguration("max_acc")
    planning_horizon = LaunchConfiguration("planning_horizon")
    cam2body_x = LaunchConfiguration("cam2body_x")
    cam2body_y = LaunchConfiguration("cam2body_y")
    cam2body_z = LaunchConfiguration("cam2body_z")
    frame_default = PythonExpression(profile_expression(VEHICLE_PROFILES, vehicle_profile, "frame"))
    fx_default = PythonExpression(profile_expression(VEHICLE_PROFILES, vehicle_profile, "fx"))
    fy_default = PythonExpression(profile_expression(VEHICLE_PROFILES, vehicle_profile, "fy"))
    cx_default = PythonExpression(profile_expression(VEHICLE_PROFILES, vehicle_profile, "cx"))
    cy_default = PythonExpression(profile_expression(VEHICLE_PROFILES, vehicle_profile, "cy"))
    cam2body_x_default = PythonExpression(
        profile_expression(VEHICLE_PROFILES, vehicle_profile, "cam2body_x")
    )
    cam2body_y_default = PythonExpression(
        profile_expression(VEHICLE_PROFILES, vehicle_profile, "cam2body_y")
    )
    cam2body_z_default = PythonExpression(
        profile_expression(VEHICLE_PROFILES, vehicle_profile, "cam2body_z")
    )
    effective_ground_height = LaunchConfiguration("effective_ground_height")

    resolve_world_action = OpaqueFunction(
        function=partial(
            resolve_world_launch_configurations,
            package_share=SIM_WORLDS_SHARE,
        )
    )

    def resolve_ground_height(context):
        override_value = ground_height.perform(context).strip()
        if override_value:
            resolved_value = override_value
        else:
            resolved_value = LaunchConfiguration("resolved_ground_height").perform(context)
        return [SetLaunchConfiguration("effective_ground_height", resolved_value)]

    resolve_ground_height_action = OpaqueFunction(function=resolve_ground_height)

    def resolve_effective_clock_mode(context):
        requested_clock_mode = clock_mode.perform(context)
        return [
            SetLaunchConfiguration(
                "effective_clock_mode",
                normalize_clock_mode(requested_clock_mode, default_value="internal"),
            )
        ]

    resolve_clock_mode_action = OpaqueFunction(function=resolve_effective_clock_mode)

    sitl_launch = GroupAction(
        actions=[
            PushLaunchConfigurations(),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sitl_uav.launch.py")),
                launch_arguments={
                    "world": world,
                    "resolved_world_id": resolved_world_id,
                    "resolved_world_sdf_path": resolved_world_sdf_path,
                    "resolved_gz_world_name": resolved_gz_world_name,
                    "resolved_ground_height": LaunchConfiguration("resolved_ground_height"),
                    "model_name": model_name,
                    "frame": frame,
                    "headless": headless,
                    "clock_mode": effective_clock_mode,
                    "gz_pose_topic": gz_pose_topic,
                    "gz_image_topic": gz_image_topic,
                    "gz_partition": gz_partition,
                    "render_engine": render_engine,
                    "px4_start_delay": px4_start_delay,
                    "global_frame": global_frame,
                    "uav_map_frame": uav_map_frame,
                    "uav_odom_frame": uav_odom_frame,
                    "global_to_uav_map": global_to_uav_map,
                    "publish_global_map_tf": publish_global_map_tf,
                    "enable_dynamic_global_alignment": enable_dynamic_global_alignment,
                    "fmu_namespace": fmu_namespace,
                    "attach_existing_model": attach_existing_model,
                    "use_initial_pose_as_map_origin": use_initial_pose_as_map_origin,
                    "use_offboard_bridge": "false",
                    "use_rviz": "false",
                }.items(),
            ),
            PopLaunchConfigurations(),
        ]
    )

    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="uav_ego_planner",
        output="screen",
        remappings=[
            ("odom_world", "/uav/odom"),
            ("planning/bspline", "/uav/planning/bspline"),
            ("planning/data_display", "/uav/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/uav/planning/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/uav/planning/broadcast_bspline"),
            ("goal_point", "/uav/plan_vis/goal_point"),
            ("global_list", "/uav/plan_vis/global_list"),
            ("init_list", "/uav/plan_vis/init_list"),
            ("optimal_list", "/uav/plan_vis/optimal_list"),
            ("a_star_list", "/uav/plan_vis/a_star_list"),
            ("grid_map/odom", "/uav/odom"),
            ("grid_map/depth", "/uav/stereo/depth/image_raw"),
            ("grid_map/occupancy_inflate", "/uav/grid_map/occupancy_inflate"),
            ("/move_base_simple/goal", "/uav/goal_pose"),
        ],
        parameters=[
            {"fsm/flight_type": 1},
            {"fsm/thresh_replan_time": 1.0},
            {"fsm/thresh_no_replan_meter": 1.0},
            {"fsm/planning_horizon": planning_horizon},
            {"fsm/planning_horizen_time": 3.0},
            {"fsm/emergency_time": 1.0},
            {"fsm/realworld_experiment": False},
            {"fsm/fail_safe": True},
            {"fsm/waypoint_num": 1},
            {"fsm/waypoint0_x": 5.0},
            {"fsm/waypoint0_y": 0.0},
            {"fsm/waypoint0_z": 1.0},
            {"grid_map/resolution": 0.10},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": 6.0},
            {"grid_map/local_update_range_y": 6.0},
            {"grid_map/local_update_range_z": 4.5},
            {"grid_map/obstacles_inflation": 0.20},
            {"grid_map/local_map_margin": 10},
            {"grid_map/ground_height": effective_ground_height},
            {"grid_map/cx": cx_default},
            {"grid_map/cy": cy_default},
            {"grid_map/fx": fx_default},
            {"grid_map/fy": fy_default},
            {"grid_map/cam2body_x": cam2body_x},
            {"grid_map/cam2body_y": cam2body_y},
            {"grid_map/cam2body_z": cam2body_z},
            {"grid_map/use_depth_filter": True},
            {"grid_map/depth_filter_tolerance": 0.15},
            {"grid_map/depth_filter_maxdist": 8.0},
            {"grid_map/depth_filter_mindist": 0.2},
            {"grid_map/depth_filter_margin": 2},
            {"grid_map/k_depth_scaling_factor": 1000.0},
            {"grid_map/skip_pixel": 2},
            {"grid_map/p_hit": 0.65},
            {"grid_map/p_miss": 0.35},
            {"grid_map/p_min": 0.12},
            {"grid_map/p_max": 0.90},
            {"grid_map/p_occ": 0.80},
            {"grid_map/min_ray_length": 0.1},
            {"grid_map/max_ray_length": 8.0},
            {"grid_map/virtual_ceil_height": 2.8},
            {"grid_map/visualization_truncate_height": 2.8},
            {"grid_map/show_occ_time": False},
            {"grid_map/pose_type": 2},
            {"grid_map/frame_id": uav_map_frame},
            {"vis/frame_id": uav_map_frame},
            {"grid_map/odom_depth_timeout": 1.0},
            {"manager/max_vel": max_vel},
            {"manager/max_acc": max_acc},
            {"manager/max_jerk": 4.0},
            {"manager/control_points_distance": 0.4},
            {"manager/feasibility_tolerance": 0.05},
            {"manager/planning_horizon": planning_horizon},
            {"manager/use_distinctive_trajs": False},
            {"manager/drone_id": 0},
            {"optimization/lambda_smooth": 1.0},
            {"optimization/lambda_collision": 0.5},
            {"optimization/lambda_feasibility": 0.1},
            {"optimization/lambda_fitness": 1.0},
            {"optimization/dist0": 0.5},
            {"optimization/swarm_clearance": 0.5},
            {"optimization/max_vel": max_vel},
            {"optimization/max_acc": max_acc},
            {"bspline/limit_vel": max_vel},
            {"bspline/limit_acc": max_acc},
            {"bspline/limit_ratio": 1.1},
            {"prediction/obj_num": 0},
            {"prediction/lambda": 1.0},
            {"prediction/predict_rate": 1.0},
        ],
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name="uav_traj_server",
        output="screen",
        remappings=[
            ("planning/bspline", "/uav/planning/bspline"),
            ("position_cmd", "/uav/planning/position_cmd"),
            ("/position_cmd", "/uav/planning/position_cmd"),
        ],
        parameters=[
            {"traj_server/time_forward": 1.0},
            {"traj_server/frame_id": uav_map_frame},
        ],
    )

    uxrce_agent_node = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", uxrce_agent_port],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_offboard_bridge),
    )

    offboard_bridge_node = Node(
        package="uav_bridge",
        executable="offboard_bridge_node",
        name="uav_offboard_bridge",
        output="screen",
        parameters=[
            {"command_topic": "/uav/planning/position_cmd"},
            {"offboard_mode_topic": offboard_mode_topic},
            {"trajectory_setpoint_topic": trajectory_setpoint_topic},
            {"vehicle_command_topic": vehicle_command_topic},
            {"planner_odom_topic": "/uav/odom"},
            {"px4_local_position_topic": vehicle_local_position_topic},
            {"vehicle_status_topic": vehicle_status_topic},
            {"command_frame_id": uav_map_frame},
            {"enable_frame_alignment": True},
            {"lock_alignment_on_first_valid": True},
            {"relock_on_local_position_reset": True},
            {"publish_rate_hz": 50.0},
            {"warmup_cycles": 20},
            {"target_system": 1},
            {"target_component": 1},
            {"source_system": 1},
            {"source_component": 1},
        ],
        condition=IfCondition(use_offboard_bridge),
    )

    rviz_soft_condition, rviz_hw_condition = create_rviz_conditions(use_rviz, rviz_software_gl)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={
            "LIBGL_DRI3_DISABLE": "1",
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
            "QT_XCB_GL_INTEGRATION": "none",
            "QT_OPENGL": "software",
        },
        condition=rviz_soft_condition,
    )
    rviz_node_hw = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=rviz_hw_condition,
    )

    summary_action = create_launch_summary_action(
        "uav_sitl_ego",
        items=[
            ("clock_mode", effective_clock_mode),
            ("gz_partition", gz_partition),
            ("world", resolved_world_id),
            ("gz_world", resolved_gz_world_name),
        ],
    )
    no_op_alignment_notice = create_true_only_notice_action(
        "uav_sitl_ego",
        argument_name="enable_dynamic_global_alignment",
        argument_value=enable_dynamic_global_alignment,
        message="The dynamic global alignment hook has no runtime consumer in the current stack.",
    )
    initial_pose_notice = create_true_only_notice_action(
        "uav_sitl_ego",
        argument_name="use_initial_pose_as_map_origin",
        argument_value=use_initial_pose_as_map_origin,
        message="The tf bridge keeps the static map->odom contract and ignores this flag.",
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
            DeclareLaunchArgument("resolved_ground_height", default_value=""),
            DeclareLaunchArgument(
                "model_name",
                default_value=EnvironmentVariable("UAV_MODEL_NAME", default_value="uav"),
                description="Gazebo model name used by bridge nodes and by PX4 when attach_existing_model=true",
            ),
            DeclareLaunchArgument(
                "vehicle_profile",
                default_value=EnvironmentVariable("UAV_VEHICLE_PROFILE", default_value="sim"),
                description="Vehicle defaults profile: sim or hw",
            ),
            DeclareLaunchArgument(
                "frame",
                default_value=EnvironmentVariable(
                    "UAV_PX4_FRAME",
                    default_value=frame_default,
                ),
                description="PX4 frame params profile: sim_uav, hw_uav, default, or simple_quad_x",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("clock_mode", default_value=""),
            DeclareLaunchArgument("effective_clock_mode", default_value=""),
            DeclareLaunchArgument("gz_pose_topic", default_value=""),
            DeclareLaunchArgument("gz_image_topic", default_value=""),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("GZ_PARTITION", default_value=default_gz_partition),
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            DeclareLaunchArgument(
                "px4_start_delay",
                default_value="4.0",
                description="Delay (s) before starting PX4 after Gazebo launch",
            ),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("uav_map_frame", default_value="uav_map"),
            DeclareLaunchArgument("uav_odom_frame", default_value="uav_odom"),
            DeclareLaunchArgument(
                "global_to_uav_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to uav_map_frame",
            ),
            DeclareLaunchArgument("publish_global_map_tf", default_value="true"),
            DeclareLaunchArgument("enable_dynamic_global_alignment", default_value="false"),
            DeclareLaunchArgument(
                "use_initial_pose_as_map_origin",
                default_value="false",
            ),
            DeclareLaunchArgument("use_offboard_bridge", default_value="true"),
            DeclareLaunchArgument("uxrce_agent_port", default_value="8888"),
            DeclareLaunchArgument("fmu_namespace", default_value="/uav/fmu"),
            DeclareLaunchArgument(
                "attach_existing_model",
                default_value=EnvironmentVariable("PX4_GZ_ATTACH_EXISTING_MODEL", default_value="false"),
                description="Attach PX4 to an already spawned Gazebo model named by model_name",
            ),
            DeclareLaunchArgument("map_size_x", default_value="50.0"),
            DeclareLaunchArgument("map_size_y", default_value="50.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("ground_height", default_value=""),
            DeclareLaunchArgument("max_vel", default_value="2.0"),
            DeclareLaunchArgument("max_acc", default_value="4.0"),
            DeclareLaunchArgument("planning_horizon", default_value="7.5"),
            DeclareLaunchArgument("cam2body_x", default_value=cam2body_x_default),
            DeclareLaunchArgument("cam2body_y", default_value=cam2body_y_default),
            DeclareLaunchArgument("cam2body_z", default_value=cam2body_z_default),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("UAV_RVIZ_SOFTWARE_GL", default_value="true"),
                description="Use software OpenGL for RViz2 to improve container GPU compatibility",
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            resolve_world_action,
            resolve_clock_mode_action,
            resolve_ground_height_action,
            no_op_alignment_notice,
            initial_pose_notice,
            summary_action,
            sitl_launch,
            uxrce_agent_node,
            offboard_bridge_node,
            ego_planner_node,
            traj_server_node,
            rviz_node,
            rviz_node_hw,
        ]
    )
