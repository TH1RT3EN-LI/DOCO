import os
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from sim_worlds.launch_common import (
    create_launch_summary_action,
    create_static_transform_node,
    create_true_only_notice_action,
    is_true,
    parse_six_dof,
    resolve_world_launch_configurations,
)
from ugv_bringup.launch_helpers import create_controller_device_ready_gate, resolve_default_map_yaml


def generate_launch_description():
    ugv_bringup_share = get_package_share_directory("ugv_bringup")
    uav_bringup_share = get_package_share_directory("uav_bringup")
    relative_position_fusion_share = get_package_share_directory("relative_position_fusion")
    sim_worlds_share = get_package_share_directory("sim_worlds")
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    default_gz_partition = f"duojin_nav_{os.getpid()}"
    default_rviz_config = os.path.join(package_root, "config", "rviz", "sim_navigation.rviz")
    default_relative_fusion_overlay = os.path.join(package_root, "config", "relative_position_fusion.sim_navigation.yaml")
    default_map_yaml = resolve_default_map_yaml(package_root)
    default_nav2_params = os.path.join(ugv_bringup_share, "config", "nav2.yaml")

    world = LaunchConfiguration("world")
    resolved_world_id = LaunchConfiguration("resolved_world_id")
    resolved_world_sdf_path = LaunchConfiguration("resolved_world_sdf_path")
    resolved_gz_world_name = LaunchConfiguration("resolved_gz_world_name")
    headless = LaunchConfiguration("headless")
    gz_partition = LaunchConfiguration("gz_partition")
    render_engine = LaunchConfiguration("render_engine")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    uav_map_frame = LaunchConfiguration("uav_map_frame")
    uav_odom_frame = LaunchConfiguration("uav_odom_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    global_to_uav_map = LaunchConfiguration("global_to_uav_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")
    enable_dynamic_global_alignment = LaunchConfiguration("enable_dynamic_global_alignment")
    controller_device_path = LaunchConfiguration("controller_device_path")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    nav_autostart = LaunchConfiguration("autostart")
    nav_log_level = LaunchConfiguration("log_level")

    uav_use_offboard_bridge = LaunchConfiguration("uav_use_offboard_bridge")
    uav_sim_model = LaunchConfiguration("uav_sim_model")
    uav_px4_instance = LaunchConfiguration("uav_px4_instance")
    uav_model_name = LaunchConfiguration("uav_model_name")
    uav_pose = LaunchConfiguration("uav_pose")
    uav_frame = LaunchConfiguration("uav_frame")
    uav_px4_start_delay = LaunchConfiguration("uav_px4_start_delay")
    uav_uxrce_agent_port = LaunchConfiguration("uav_uxrce_agent_port")
    use_rviz = LaunchConfiguration("use_rviz")
    top_level_use_rviz = LaunchConfiguration("top_level_use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_relative_position_fusion = LaunchConfiguration("enable_relative_position_fusion")
    enable_relative_tracking = LaunchConfiguration("enable_relative_tracking")
    relative_position_fusion_config = LaunchConfiguration("relative_position_fusion_config")
    default_uav_model_name = PythonExpression(['"', uav_sim_model, '" + "_" + "', uav_px4_instance, '"'])
    sim_nav_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "odom_topic": "/ugv/odom",
        },
        convert_types=True,
    )

    resolve_world_action = OpaqueFunction(
        function=partial(
            resolve_world_launch_configurations,
            package_share=sim_worlds_share,
        )
    )

    def create_global_map_tfs(context):
        if not is_true(publish_global_map_tf.perform(context)):
            return []

        return [
            create_static_transform_node(
                node_name="global_to_ugv_map_tf",
                transform_values=parse_six_dof(global_to_ugv_map.perform(context), "global_to_ugv_map"),
                parent_frame=global_frame.perform(context),
                child_frame=ugv_map_frame.perform(context),
            ),
            create_static_transform_node(
                node_name="global_to_uav_map_tf",
                transform_values=parse_six_dof(global_to_uav_map.perform(context), "global_to_uav_map"),
                parent_frame=global_frame.perform(context),
                child_frame=uav_map_frame.perform(context),
            ),
        ]

    global_map_tfs_action = OpaqueFunction(function=create_global_map_tfs)

    sim_clock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_worlds_share, "launch", "sim_clock.launch.py")),
        launch_arguments={
            "gz_partition": gz_partition,
        }.items(),
    )

    ugv_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_bringup_share, "launch", "sim.launch.py")),
        launch_arguments={
            "world": world,
            "resolved_world_id": resolved_world_id,
            "resolved_world_sdf_path": resolved_world_sdf_path,
            "resolved_gz_world_name": resolved_gz_world_name,
            "headless": headless,
            "gz_partition": gz_partition,
            "render_engine": render_engine,
            "use_rviz": "false",
            "use_teleop": "false",
            "keyboard_enabled": "false",
            "use_sim_tf": "true",
            "publish_map_tf": "false",
            "global_frame": global_frame,
            "ugv_map_frame": ugv_map_frame,
            "global_to_ugv_map": global_to_ugv_map,
            "publish_global_map_tf": "false",
            "clock_mode": "external",
            "controller_device_path": controller_device_path,
        }.items(),
    )

    ugv_scan_rewriter = Node(
        package="ugv_sim_tools",
        executable="scan_frame_rewriter",
        name="scan_frame_rewriter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/ugv/scan_raw",
                "output_topic": "/ugv/scan",
                "output_frame_id": "ugv_laser",
            }
        ],
    )

    ugv_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_bringup_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "use_rviz": "false",
            "autostart": nav_autostart,
            "log_level": nav_log_level,
            "map": map_yaml,
            "map_frame": ugv_map_frame,
            "params_file": sim_nav_params,
            "auto_initial_pose": "true",
        }.items(),
    )

    ugv_nav_launch_gate = create_controller_device_ready_gate(
        controller_device_path=controller_device_path,
        actions=[ugv_nav_launch],
        label="duojin_sim_navigation",
    )

    uav_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(uav_bringup_share, "launch", "sitl_uav.launch.py")),
        launch_arguments={
            "world": world,
            "resolved_world_id": resolved_world_id,
            "resolved_world_sdf_path": resolved_world_sdf_path,
            "resolved_gz_world_name": resolved_gz_world_name,
            "headless": headless,
            "launch_gz": "false",
            "clock_mode": "external",
            "sim_model": uav_sim_model,
            "px4_instance": uav_px4_instance,
            "model_name": uav_model_name,
            "pose": uav_pose,
            "gz_partition": gz_partition,
            "render_engine": render_engine,
            "frame": uav_frame,
            "px4_start_delay": uav_px4_start_delay,
            "global_frame": global_frame,
            "uav_map_frame": uav_map_frame,
            "uav_odom_frame": uav_odom_frame,
            "global_to_uav_map": global_to_uav_map,
            "publish_global_map_tf": "false",
            "use_initial_pose_as_map_origin": "false",
            "use_offboard_bridge": uav_use_offboard_bridge,
            "uxrce_agent_port": uav_uxrce_agent_port,
            "use_rviz": "false",
        }.items(),
    )

    relative_position_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(relative_position_fusion_share, "launch", "relative_position_fusion.launch.py")
        ),
        launch_arguments={
            "preset": "duojin_sim",
            "use_sim_time": "true",
            "global_frame": global_frame,
            "uav_body_frame": "uav_base_link",
            "enable_relative_tracking": enable_relative_tracking,
            "config_overlay": relative_position_fusion_config,
        }.items(),
        condition=IfCondition(enable_relative_position_fusion),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(top_level_use_rviz),
    )

    summary_action = create_launch_summary_action(
        "duojin_sim_navigation",
        items=[
            ("clock_mode", "top_level_internal"),
            ("gz_partition", gz_partition),
            ("world", resolved_world_id),
            ("gz_world", resolved_gz_world_name),
            ("map", map_yaml),
        ],
    )
    no_op_alignment_notice = create_true_only_notice_action(
        "duojin_sim_navigation",
        argument_name="enable_dynamic_global_alignment",
        argument_value=enable_dynamic_global_alignment,
        message="The dynamic global alignment hook has no runtime consumer in the current stack.",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("SIM_WORLD", default_value="test"),
                description="Registered sim_worlds world id",
            ),
            DeclareLaunchArgument("resolved_world_id", default_value=""),
            DeclareLaunchArgument("resolved_world_sdf_path", default_value=""),
            DeclareLaunchArgument("resolved_gz_world_name", default_value=""),
            DeclareLaunchArgument("resolved_ground_height", default_value=""),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("GZ_PARTITION", default_value=default_gz_partition),
            ),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("ugv_map_frame", default_value="ugv_map"),
            DeclareLaunchArgument("uav_map_frame", default_value="uav_map"),
            DeclareLaunchArgument("uav_odom_frame", default_value="uav_odom"),
            DeclareLaunchArgument(
                "global_to_ugv_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to ugv_map_frame",
            ),
            DeclareLaunchArgument(
                "global_to_uav_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to uav_map_frame",
            ),
            DeclareLaunchArgument("publish_global_map_tf", default_value="true"),
            DeclareLaunchArgument("enable_dynamic_global_alignment", default_value="false"),
            DeclareLaunchArgument("controller_device_path", default_value="/tmp/ugv_controller"),
            DeclareLaunchArgument("map", default_value=default_map_yaml),
            DeclareLaunchArgument("params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("uav_use_offboard_bridge", default_value="true"),
            DeclareLaunchArgument("uav_sim_model", default_value="uav"),
            DeclareLaunchArgument("uav_px4_instance", default_value="0"),
            DeclareLaunchArgument(
                "uav_model_name",
                default_value=default_uav_model_name,
                description="Gazebo entity name used by the UAV SITL stack",
            ),
            DeclareLaunchArgument("uav_pose", default_value="-0.03,-0.0,0.3,0,0,0"),
            DeclareLaunchArgument(
                "uav_frame",
                default_value=EnvironmentVariable("UAV_PX4_FRAME", default_value="hw_uav"),
            ),
            DeclareLaunchArgument("uav_px4_start_delay", default_value="4.0"),
            DeclareLaunchArgument("uav_uxrce_agent_port", default_value="8888"),
            DeclareLaunchArgument("enable_relative_position_fusion", default_value="true"),
            DeclareLaunchArgument("enable_relative_tracking", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("relative_position_fusion_config", default_value=default_relative_fusion_overlay),
            SetLaunchConfiguration("top_level_use_rviz", use_rviz),
            resolve_world_action,
            no_op_alignment_notice,
            summary_action,
            sim_clock_launch,
            global_map_tfs_action,
            ugv_sim_launch,
            ugv_scan_rewriter,
            ugv_nav_launch_gate,
            uav_sitl_launch,
            relative_position_fusion_launch,
            rviz_node,
        ]
    )
