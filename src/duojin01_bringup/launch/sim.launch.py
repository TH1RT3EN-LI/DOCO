import os

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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin01_bringup")
    ugv_bringup_share = get_package_share_directory("ugv_bringup")
    uav_bringup_share = get_package_share_directory("uav_bringup")
    sim_worlds_share = get_package_share_directory("sim_worlds")

    default_gz_partition = f"duojin_{os.getpid()}"
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "sim.rviz")

    world = LaunchConfiguration("world")
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

    ugv_use_teleop = LaunchConfiguration("ugv_use_teleop")
    ugv_keyboard_enabled = LaunchConfiguration("ugv_keyboard_enabled")
    ugv_keyboard_backend = LaunchConfiguration("ugv_keyboard_backend")
    ugv_use_sim_tf = LaunchConfiguration("ugv_use_sim_tf")

    uav_use_offboard_bridge = LaunchConfiguration("uav_use_offboard_bridge")
    uav_model_name = LaunchConfiguration("uav_model_name")
    uav_pose = LaunchConfiguration("uav_pose")
    uav_frame = LaunchConfiguration("uav_frame")
    uav_px4_start_delay = LaunchConfiguration("uav_px4_start_delay")
    uav_uxrce_agent_port = LaunchConfiguration("uav_uxrce_agent_port")
    use_rviz = LaunchConfiguration("use_rviz")
    top_level_use_rviz = LaunchConfiguration("top_level_use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    def parse_six_dof(raw_value: str, arg_name: str):
        tokens = [token.strip() for token in raw_value.replace(",", " ").split() if token.strip()]
        if len(tokens) != 6:
            raise RuntimeError(
                f"Launch argument '{arg_name}' must contain 6 numeric values (x y z roll pitch yaw), got: '{raw_value}'"
            )
        return tokens

    def create_global_map_tfs(context):
        if publish_global_map_tf.perform(context).lower() != "true":
            return []

        ugv_transform = parse_six_dof(global_to_ugv_map.perform(context), "global_to_ugv_map")
        uav_transform = parse_six_dof(global_to_uav_map.perform(context), "global_to_uav_map")
        return [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="global_to_ugv_map_tf",
                output="screen",
                arguments=[
                    *ugv_transform,
                    global_frame.perform(context),
                    ugv_map_frame.perform(context),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="global_to_uav_map_tf",
                output="screen",
                arguments=[
                    *uav_transform,
                    global_frame.perform(context),
                    uav_map_frame.perform(context),
                ],
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
            "headless": headless,
            "gz_partition": gz_partition,
            "render_engine": render_engine,
            "use_rviz": "false",
            "use_teleop": ugv_use_teleop,
            "keyboard_enabled": ugv_keyboard_enabled,
            "keyboard_backend": ugv_keyboard_backend,
            "use_sim_tf": ugv_use_sim_tf,
            "publish_map_tf": "true",
            "global_frame": global_frame,
            "ugv_map_frame": ugv_map_frame,
            "global_to_ugv_map": global_to_ugv_map,
            "publish_global_map_tf": "false",
            "enable_dynamic_global_alignment": enable_dynamic_global_alignment,
            "launch_ros_clock": "false",
        }.items(),
    )

    uav_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(uav_bringup_share, "launch", "sitl_uav.launch.py")),
        launch_arguments={
            "world": world,
            "headless": headless,
            "launch_gz": "false",
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
            "enable_dynamic_global_alignment": enable_dynamic_global_alignment,
            "use_initial_pose_as_map_origin": "false",
            "use_offboard_bridge": uav_use_offboard_bridge,
            "uxrce_agent_port": uav_uxrce_agent_port,
            "use_rviz": "false",
        }.items(),
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("SIM_WORLD", default_value="test"),
                description="Registered sim_worlds world id",
            ),
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
            DeclareLaunchArgument("ugv_use_teleop", default_value="true"),
            DeclareLaunchArgument("ugv_keyboard_enabled", default_value="false"),
            DeclareLaunchArgument("ugv_keyboard_backend", default_value="tty"),
            DeclareLaunchArgument("ugv_use_sim_tf", default_value="true"),
            DeclareLaunchArgument("uav_use_offboard_bridge", default_value="true"),
            DeclareLaunchArgument("uav_model_name", default_value="uav_0"),
            DeclareLaunchArgument("uav_pose", default_value="-0.03,-0.0,0.3,0,0,0"),
            DeclareLaunchArgument(
                "uav_frame",
                default_value=EnvironmentVariable("UAV_PX4_FRAME", default_value="hw_uav"),
            ),
            DeclareLaunchArgument("uav_px4_start_delay", default_value="4.0"),
            DeclareLaunchArgument("uav_uxrce_agent_port", default_value="8888"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            SetLaunchConfiguration("top_level_use_rviz", use_rviz),
            sim_clock_launch,
            global_map_tfs_action,
            ugv_sim_launch,
            uav_sitl_launch,
            rviz_node,
        ]
    )
