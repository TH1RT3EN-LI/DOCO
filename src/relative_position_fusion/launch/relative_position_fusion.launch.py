import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("relative_position_fusion")
    config_dir = os.path.join(package_share, "config")

    preset = LaunchConfiguration("preset")
    use_sim_time = LaunchConfiguration("use_sim_time")
    global_frame = LaunchConfiguration("global_frame")
    uav_body_frame = LaunchConfiguration("uav_body_frame")
    enable_relative_tracking = LaunchConfiguration("enable_relative_tracking")

    def create_node(context):
        preset_value = preset.perform(context).strip()
        tracking_enabled = enable_relative_tracking.perform(context).strip().lower() in ("1", "true", "yes", "on")
        preset_map = {
            "duojin_sim": os.path.join(config_dir, "relative_position_fusion.duojin_sim.yaml"),
            "sim_navigation": os.path.join(config_dir, "relative_position_fusion.sim_navigation.yaml"),
            "hw": os.path.join(config_dir, "relative_position_fusion.hw.yaml"),
        }
        if preset_value not in preset_map:
            raise RuntimeError(
                f"Unknown relative_position_fusion preset '{preset_value}'. "
                f"Expected one of: {', '.join(sorted(preset_map.keys()))}"
            )

        common_yaml = os.path.join(config_dir, "relative_position_fusion.common.yaml")
        preset_yaml = preset_map[preset_value]
        tracking_yaml = os.path.join(config_dir, "relative_tracking.common.yaml")

        nodes = [
            Node(
                package="relative_position_fusion",
                executable="relative_position_fusion_node",
                name="relative_position_fuser",
                output="screen",
                parameters=[
                    common_yaml,
                    preset_yaml,
                    {
                        "use_sim_time": use_sim_time,
                        "global_frame": global_frame,
                        "uav_body_frame": uav_body_frame,
                    },
                ],
            )
        ]

        if tracking_enabled:
            nodes.append(
                Node(
                    package="relative_position_fusion",
                    executable="relative_tracking_node",
                    name="relative_tracking_controller",
                    output="screen",
                    parameters=[
                        tracking_yaml,
                        {
                            "use_sim_time": use_sim_time,
                        },
                    ],
                )
            )

        return nodes

    return LaunchDescription(
        [
            DeclareLaunchArgument("preset", default_value="duojin_sim"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            ),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("uav_body_frame", default_value="uav_base_link"),
            DeclareLaunchArgument("enable_relative_tracking", default_value="false"),
            OpaqueFunction(function=create_node),
        ]
    )
