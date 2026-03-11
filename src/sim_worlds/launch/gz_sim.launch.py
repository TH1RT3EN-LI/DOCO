import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    sim_worlds_share = get_package_share_directory("sim_worlds")

    world = LaunchConfiguration("world")
    world_sdf_path = LaunchConfiguration("world_sdf_path")
    resolved_world_id = LaunchConfiguration("resolved_world_id")
    resolved_world_sdf_path = LaunchConfiguration("resolved_world_sdf_path")
    resolved_gz_world_name = LaunchConfiguration("resolved_gz_world_name")
    resolved_ground_height = LaunchConfiguration("resolved_ground_height")
    headless = LaunchConfiguration("headless")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")

    runtime_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_worlds_share, "launch", "runtime.launch.py")),
        launch_arguments={
            "world": world,
            "world_sdf_path": world_sdf_path,
            "resolved_world_id": resolved_world_id,
            "resolved_world_sdf_path": resolved_world_sdf_path,
            "resolved_gz_world_name": resolved_gz_world_name,
            "resolved_ground_height": resolved_ground_height,
            "headless": headless,
            "render_engine": render_engine,
            "gz_partition": gz_partition,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=EnvironmentVariable("SIM_WORLD", default_value="baylands"),
            description="Registered sim_worlds world id",
        ),
        DeclareLaunchArgument("world_sdf_path", default_value=""),
        DeclareLaunchArgument("resolved_world_id", default_value=""),
        DeclareLaunchArgument("resolved_world_sdf_path", default_value=""),
        DeclareLaunchArgument("resolved_gz_world_name", default_value=""),
        DeclareLaunchArgument("resolved_ground_height", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gz_partition", default_value=EnvironmentVariable("GZ_PARTITION", default_value="")),
        DeclareLaunchArgument("render_engine", default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2")),
        runtime_launch,
    ])
