import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    sim_worlds_share = get_package_share_directory("sim_worlds")
    default_gz_partition = f"uav_{os.getpid()}"

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_worlds_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "world": world,
            "headless": headless,
            "render_engine": render_engine,
            "gz_partition": gz_partition,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("UAV_WORLD", default_value="test"),
                description="Registered sim_worlds world id",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("GZ_PARTITION", default_value=default_gz_partition),
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            gz_launch,
        ]
    )
