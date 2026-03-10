import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression


def generate_launch_description():
    description_share = get_package_share_directory("uav_description")
    sim_worlds_share = get_package_share_directory("sim_worlds")
    worlds_dir = os.path.join(sim_worlds_share, "worlds")
    uav_models_dir = os.path.join(description_share, "models")
    sim_models_dir = os.path.join(sim_worlds_share, "models")

    resource_dirs = [
        uav_models_dir,
        sim_models_dir,
        worlds_dir,
    ]
    if os.path.isdir("/usr/share/gz"):
        resource_dirs.append("/usr/share/gz")
    resource_path = ":".join([d for d in resource_dirs if os.path.isdir(d)])

    gazebo_cmd = ["gz", "sim"]
    default_gz_partition = f"uav_{os.getpid()}"

    world_name = LaunchConfiguration("world")
    world_file = PythonExpression([
        '"', world_name, '" if "', world_name, '".startswith("/") else "',
        worlds_dir, '/" + "', world_name, '"'
    ])
    headless = LaunchConfiguration("headless")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("UAV_WORLD", default_value="test.sdf"),
                description="World filename under sim_worlds/worlds or absolute path",
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
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            ExecuteProcess(
                cmd=gazebo_cmd
                + [
                    "-r",
                    "-s",
                    "--headless-rendering",
                    "--render-engine-server",
                    render_engine,
                    world_file,
                ],
                output="screen",
                condition=IfCondition(headless),
            ),
            ExecuteProcess(
                cmd=gazebo_cmd + ["-r", "--render-engine", render_engine, world_file],
                output="screen",
                condition=UnlessCondition(headless),
            ),
        ]
    )
