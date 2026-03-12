import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_worlds_share = get_package_share_directory("sim_worlds")
    bridge_config_path = os.path.join(sim_worlds_share, "config", "ros_gz_bridge_clock.yaml")

    gz_partition = LaunchConfiguration("gz_partition")
    clock_topic = LaunchConfiguration("clock_topic")

    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sim_clock_bridge",
        namespace="sim_clock",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "lazy": False,
                "config_file": bridge_config_path,
            }
        ],
        remappings=[
            ("/clock", clock_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_partition",
            default_value=EnvironmentVariable("GZ_PARTITION", default_value=""),
        ),
        DeclareLaunchArgument("clock_topic", default_value="/clock"),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        clock_bridge_node,
    ])
