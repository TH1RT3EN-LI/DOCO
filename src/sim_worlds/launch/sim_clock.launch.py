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
    raw_clock_topic = LaunchConfiguration("raw_clock_topic")
    clock_topic = LaunchConfiguration("clock_topic")
    allow_clock_reset = LaunchConfiguration("allow_clock_reset")
    reset_newer_than_sec = LaunchConfiguration("reset_newer_than_sec")
    reset_older_than_sec = LaunchConfiguration("reset_older_than_sec")

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
            ("/sim/clock_raw", raw_clock_topic),
        ],
    )

    clock_guard_node = Node(
        package="ugv_sim_tools",
        executable="clock_guard_node",
        name="ugv_clock_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_topic": raw_clock_topic,
                "output_topic": clock_topic,
                "allow_clock_reset": allow_clock_reset,
                "reset_newer_than_sec": reset_newer_than_sec,
                "reset_older_than_sec": reset_older_than_sec,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_partition",
            default_value=EnvironmentVariable("GZ_PARTITION", default_value=""),
        ),
        DeclareLaunchArgument("raw_clock_topic", default_value="/sim/clock_raw"),
        DeclareLaunchArgument("clock_topic", default_value="/clock"),
        DeclareLaunchArgument("allow_clock_reset", default_value="true"),
        DeclareLaunchArgument("reset_newer_than_sec", default_value="30.0"),
        DeclareLaunchArgument("reset_older_than_sec", default_value="5.0"),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        clock_bridge_node,
        clock_guard_node,
    ])
