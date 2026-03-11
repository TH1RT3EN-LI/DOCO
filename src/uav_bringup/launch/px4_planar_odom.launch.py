from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    vehicle_local_position_topic = LaunchConfiguration("vehicle_local_position_topic")
    vehicle_odometry_topic = LaunchConfiguration("vehicle_odometry_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    world_frame_id = LaunchConfiguration("world_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    log_state = LaunchConfiguration("log_state")

    planar_state_reader = Node(
        package="uav_bridge",
        executable="px4_planar_state_reader_node",
        name="px4_planar_state_reader",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "vehicle_local_position_topic": vehicle_local_position_topic,
                "vehicle_odometry_topic": vehicle_odometry_topic,
                "output_odom_topic": output_odom_topic,
                "world_frame_id": world_frame_id,
                "base_frame_id": base_frame_id,
                "publish_rate_hz": publish_rate_hz,
                "publish_odometry": True,
                "log_state": log_state,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            ),
            DeclareLaunchArgument(
                "vehicle_local_position_topic",
                default_value="/uav/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "vehicle_odometry_topic",
                default_value="/uav/fmu/out/vehicle_odometry",
            ),
            DeclareLaunchArgument("output_odom_topic", default_value="/uav/odom"),
            DeclareLaunchArgument("world_frame_id", default_value="uav_map"),
            DeclareLaunchArgument("base_frame_id", default_value="uav_base_link"),
            DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("log_state", default_value="false"),
            planar_state_reader,
        ]
    )
