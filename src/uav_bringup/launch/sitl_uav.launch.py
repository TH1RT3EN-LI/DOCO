from uav_bringup.deprecated_launch import forward_to_package_launch


def generate_launch_description():
    return forward_to_package_launch(
        target_package="uav_sim_bringup",
        target_launch_file="sitl_uav.launch.py",
        message="[uav_bringup] 'sitl_uav.launch.py' has moved to package 'uav_sim_bringup'; this wrapper will be removed in a future cleanup.",
    )
