import os
from functools import partial

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from sim_worlds.launch_common import is_true, resolve_world_launch_configurations


def _display_is_available():
    wayland_display = os.environ.get("WAYLAND_DISPLAY", "").strip()
    xdg_runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if wayland_display and xdg_runtime_dir:
        wayland_socket = os.path.join(xdg_runtime_dir, wayland_display)
        if os.path.exists(wayland_socket):
            return True, f"wayland socket '{wayland_socket}'"

    display = os.environ.get("DISPLAY", "").strip()
    if not display:
        return False, "DISPLAY and WAYLAND_DISPLAY are both unset"

    normalized_display = display
    if normalized_display.startswith("unix:"):
        normalized_display = normalized_display[len("unix:") :]

    if normalized_display.startswith(":"):
        screen = normalized_display[1:].split(".", 1)[0]
        if screen.isdigit():
            x11_socket = f"/tmp/.X11-unix/X{screen}"
            if os.path.exists(x11_socket):
                return True, f"local X11 socket '{x11_socket}'"
            return False, f"local X11 socket '{x11_socket}' not found for DISPLAY='{display}'"

    if ":" in normalized_display:
        return True, f"remote X11 display '{display}'"

    return False, f"invalid DISPLAY format '{display}'"


def _resolve_effective_headless(context):
    requested_headless = is_true(LaunchConfiguration("headless").perform(context))
    if requested_headless:
        return [SetLaunchConfiguration("effective_headless", "true")]

    has_display, reason = _display_is_available()
    if has_display:
        return [SetLaunchConfiguration("effective_headless", "false")]

    return [
        SetLaunchConfiguration("effective_headless", "true"),
        LogInfo(
            msg=(
                "[sim_worlds] GUI unavailable, auto-switch to headless. "
                f"reason={reason}; DISPLAY='{os.environ.get('DISPLAY', '')}'; "
                f"WAYLAND_DISPLAY='{os.environ.get('WAYLAND_DISPLAY', '')}'."
            )
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("sim_worlds")
    models_dir = os.path.join(pkg_share, "models")
    worlds_dir = os.path.join(pkg_share, "worlds")

    try:
        ugv_desc = get_package_share_directory("ugv_description")
        ugv_desc_parent = os.path.dirname(ugv_desc)
    except Exception:
        ugv_desc_parent = None

    try:
        uav_desc = get_package_share_directory("uav_description")
        uav_models_dir = os.path.join(uav_desc, "models")
    except Exception:
        uav_models_dir = None

    resource_dirs = []
    if uav_models_dir and os.path.isdir(uav_models_dir):
        resource_dirs.append(uav_models_dir)
    resource_dirs.extend([models_dir, worlds_dir])
    if ugv_desc_parent and os.path.isdir(ugv_desc_parent):
        resource_dirs.append(ugv_desc_parent)
    if os.path.isdir("/usr/share/gz"):
        resource_dirs.append("/usr/share/gz")
    resource_path = ":".join([directory for directory in resource_dirs if os.path.isdir(directory)])

    world_file = LaunchConfiguration("resolved_world_sdf_path")
    effective_headless = LaunchConfiguration("effective_headless")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="baylands",
            description="Registered sim_worlds world id",
        ),
        DeclareLaunchArgument(
            "world_sdf_path",
            default_value="",
            description="Internal override for resolved SDF path",
        ),
        DeclareLaunchArgument("resolved_world_id", default_value=""),
        DeclareLaunchArgument("resolved_world_sdf_path", default_value=""),
        DeclareLaunchArgument("resolved_gz_world_name", default_value=""),
        DeclareLaunchArgument("resolved_ground_height", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gz_partition", default_value=EnvironmentVariable("GZ_PARTITION", default_value="")),
        DeclareLaunchArgument("render_engine", default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2")),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        OpaqueFunction(
            function=partial(
                resolve_world_launch_configurations,
                package_share=pkg_share,
            )
        ),
        OpaqueFunction(function=_resolve_effective_headless),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", "--headless-rendering", "--render-engine-server", render_engine, world_file],
            output="screen",
            condition=IfCondition(effective_headless),
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", "--render-engine", render_engine, world_file],
            output="screen",
            condition=UnlessCondition(effective_headless),
        ),
    ])
