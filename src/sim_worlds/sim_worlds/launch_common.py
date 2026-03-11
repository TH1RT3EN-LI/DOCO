import os
from dataclasses import dataclass

from launch.actions import LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


@dataclass(frozen=True)
class WorldDefinition:
    world_id: str
    sdf_file_name: str
    gz_world_name: str
    ground_height: str


WORLD_DEFINITIONS = {
    "test": WorldDefinition(
        world_id="test",
        sdf_file_name="test.sdf",
        gz_world_name="test",
        ground_height="-0.30",
    ),
    "baylands": WorldDefinition(
        world_id="baylands",
        sdf_file_name="baylands.sdf",
        gz_world_name="baylands",
        ground_height="-0.30",
    ),
    "empty": WorldDefinition(
        world_id="empty",
        sdf_file_name="empty.sdf",
        gz_world_name="empty_world",
        ground_height="0.0",
    ),
}


def available_world_ids():
    return tuple(WORLD_DEFINITIONS.keys())


def available_world_ids_text():
    return ", ".join(available_world_ids())


def is_true(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def read_launch_configuration(context, name: str, default: str = "") -> str:
    try:
        return LaunchConfiguration(name).perform(context).strip()
    except Exception:
        return default


def all_true_condition(*substitutions):
    expression = []
    for index, substitution in enumerate(substitutions):
        if index:
            expression.append(" and ")
        expression.extend([
            '"',
            substitution,
                '".strip().lower() in ("1", "true", "yes", "on")',
        ])
    return IfCondition(PythonExpression(expression))


def create_rviz_conditions(use_rviz, rviz_software_gl):
    software_condition = all_true_condition(use_rviz, rviz_software_gl)
    hardware_condition = IfCondition(
        PythonExpression([
            '"',
            use_rviz,
            '".strip().lower() in ("1", "true", "yes", "on") and "',
            rviz_software_gl,
            '".strip().lower() not in ("1", "true", "yes", "on")',
        ])
    )
    return software_condition, hardware_condition


def parse_six_dof(raw_value: str, arg_name: str):
    tokens = [token.strip() for token in raw_value.replace(",", " ").split() if token.strip()]
    if len(tokens) != 6:
        raise RuntimeError(
            f"Launch argument '{arg_name}' must contain 6 numeric values (x y z roll pitch yaw), got: '{raw_value}'"
        )
    return tokens


def create_static_transform_node(*, node_name: str, transform_values, parent_frame: str, child_frame: str):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=node_name,
        output="screen",
        arguments=[*transform_values, parent_frame, child_frame],
    )


def resolve_world_definition(world_id: str) -> WorldDefinition:
    normalized_world_id = str(world_id).strip()
    if normalized_world_id in WORLD_DEFINITIONS:
        return WORLD_DEFINITIONS[normalized_world_id]

    raise RuntimeError(
        f"Invalid sim_worlds world id '{world_id}'. "
        f"Registered world ids: {available_world_ids_text()}"
    )


def resolve_registered_world(world_id: str, package_share: str, world_sdf_path_override: str = ""):
    definition = resolve_world_definition(world_id)
    resolved_world_sdf_path = str(world_sdf_path_override).strip()
    if resolved_world_sdf_path:
        if not os.path.isabs(resolved_world_sdf_path):
            raise RuntimeError(
                "Internal sim_worlds world_sdf_path override must be an absolute path"
            )
        if not os.path.isfile(resolved_world_sdf_path):
            raise RuntimeError(
                f"Resolved sim_worlds override world file does not exist: '{resolved_world_sdf_path}'"
            )
    else:
        resolved_world_sdf_path = os.path.join(package_share, "worlds", definition.sdf_file_name)
        if not os.path.isfile(resolved_world_sdf_path):
            raise RuntimeError(
                f"Registered sim_worlds world file does not exist: '{resolved_world_sdf_path}'"
            )

    return {
        "resolved_world_id": definition.world_id,
        "resolved_world_sdf_path": resolved_world_sdf_path,
        "resolved_gz_world_name": definition.gz_world_name,
        "resolved_ground_height": definition.ground_height,
    }


def resolve_world_launch_configurations(
    context,
    *,
    package_share: str,
    world_arg_name: str = "world",
    world_sdf_path_arg_name: str = "world_sdf_path",
):
    existing = {
        key: read_launch_configuration(context, key)
        for key in (
            "resolved_world_id",
            "resolved_world_sdf_path",
            "resolved_gz_world_name",
            "resolved_ground_height",
        )
    }
    if existing["resolved_world_id"] and existing["resolved_world_sdf_path"] and existing["resolved_gz_world_name"]:
        if not existing["resolved_ground_height"]:
            existing["resolved_ground_height"] = resolve_world_definition(existing["resolved_world_id"]).ground_height
        return [SetLaunchConfiguration(key, value) for key, value in existing.items()]

    world_id = read_launch_configuration(context, world_arg_name)
    world_sdf_path_override = read_launch_configuration(context, world_sdf_path_arg_name)
    resolved = resolve_registered_world(
        world_id,
        package_share=package_share,
        world_sdf_path_override=world_sdf_path_override,
    )
    return [SetLaunchConfiguration(key, value) for key, value in resolved.items()]


def create_launch_summary_action(label: str, *, items):
    def _log_summary(context):
        rendered_items = []
        for key, value in items:
            rendered_value = value.perform(context) if hasattr(value, "perform") else str(value)
            rendered_items.append(f"{key}={rendered_value}")
        return [LogInfo(msg=f"[{label}] " + ", ".join(rendered_items))]

    return OpaqueFunction(function=_log_summary)


def create_true_only_notice_action(label: str, *, argument_name: str, argument_value, message: str):
    def _maybe_log(context):
        rendered_value = argument_value.perform(context) if hasattr(argument_value, "perform") else str(argument_value)
        if not is_true(rendered_value):
            return []
        return [LogInfo(msg=f"[{label}] '{argument_name}' is accepted for compatibility but is currently a no-op. {message}")]

    return OpaqueFunction(function=_maybe_log)


def normalize_clock_mode(value: str, *, default_value: str) -> str:
    normalized_value = str(value).strip().lower() or default_value
    if normalized_value not in {"internal", "external"}:
        raise RuntimeError(
            f"Invalid clock_mode '{value}'. Expected one of: internal, external"
        )
    return normalized_value
