import os
from dataclasses import dataclass

from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


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


def resolve_world_definition(world_id: str) -> WorldDefinition:
    normalized_world_id = str(world_id).strip()
    if normalized_world_id in WORLD_DEFINITIONS:
        return WORLD_DEFINITIONS[normalized_world_id]

    raise RuntimeError(
        f"Invalid sim_worlds world id '{world_id}'. "
        f"Registered world ids: {available_world_ids_text()}"
    )


def resolve_registered_world(
    world_id: str,
    package_share: str,
    world_sdf_path_override: str = "",
):
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
        resolved_world_sdf_path = os.path.join(
            package_share, "worlds", definition.sdf_file_name
        )
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
    world_id = LaunchConfiguration(world_arg_name).perform(context).strip()

    world_sdf_path_override = ""
    if world_sdf_path_arg_name:
        try:
            world_sdf_path_override = (
                LaunchConfiguration(world_sdf_path_arg_name).perform(context).strip()
            )
        except Exception:
            world_sdf_path_override = ""

    resolved = resolve_registered_world(
        world_id,
        package_share=package_share,
        world_sdf_path_override=world_sdf_path_override,
    )

    return [
        SetLaunchConfiguration(key, value)
        for key, value in resolved.items()
    ]
