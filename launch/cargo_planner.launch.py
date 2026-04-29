#!/usr/bin/env python3
"""
Launches the cargo_planner node, loading parameters from
config/example_cargo_planner.yaml (installed with the package).
That file is used by default. Pass params_file:=/path/to/your.yaml to use a
different file, and override individual parameters via CLI launch arguments.

ros2 launch cargo_planner cargo_planner.launch.py \\
    namespace:=my_cargo_planner \\
    pallet_margin:=0.1 \\
    enable_rotation:=false \\
    node_options:="{name: cargo_planner, output: screen}"
"""

import os
from pathlib import Path
from typing import Any, List

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace', default_value='cargo_planner', description='ROS 2 namespace for the node'
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('cargo_planner'), 'config', 'example_cargo_planner.yaml'
                ),
                description='YAML file with node parameters.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock when True.',
            ),
            DeclareLaunchArgument(
                'occupied_threshold',
                default_value='',
                description='Grid cells with value >= this are treated as occupied.',
            ),
            DeclareLaunchArgument(
                'pallet_margin',
                default_value='',
                description='Safety clearance around obstacles before pallet placement [m].',
            ),
            DeclareLaunchArgument(
                'enable_rotation',
                default_value='',
                choices=['True', 'true', 'False', 'false', ''],
                description='Allow cargo units to be placed at 90 degrees yaw when True.',
            ),
            DeclareLaunchArgument('node_remappings', default_value='', description=rlh.REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_logging_options',
                default_value=rlh.default_logging_options_str(),
                description=rlh.LOGGING_OPTIONS_DESC,
            ),
            OpaqueFunction(function=launch_cargo_planner_node),
        ]
    )


def launch_cargo_planner_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # If params_file exists, load it first. CLI launch arguments added later
    # override only the specific parameters that were provided.
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx).strip()
    occupied_threshold = LaunchConfiguration('occupied_threshold').perform(ctx).strip()
    pallet_margin = LaunchConfiguration('pallet_margin').perform(ctx).strip()
    enable_rotation = LaunchConfiguration('enable_rotation').perform(ctx).strip()

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist.")

        parameters.append(ParameterFile(params_file, allow_substs=True))

    if occupied_threshold:
        try:
            parameters.append({'occupied_threshold': int(occupied_threshold)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for occupied_threshold: '{occupied_threshold}'. Must be an int.") from exc

    if pallet_margin:
        try:
            parameters.append({'pallet_margin': float(pallet_margin)})
        except ValueError as exc:
            raise ValueError(f"Invalid value for pallet_margin: '{pallet_margin}'. Must be a float.") from exc

    if enable_rotation:
        parameters.append({'enable_rotation': enable_rotation.lower() == 'true'})

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    node_options: dict[str, Any] = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'cargo_planner'

    if not rlh.is_valid_name(node_name):
        raise RuntimeError(f"The name of the node must be ASCII [A-Za-z0-9_] only: '{node_name}'")

    return [
        Node(
            package='cargo_planner',
            executable='cargo_planner_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            remappings=rlh.process_remappings(LaunchConfiguration('node_remappings').perform(ctx)),
            ros_arguments=rlh.process_node_logging_options(LaunchConfiguration('node_logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
