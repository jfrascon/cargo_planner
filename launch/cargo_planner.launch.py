"""Launch cargo_planner from one complete YAML parameter file."""

import os
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
import ros2_launch_helpers as rlh

DEFAULT_NODE_ARGS = '{"output":"both","ros_arguments":["--log-level","info"]}'


def generate_launch_description() -> LaunchDescription:
    """Declare the parameter, clock, namespace, and node-action inputs."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='cargo_planner',
                description='Namespace where the cargo planner node is launched.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('cargo_planner'),
                    'config',
                    'default_cargo_planner.yaml',
                ),
                description='Complete YAML file with cargo planner parameters.',
            ),
            DeclareLaunchArgument(
                'params_file_allow_substs',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in params_file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS simulation time when true.',
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value=DEFAULT_NODE_ARGS,
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Validate the parameter file and create the cargo planner node."""
    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    if not params_file:
        raise ValueError("Launch argument 'params_file' must identify a YAML file.")

    if not Path(params_file).is_file():
        raise FileNotFoundError(f"Params file '{params_file}' does not exist.")

    allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool),
        bool,
    )

    parameters: list[Any] = [
        ParameterFile(params_file, allow_substs=allow_substs),
        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
    ]

    return [
        Node(
            package='cargo_planner',
            executable='cargo_planner_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=parameters,
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'cargo_planner'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]
