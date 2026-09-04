"""Test the public cargo_planner launch contract."""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_NODE_ARGS = '{"output":"both","ros_arguments":["--log-level","info"]}'


def _load_launch_module() -> ModuleType:
    """Load the source launch file as a Python module."""
    path = PACKAGE_ROOT / 'launch' / 'cargo_planner.launch.py'
    spec = importlib.util.spec_from_file_location('cargo_planner_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_exposes_only_file_clock_namespace_and_node_arguments() -> None:
    """Protect the compact launch API and prevent functional parameter overrides."""
    module = _load_launch_module()
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert set(declarations) == {
        'namespace',
        'params_file',
        'params_file_allow_substs',
        'use_sim_time',
        'node_args',
    }

    context = LaunchContext()
    declarations['node_args'].visit(context)
    assert context.launch_configurations['node_args'] == DEFAULT_NODE_ARGS


@pytest.mark.parametrize('allow_substs', ['True', 'False'])
def test_launch_passes_parameter_file_and_clock_to_the_node(
    allow_substs: str, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Keep use_sim_time as the only node parameter outside the YAML file."""
    module = _load_launch_module()
    params_file = tmp_path / 'params.yaml'
    params_file.write_text('/**:\n  ros__parameters:\n    pallet_margin: 0.05\n', encoding='utf-8')
    captured: dict[str, object] = {}

    class FakeParameterFile:
        def __init__(self, path: str, *, allow_substs: bool) -> None:
            self.path = path
            self.allow_substs = allow_substs

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    monkeypatch.setattr(module, 'ParameterFile', FakeParameterFile)
    monkeypatch.setattr(module, 'Node', FakeNode)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'namespace': 'cargo',
            'params_file': str(params_file),
            'params_file_allow_substs': allow_substs,
            'use_sim_time': 'False',
            'node_args': DEFAULT_NODE_ARGS,
        }
    )

    actions = module._launch_node(context)

    assert len(actions) == 1
    assert captured['name'] == 'cargo_planner'
    assert captured['output'] == 'both'
    assert captured['ros_arguments'] == ['--log-level', 'info']
    parameters = captured['parameters']
    assert len(parameters) == 2
    assert parameters[0].path == str(params_file)
    assert parameters[0].allow_substs is (allow_substs == 'True')
    assert set(parameters[1]) == {'use_sim_time'}


def test_launch_rejects_a_missing_parameter_file(tmp_path: Path) -> None:
    """Reject an explicit missing YAML file before starting the node."""
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'namespace': 'cargo',
            'params_file': str(tmp_path / 'missing.yaml'),
            'params_file_allow_substs': 'False',
            'use_sim_time': 'False',
            'node_args': DEFAULT_NODE_ARGS,
        }
    )

    with pytest.raises(FileNotFoundError, match='missing.yaml'):
        module._launch_node(context)
