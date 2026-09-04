"""Test reusable validation used by the cargo planner visualizer."""

import math
from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest

from cargo_planner.visualizer import load_cargo_units, occupancy_grid_to_free_mask, to_cells


def test_load_cargo_units_validates_and_preserves_dimensions(tmp_path: Path) -> None:
    """Load a complete cargo list using the same contract as the ROS service."""
    cargo_file = tmp_path / 'cargo.yaml'
    cargo_file.write_text(
        'cargo_units:\n  - id: cargo_01\n    lx: 1.2\n    ly: 0.8\n    lz: 1.5\n', encoding='utf-8'
    )

    units = load_cargo_units(cargo_file)

    assert len(units) == 1
    assert units[0].id == 'cargo_01'
    assert units[0].volume == pytest.approx(1.44)


@pytest.mark.parametrize(
    'cargo_yaml',
    [
        'cargo_units:\n  - id: ""\n    lx: 1.2\n    ly: 0.8\n    lz: 1.5\n',
        'cargo_units:\n  - id: cargo\n    lx: 0.0\n    ly: 0.8\n    lz: 1.5\n',
        'cargo_units:\n  - id: cargo\n    lx: .inf\n    ly: 0.8\n    lz: 1.5\n',
        'cargo_units:\n  - id: cargo\n    lx: 1.2\n    ly: 0.8\n    lz: 1.5\n    typo: 1\n',
        (
            'cargo_units:\n'
            '  - {id: cargo, lx: 1.2, ly: 0.8, lz: 1.5}\n'
            '  - {id: cargo, lx: 1.0, ly: 0.8, lz: 1.5}\n'
        ),
    ],
)
def test_load_cargo_units_rejects_invalid_entries(tmp_path: Path, cargo_yaml: str) -> None:
    """Reject invalid cargo before generating explanatory images."""
    cargo_file = tmp_path / 'cargo.yaml'
    cargo_file.write_text(cargo_yaml, encoding='utf-8')

    with pytest.raises(ValueError):
        load_cargo_units(cargo_file)


@pytest.mark.parametrize(
    ('metres', 'resolution'), [(-1.0, 0.1), (1.0, 0.0), (math.inf, 0.1), (1.0, math.nan)]
)
def test_to_cells_rejects_invalid_values(metres: float, resolution: float) -> None:
    """Reject values that cannot produce a finite grid-cell count."""
    with pytest.raises(ValueError):
        to_cells(metres, resolution)


def test_occupancy_grid_to_free_mask_rejects_invalid_threshold() -> None:
    """Keep the visualizer occupancy contract aligned with the C++ converter."""
    grid = OccupancyGrid()
    grid.info.width = 1
    grid.info.height = 1
    grid.data = [0]

    with pytest.raises(ValueError, match='occupied_threshold'):
        occupancy_grid_to_free_mask(grid, 101)

    assert np.array_equal(occupancy_grid_to_free_mask(grid, 50), np.array([[True]]))
