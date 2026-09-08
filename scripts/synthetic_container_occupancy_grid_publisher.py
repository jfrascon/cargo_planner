#!/usr/bin/env python3
"""
Publish a synthetic occupancy grid for an empty or partially loaded container.

This utility supports tests without a real robot and is not intended for production use.

Parameters
----------
  container_frame            TF frame of the container interior.              Default:
                             "container_frame"
  container_inner_size_x [m]  Internal container size along container_frame X. Default: 12.01
  container_inner_size_y [m]  Internal container size along container_frame Y. Default:  2.33
  resolution            [m]  Grid cell size.                                  Default:  0.05
  free_cell_value            Occupancy value for empty cells.                 Default:  0
  occupied_cell_value        Occupancy value for occupied cells.              Default:  100
  pre_loaded_cargo_units_file YAML file with explicit pre-loaded cargo unit poses.

Default inner container dimensions are based on a standard 40 ft container
(inner size X = 12.01 m, inner size Y = 2.33 m).

Usage
-----
  Publish a longer container with pre-loaded cargo units:
  ros2 run cargo_planner synthetic_container_occupancy_grid_publisher.py --ros-args \
    -p container_frame:=container_frame \
    -p container_inner_size_x:=13.6 \
    -p container_inner_size_y:=2.4 \
    -p resolution:=0.05 \
    -p free_cell_value:=0 \
    -p occupied_cell_value:=100 \
    -p pre_loaded_cargo_units_file:=/path/to/example_pre_loaded_cargo_units.yaml \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid

  Forward that topic to the `cargo_planner` node:
  ros2 run cargo_planner container_occupancy_grid_forwarder.py --ros-args \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid \
    -r container_occupancy_grid_registration:=cargo_planner/container_occupancy_grid_registration

"""

from collections.abc import Mapping
import math
from pathlib import Path

from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
import yaml


class SyntheticContainerOccupancyGridPublisher(Node):
    """Publishes one synthetic OccupancyGrid for tests and demos."""

    def __init__(self):
        """Declare parameters, build the grid once, and publish it with latched QoS."""
        super().__init__('synthetic_container_occupancy_grid_publisher')

        self.declare_parameter('container_frame', 'container_frame')
        self.declare_parameter('container_inner_size_x', 12.01)
        self.declare_parameter('container_inner_size_y', 2.33)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('free_cell_value', 0)
        self.declare_parameter('occupied_cell_value', 100)
        self.declare_parameter('pre_loaded_cargo_units_file', '')

        self._frame = self._non_empty_string_parameter('container_frame')
        self._inner_size_x = self._positive_float_parameter('container_inner_size_x')
        self._inner_size_y = self._positive_float_parameter('container_inner_size_y')
        self._res = self._positive_float_parameter('resolution')
        self._free_cell_value = self._occupancy_value_parameter('free_cell_value')
        self._occupied_cell_value = self._occupancy_value_parameter('occupied_cell_value')
        self._pre_loaded_cargo_units_file = self._optional_string_parameter(
            'pre_loaded_cargo_units_file'
        )

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(OccupancyGrid, 'container_occupancy_grid', latch_qos)

        self._grid = self._build_grid()
        self._pub.publish(self._grid)
        self.get_logger().info(
            f'Published synthetic grid {self._nx}x{self._ny} cells '
            f'({self._inner_size_x:.1f}x{self._inner_size_y:.1f} m @ {self._res} m/cell) '
            f'in frame {self._frame} with '
            f'{self._pre_loaded_cargo_unit_count} pre-loaded cargo unit(s).'
        )

    def _build_grid(self) -> OccupancyGrid:
        """
        Build a synthetic OccupancyGrid for the internal container area.

        The grid X and Y dimensions are the internal container sizes along the X and Y axes of
        configured frame. Free cells use the configured free cell value, and cargo units
        loaded from the configured YAML file use the configured occupied cell value.
        """
        res = self._res
        self._nx = max(1, math.ceil(self._inner_size_x / res))
        self._ny = max(1, math.ceil(self._inner_size_y / res))
        cell_count = self._nx * self._ny
        data = [self._free_cell_value] * cell_count

        pre_loaded_cargo_units = self._load_pre_loaded_cargo_units()
        self._pre_loaded_cargo_unit_count = len(pre_loaded_cargo_units)
        for index, cargo_unit in enumerate(pre_loaded_cargo_units):
            self._mark_pre_loaded_cargo_unit(data, cargo_unit, index)

        grid = OccupancyGrid()
        stamp = self.get_clock().now().to_msg()
        grid.header.stamp = stamp
        grid.header.frame_id = self._frame
        grid.info = MapMetaData()
        grid.info.map_load_time = stamp
        grid.info.resolution = float(res)
        grid.info.width = self._nx
        grid.info.height = self._ny
        grid.info.origin = Pose()
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data

        return grid

    def _load_pre_loaded_cargo_units(self) -> list[object]:
        """Load and validate the optional YAML list of pre-loaded cargo units."""
        if not self._pre_loaded_cargo_units_file:
            return []

        cargo_units_file = Path(str(self._pre_loaded_cargo_units_file)).expanduser()

        if not cargo_units_file.is_file():
            raise FileNotFoundError(f"Cargo units YAML file '{cargo_units_file}' does not exist")

        with cargo_units_file.open('r', encoding='utf-8') as stream:
            content = yaml.safe_load(stream)

        if content is None:
            return []

        if not isinstance(content, Mapping):
            raise ValueError(
                "Cargo units YAML file must contain a mapping with key 'pre_loaded_cargo_units'"
            )

        if 'pre_loaded_cargo_units' not in content:
            raise ValueError("Cargo units YAML file must contain key 'pre_loaded_cargo_units'")

        cargo_units = content['pre_loaded_cargo_units']

        if not isinstance(cargo_units, list):
            raise ValueError("'pre_loaded_cargo_units' must be a list")

        return cargo_units

    def _mark_pre_loaded_cargo_unit(self, data: list[int], cargo_unit: object, index: int) -> None:
        """Validate one cargo unit and mark its rectangular footprint as occupied."""
        if not isinstance(cargo_unit, Mapping):
            raise ValueError(f'pre_loaded_cargo_units[{index}] must be a mapping')

        x = self._required_float(cargo_unit, 'x', index)
        y = self._required_float(cargo_unit, 'y', index)
        yaw = self._required_float(cargo_unit, 'yaw', index)
        size_x = self._required_float(cargo_unit, 'size_x', index)
        size_y = self._required_float(cargo_unit, 'size_y', index)

        if size_x <= 0.0 or size_y <= 0.0:
            raise ValueError(
                f'pre_loaded_cargo_units[{index}] size_x and size_y must be greater than 0'
            )

        if self._is_parallel_to_x_axis(yaw):
            effective_size_x = size_x
            effective_size_y = size_y
        elif self._is_parallel_to_y_axis(yaw):
            effective_size_x = size_y
            effective_size_y = size_x
        else:
            raise ValueError(
                f'pre_loaded_cargo_units[{index}] yaw={yaw} rad is invalid. '
                'Only multiples of pi/2 rad are supported.'
            )

        x_min = x - effective_size_x * 0.5
        x_max = x + effective_size_x * 0.5
        y_min = y - effective_size_y * 0.5
        y_max = y + effective_size_y * 0.5

        if x_min < 0.0 or y_min < 0.0 or x_max > self._inner_size_x or y_max > self._inner_size_y:
            raise ValueError(
                f'pre_loaded_cargo_units[{index}] footprint is outside the container: '
                f'x=[{x_min:.3f}, {x_max:.3f}], y=[{y_min:.3f}, {y_max:.3f}], '
                f'container inner size in {self._frame}: '
                f'x=[0, {self._inner_size_x:.3f}], y=[0, {self._inner_size_y:.3f}]'
            )

        self._mark_rectangle(data, x_min, x_max, y_min, y_max)

    def _mark_rectangle(
        self, data: list[int], x_min: float, x_max: float, y_min: float, y_max: float
    ) -> None:
        """Mark every grid cell touched by an axis-aligned rectangle as occupied."""
        col_start = max(0, math.floor(x_min / self._res))
        col_end = min(self._nx, math.ceil(x_max / self._res))
        row_start = max(0, math.floor(y_min / self._res))
        row_end = min(self._ny, math.ceil(y_max / self._res))

        for row in range(row_start, row_end):
            for col in range(col_start, col_end):
                data[row * self._nx + col] = self._occupied_cell_value

    @staticmethod
    def _is_close(value: float, expected: float) -> bool:
        """Return true when two angles differ by no more than the local yaw tolerance."""
        return abs(value - expected) <= 1e-6

    @classmethod
    def _is_parallel_to_x_axis(cls, yaw: float) -> bool:
        """Return true when yaw leaves the cargo unit footprint parallel to the X axis."""
        return cls._is_close(math.remainder(yaw, math.pi), 0.0)

    @classmethod
    def _is_parallel_to_y_axis(cls, yaw: float) -> bool:
        """Return true when yaw leaves the cargo unit footprint parallel to the Y axis."""
        return cls._is_close(abs(math.remainder(yaw, math.pi)), math.pi * 0.5)

    def _non_empty_string_parameter(self, name: str) -> str:
        """Return a string parameter after rejecting empty or whitespace-padded values."""
        value = self.get_parameter(name).value
        if not isinstance(value, str) or not value.strip() or value != value.strip():
            raise ValueError(
                f"Parameter '{name}' must be a non-empty string without surrounding whitespace"
            )
        return value

    def _optional_string_parameter(self, name: str) -> str:
        """Return an optional string parameter, converting None to an empty string."""
        value = self.get_parameter(name).value

        if value is None:
            return ''

        if not isinstance(value, str):
            raise ValueError(f"Parameter '{name}' must be a string")

        if value and (not value.strip() or value != value.strip()):
            raise ValueError(
                f"Parameter '{name}' must be empty or a string without surrounding whitespace"
            )

        return value

    def _occupancy_value_parameter(self, name: str) -> int:
        """Return an OccupancyGrid cell value in the documented [-1, 100] range."""
        value = self._int_parameter(name)

        if value < -1 or value > 100:
            raise ValueError(f"Parameter '{name}' must be between -1 and 100")

        return value

    def _positive_float_parameter(self, name: str) -> float:
        """Return a numeric parameter after requiring a finite value greater than zero."""
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            raise ValueError(f"Parameter '{name}' must be numeric")

        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Parameter '{name}' must be numeric") from exc

        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"Parameter '{name}' must be greater than 0")

        return number

    def _int_parameter(self, name: str) -> int:
        """Return an integer parameter after rejecting bool and non-integer values."""
        value = self.get_parameter(name).value

        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"Parameter '{name}' must be an integer")

        return value

    @staticmethod
    def _required_float(item: Mapping, key: str, index: int) -> float:
        """Return a required cargo unit field as a finite float."""
        if key not in item:
            raise ValueError(f"pre_loaded_cargo_units[{index}] is missing required key '{key}'")

        if isinstance(item[key], bool):
            raise ValueError(f"pre_loaded_cargo_units[{index}] key '{key}' must be numeric")

        try:
            value = float(item[key])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"pre_loaded_cargo_units[{index}] key '{key}' must be numeric"
            ) from exc

        if not math.isfinite(value):
            raise ValueError(f"pre_loaded_cargo_units[{index}] key '{key}' must be finite")

        return value


def main(args=None):
    """Initialize ROS, run the synthetic publisher node, and shut it down cleanly."""
    rclpy.init(args=args)
    node = None

    try:
        node = SyntheticContainerOccupancyGridPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
