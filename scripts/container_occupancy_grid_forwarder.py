#!/usr/bin/env python3
"""
Forward a container occupancy grid to the cargo planner service.

This utility supports tests without a real robot and is not intended for production use.

The node subscribes to container_occupancy_grid, wraps each received grid in a
ContainerOccupancyGridRegistration request, sets the internal container size along Z, and calls
container_occupancy_grid_registration.

Parameters
----------
  container_inner_size_z [m] Internal container size along the vertical axis. Default: 2.38
  wait_for_service_timeout [s]  Time to wait for the planner service after a
                                grid arrives. Default: 2.0
  one_shot                   If true, forward only the first received grid and
                             then shut down. If false, forward every grid.
                             Default: true

Usage
-----
  Keep running and forward every new grid with all node parameters set explicitly:
  ros2 run cargo_planner container_occupancy_grid_forwarder.py --ros-args \
    -p container_inner_size_z:=2.38 \
    -p wait_for_service_timeout:=2.0 \
    -p one_shot:=false \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid \
    -r container_occupancy_grid_registration:=cargo_planner/container_occupancy_grid_registration

"""

import math

from cargo_planner_msgs.srv import ContainerOccupancyGridRegistration
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile


class ContainerOccupancyGridForwarder(Node):
    """Forwards container_occupancy_grid messages to container_occupancy_grid_registration."""

    def __init__(self):
        """Declare parameters and subscribe to the container occupancy grid topic."""
        super().__init__('container_occupancy_grid_forwarder')

        self.declare_parameter('container_inner_size_z', 2.38)
        self.declare_parameter('wait_for_service_timeout', 2.0)
        self.declare_parameter('one_shot', True)

        self._container_inner_size_z = self._positive_float_parameter('container_inner_size_z')
        self._wait_for_service_timeout = self._non_negative_float_parameter(
            'wait_for_service_timeout'
        )
        self._one_shot = self._bool_parameter('one_shot')
        self._forwarded = False
        self._request_in_flight = False

        self._client = self.create_client(
            ContainerOccupancyGridRegistration, 'container_occupancy_grid_registration'
        )

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        self._sub = self.create_subscription(
            OccupancyGrid, 'container_occupancy_grid', self._on_grid, qos
        )

        self.get_logger().info(
            'Waiting for container_occupancy_grid to forward to '
            f'container_occupancy_grid_registration (one_shot={self._one_shot}).'
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        """Forward a received OccupancyGrid message when the service is available."""
        if self._one_shot and (self._forwarded or self._request_in_flight):
            return

        if not self._client.wait_for_service(timeout_sec=self._wait_for_service_timeout):
            self.get_logger().warn(
                'container_occupancy_grid_registration not available yet; retrying on next '
                'grid message.'
            )
            return

        req = ContainerOccupancyGridRegistration.Request()
        req.grid_map = msg
        req.container_height = self._container_inner_size_z

        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)
        self._request_in_flight = True

        self.get_logger().info(
            f'Forwarded OccupancyGrid '
            f'({msg.info.width}x{msg.info.height} cells @ '
            f'{msg.info.resolution:.3f} m/cell, '
            f'frame={msg.header.frame_id}) to container_occupancy_grid_registration.'
        )

    def _on_response(self, future) -> None:
        """Handle the response from the container occupancy grid registration service."""
        self._request_in_flight = False
        try:
            res = future.result()
        except Exception as exc:
            self.get_logger().error(f'container_occupancy_grid_registration call failed: {exc}')
            return

        if res.success:
            self._forwarded = True
            self.get_logger().info(f'container_occupancy_grid_registration OK: {res.message}')
            if self._one_shot:
                self.get_logger().info('one_shot=true; shutting down.')
                raise SystemExit
        else:
            self.get_logger().error(
                f'container_occupancy_grid_registration returned failure: {res.message}'
            )

    def _bool_parameter(self, name: str) -> bool:
        """Return a bool parameter after rejecting non-bool values."""
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError(f"Parameter '{name}' must be a bool")
        return value

    def _non_negative_float_parameter(self, name: str) -> float:
        """Return a finite numeric parameter greater than or equal to zero."""
        value = self._float_parameter(name)
        if value < 0.0:
            raise ValueError(f"Parameter '{name}' must be greater than or equal to 0")
        return value

    def _positive_float_parameter(self, name: str) -> float:
        """Return a numeric parameter after requiring a finite value greater than zero."""
        value = self._float_parameter(name)
        if value <= 0.0:
            raise ValueError(f"Parameter '{name}' must be greater than 0")
        return value

    def _float_parameter(self, name: str) -> float:
        """Return a numeric parameter after rejecting bool and non-finite values."""
        value = self.get_parameter(name).value

        if isinstance(value, bool):
            raise ValueError(f"Parameter '{name}' must be numeric")

        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Parameter '{name}' must be numeric") from exc

        if not math.isfinite(number):
            raise ValueError(f"Parameter '{name}' must be finite")

        return number


def main(args=None):
    """Initialize ROS, run the forwarder node, and shut it down cleanly."""
    rclpy.init(args=args)
    node = ContainerOccupancyGridForwarder()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
