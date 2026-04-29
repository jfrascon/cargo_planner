#!/usr/bin/env python3
"""
This script is an utility node, not meant to be used in production, aimed to test the
`cargo_planner` node without a real robot.

Forward the OccupancyGrid topic representing the container interior to the
`container_occupancy_grid_registration` service.

The node subscribes to container_occupancy_grid, wraps each received grid in a
ContainerOccupancyGridRegistration request, sets container_height, and calls
container_occupancy_grid_registration.

Parameters
----------
  container_height [m]       Container interior height passed to the service.
                             Default: 2.38
  wait_for_service_timeout [s]  Time to wait for the planner service after a
                                grid arrives. Default: 2.0
  one_shot                   If true, forward only the first received grid and
                             then shut down. If false, forward every grid.
                             Default: true

Usage
-----
  Forward the first grid and exit:
  ros2 run cargo_planner container_occupancy_grid_forwarder.py --ros-args \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid \
    -r container_occupancy_grid_registration:=cargo_planner/container_occupancy_grid_registration

  Keep running and forward every new grid:
  ros2 run cargo_planner container_occupancy_grid_forwarder.py --ros-args \
    -p one_shot:=false \
    -p container_height:=2.38 \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid \
    -r container_occupancy_grid_registration:=cargo_planner/container_occupancy_grid_registration
"""

import rclpy
from cargo_planner_msgs.srv import ContainerOccupancyGridRegistration
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class ContainerOccupancyGridForwarder(Node):
    """Forwards container_occupancy_grid messages to container_occupancy_grid_registration."""

    def __init__(self):
        super().__init__('container_occupancy_grid_forwarder')

        self.declare_parameter('container_height', 2.38)
        self.declare_parameter('wait_for_service_timeout', 2.0)
        self.declare_parameter('one_shot', True)

        self._container_height = self.get_parameter('container_height').value
        self._wait_for_service_timeout = self.get_parameter('wait_for_service_timeout').value
        self._one_shot = self.get_parameter('one_shot').value
        self._forwarded = False

        self._client = self.create_client(
            ContainerOccupancyGridRegistration, 'container_occupancy_grid_registration'
        )

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        self._sub = self.create_subscription(OccupancyGrid, 'container_occupancy_grid', self._on_grid, qos)

        self.get_logger().info(
            'Waiting for container_occupancy_grid to forward to '
            f'container_occupancy_grid_registration (one_shot={self._one_shot}).'
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        """Callback for received OccupancyGrid messages."""
        if self._one_shot and self._forwarded:
            return

        if not self._client.wait_for_service(timeout_sec=self._wait_for_service_timeout):
            self.get_logger().warn(
                'container_occupancy_grid_registration not available yet; retrying on next grid message.'
            )
            return

        req = ContainerOccupancyGridRegistration.Request()
        req.grid_map = msg
        req.container_height = self._container_height

        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)
        self._forwarded = True

        self.get_logger().info(
            f'Forwarded OccupancyGrid '
            f'({msg.info.width}x{msg.info.height} cells @ {msg.info.resolution:.3f} m/cell, '
            f'frame={msg.header.frame_id}) to container_occupancy_grid_registration.'
        )

    def _on_response(self, future) -> None:
        """Callback for the container_occupancy_grid_registration service response."""
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'container_occupancy_grid_registration call failed: {e}')
            return

        if res.success:
            self.get_logger().info(f'container_occupancy_grid_registration OK: {res.message}')
            if self._one_shot:
                self.get_logger().info('one_shot=true; shutting down.')
                raise SystemExit
        else:
            self.get_logger().error(f'container_occupancy_grid_registration returned failure: {res.message}')


def main(args=None):
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
