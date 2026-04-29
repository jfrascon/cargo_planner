#!/usr/bin/env python3
"""
This is an utility node, not meant to be used in production, aimed to test the `cargo_planner` node
without a real robot.

Publish a synthetic OccupancyGrid representing an empty or partially loaded container interior on a
ROS topic.

Parameters
----------
  container_length [m]  Length of the container interior (X axis).  Default: 12.01
  container_width  [m]  Width of the container interior (Y axis).   Default:  2.33
  resolution       [m]  Grid cell size.                             Default:  0.05
  container_frame       TF frame of the container interior.         Default: "container_frame"
  pre_loaded_pallets    Number of EUR pallets to simulate as        Default:  0
                        already loaded at the back of the container.

Default inner container dimensions are based on a standard 40 ft container
(length = 12.01 m, width = 2.33 m).

Usage
-----
  ros2 run cargo_planner synthetic_container_occupancy_grid_publisher.py --ros-args \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid

  Publish a longer container with pre-loaded pallets:
  ros2 run cargo_planner synthetic_container_occupancy_grid_publisher.py --ros-args \
    -p container_length:=13.6 \
    -p pre_loaded_pallets:=3 \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid

  Forward that topic to the `cargo_planner` node:
  ros2 run cargo_planner container_occupancy_grid_forwarder.py --ros-args \
    -r container_occupancy_grid:=myrobot/container_occupancy_grid \
    -r container_occupancy_grid_registration:=cargo_planner/container_occupancy_grid_registration
"""

import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class SyntheticContainerOccupancyGridPublisher(Node):
    """Publishes one synthetic OccupancyGrid for tests and demos."""

    def __init__(self):
        super().__init__('synthetic_container_occupancy_grid_publisher')

        self.declare_parameter('container_length', 12.01)
        self.declare_parameter('container_width', 2.33)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('container_frame', 'container_frame')
        self.declare_parameter('pre_loaded_pallets', 0)

        self._length = self.get_parameter('container_length').value
        self._width = self.get_parameter('container_width').value
        self._res = self.get_parameter('resolution').value
        self._frame = self.get_parameter('container_frame').value
        self._pre_loaded = self.get_parameter('pre_loaded_pallets').value

        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(OccupancyGrid, 'container_occupancy_grid', latch_qos)

        self._grid = self._build_grid()
        self._pub.publish(self._grid)
        self.get_logger().info(
            f'Published synthetic grid {self._nx}x{self._ny} cells '
            f'({self._length:.1f}x{self._width:.1f} m @ {self._res} m/cell).'
        )

    def _build_grid(self) -> OccupancyGrid:
        """
        Build a synthetic OccupancyGrid for a container interior.

        Free cells are represented as 0, occupied cells as 100. Pre-loaded
        pallets are marked as occupied near the back of the container.
        """
        res = self._res
        self._nx = max(1, math.ceil(self._length / res))
        self._ny = max(1, math.ceil(self._width / res))

        data = [0] * (self._nx * self._ny)

        if self._pre_loaded > 0:
            pallet_lp = 1.2
            pallet_wp = 0.8
            pallet_cols = math.ceil(pallet_lp / res)
            pallet_rows = math.ceil(pallet_wp / res)

            x_start_col = self._nx - pallet_cols

            for i in range(self._pre_loaded):
                y_start_row = i * pallet_rows

                if y_start_row + pallet_rows > self._ny:
                    self.get_logger().warn(
                        f'pre_loaded_pallets={self._pre_loaded} exceeds container width. Only {i} pallets simulated.'
                    )
                    break

                for row in range(y_start_row, y_start_row + pallet_rows):
                    for col in range(x_start_col, self._nx):
                        idx = row * self._nx + col
                        data[idx] = 100

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._frame
        grid.info = MapMetaData()
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


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticContainerOccupancyGridPublisher()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
