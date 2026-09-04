# cargo_planner

`cargo_planner` computes greedy 2D placements for rectangular cargo units inside a container
occupancy grid.
It provides a C++ planning library, a ROS 2 node, visualization output and development tools for
creating synthetic inputs and algorithm figures.

The planner is deterministic but not globally optimal.
It places larger-volume cargo first and selects a deep wall-adjacent position for each unit.

## Runtime data flow

```text
ContainerOccupancyGridRegistration ─┐
                                    ├─> stored planner state ─> PlanCargo action
CargoListRegistration ──────────────┘                             |
                                                                  ├─ placements
                                                                  ├─ cargo_markers
                                                                  └─ container_free_map
```

The two registration services replace stored state.
They may be called in either order, but both must succeed before a planning goal can execute.

## ROS 2 interfaces

| Name | Type | Responsibility |
| --- | --- | --- |
| `container_occupancy_grid_registration` | `cargo_planner_msgs/srv/ContainerOccupancyGridRegistration` | Store the container map and height. |
| `cargo_list_registration` | `cargo_planner_msgs/srv/CargoListRegistration` | Validate and store the cargo list. |
| `cargo_planning` | `cargo_planner_msgs/action/PlanCargo` | Compute one plan from stored state. |
| `cargo_markers` | `visualization_msgs/msg/MarkerArray` | Publish RViz cuboids for successful placements. |
| `container_free_map` | `nav_msgs/msg/OccupancyGrid` | Publish the remaining usable grid. |

A second action goal is rejected while one plan is running.
Cancellation is checked before and after the non-interruptible planning step.
The planning thread retains shared ownership of the node until it finishes.

The node snapshots the occupancy grid used by each plan.
A later registration cannot make result poses or the free-map output use metadata from a different
grid.

## Validation

The planner rejects:

- Occupancy thresholds outside `[0, 100]`.
- Empty maps, non-binary core maps and inconsistent occupancy-grid data sizes.
- Non-positive or non-finite grid resolution and container height.
- Negative or non-finite pallet margin.
- Empty cargo lists at the service boundary.
- Blank or duplicate cargo IDs.
- Non-positive or non-finite cargo dimensions.

A valid cargo unit with `lz` greater than the container height is reported in `no_placed`.
It is not a malformed request; it is a valid unit that does not fit.

## Coordinate convention

The occupancy-grid header identifies the frame used by all output poses and markers.

Within that frame:

- X follows grid columns from the door toward the back wall.
- Y follows grid rows from the origin-side wall toward the opposite wall.
- Z starts at the floor and points upward.

The planner includes a non-zero `OccupancyGrid.info.origin` when converting grid anchors to output
poses.
A `CargoPlacement` has no header, so consumers must retain the registered grid frame as context.

For an unrotated cargo unit, `lx` follows container X and `ly` follows container Y.
For a rotated unit, the planner swaps those footprint extents and returns a `pi / 2` yaw.
The output Z coordinate is `lz / 2`.

## Planning algorithm

The algorithm has two layers.

### Collision layer

The input occupancy grid is converted to a binary OpenCV image:

- 255 represents free space.
- 0 represents occupied or unknown space.

The planner erodes free space by the configured pallet margin.
For each cargo orientation, another rectangular erosion identifies every top-left anchor where the
footprint fits completely.

After placing a unit, its footprint plus margin is marked occupied.
Later units therefore keep clearance from static obstacles and previously placed cargo.

### Selection layer

`GreedyPlacer` scans anchors from the back of the container toward the door.
Successive successful placements alternate between the two lateral walls.
When both orientations fit, the planner selects the orientation whose anchor lies deeper in the
container and uses zero yaw to break a tie.

Cargo units are sorted by descending `lx * ly * lz` before planning.
The input service order is not preserved.

## Area metrics

The result reports:

- `placed_area_m2`: physical footprint area of successful placements.
- `free_area_m2`: remaining usable grid area after safety margins and placements.
- `total_area_m2`: complete rectangular occupancy-grid area.
- `utilization_pct`: `placed_area_m2 / total_area_m2 * 100`.

The denominator includes cells that were initially occupied.
The metric therefore measures utilization of the complete grid rectangle, not only its initially
free cells.

## Launch contract

`cargo_planner.launch.py` exposes:

| Argument | Default | Responsibility |
| --- | --- | --- |
| `namespace` | `cargo_planner` | Namespace where the node is launched. |
| `params_file` | installed default YAML | Complete functional configuration. |
| `params_file_allow_substs` | `False` | Allow ROS launch substitutions in the YAML file. |
| `use_sim_time` | `False` | Select the ROS simulation clock. |
| `node_args` | standard JSON | Configure supported `launch_ros.actions.Node` arguments. |

The standard node arguments are:

```json
{"output":"both","ros_arguments":["--log-level","info"]}
```

The YAML file owns all functional parameters.
`use_sim_time` is appended afterward because clock selection belongs to the launch environment.

The obsolete `occupied_threshold`, `pallet_margin`, `enable_rotation`, `node_remappings`,
`node_options` and `node_logging_options` launch arguments are no longer supported.
Edit or replace the YAML for functional changes and use `node_args` for action behavior.

## Parameters

The installed `config/default_cargo_planner.yaml` defines:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `occupied_threshold` | `65` | Grid values at or above this value are occupied. |
| `pallet_margin` | `0.05` | Non-negative safety clearance in meters. |
| `enable_rotation` | `true` | Test both zero and 90-degree yaw. |

Start the planner:

```bash
ros2 launch cargo_planner cargo_planner.launch.py
```

Use another configuration:

```bash
ros2 launch cargo_planner cargo_planner.launch.py \
  namespace:=cargo_plan \
  params_file:=/absolute/path/to/cargo_planner.yaml
```

## Example call sequence

Publish a synthetic container grid:

```bash
ros2 run cargo_planner synthetic_container_occupancy_grid_publisher.py \
  --ros-args \
  -p container_frame:=container_frame \
  -p container_inner_size_x:=13.6 \
  -p container_inner_size_y:=2.4 \
  -r container_occupancy_grid:=container_inspector/container_occupancy_grid
```

Forward that grid to the registration service:

```bash
ros2 run cargo_planner container_occupancy_grid_forwarder.py \
  --ros-args \
  -p container_inner_size_z:=2.38 \
  -r container_occupancy_grid:=container_inspector/container_occupancy_grid \
  -r container_occupancy_grid_registration:=cargo_plan/container_occupancy_grid_registration
```

Register cargo:

```bash
ros2 service call /cargo_plan/cargo_list_registration \
  cargo_planner_msgs/srv/CargoListRegistration \
  "{cargo_units: [{id: 'c1', lx: 1.2, ly: 0.8, lz: 1.5}, {id: 'c2', lx: 1.2, ly: 0.8, lz: 1.5}]}"
```

Request a plan:

```bash
ros2 action send_goal /cargo_plan/cargo_planning cargo_planner_msgs/action/PlanCargo "{}"
```

## Development tools

`synthetic_container_occupancy_grid_publisher.py` publishes a configurable test grid.
Its optional pre-loaded cargo file uses the structure in
`config/example_pre_loaded_cargo_units.yaml`.

`container_occupancy_grid_forwarder.py` forwards grids to the registration service.
In one-shot mode it stops after a successful response and retries when a request fails.

`cargo_planner_visualizer.py` replays planning stages and writes explanatory PNG files.
It is a documentation tool, not the production planner.
The visualizer validates cargo units using the same ID and dimension rules as the C++ planner.

## Libraries

`cargo_planner_core` contains the ROS-independent planning algorithm and heuristic.
`cargo_planner_ros_support` contains occupancy-grid conversion and ROS visualization helpers.
Both targets are exported for downstream CMake consumers.
The executable links both libraries and owns services, action handling and publishers.

## Build and test

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --symlink-install --packages-select cargo_planner_msgs cargo_planner
source install/setup.bash
colcon test --merge-install --packages-select cargo_planner_msgs cargo_planner
colcon test-result --test-result-base build/cargo_planner_msgs --verbose
colcon test-result --test-result-base build/cargo_planner --verbose
```

## License

This package is distributed under the Apache License 2.0.
See [LICENSE](LICENSE).
