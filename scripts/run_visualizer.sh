#!/usr/bin/env bash
set -euo pipefail

# Wrapper to replay the cargo_planner visualizer and write PNG figures.
# Edit the defaults below for repeated test sessions.

PACKAGE_NAME="cargo_planner"

# Default values for repeated runs.
DEFAULT_OCCUPANCY_GRID_TOPIC="container_occupancy_grid"
DEFAULT_PALLETS_YAML_FILE="$(ros2 pkg prefix "${PACKAGE_NAME}")/share/${PACKAGE_NAME}/scripts/example_visualizer_pallets.yaml"
DEFAULT_OUTPUT_DIR="tmp/cargo_planner_visualizer"
DEFAULT_CONTAINER_HEIGHT="2.38"
DEFAULT_OCCUPIED_THRESHOLD="65"
DEFAULT_PALLET_MARGIN="0.05"
DEFAULT_ENABLE_ROTATION="true"
DEFAULT_ONE_SHOT="true"
DEFAULT_IMAGE_SCALE="8"

if [[ $# -gt 9 ]]; then
  echo "Usage: $0 [occupancy_grid_topic] [output_dir] [pallets_yaml_file] [container_height] [occupied_threshold] [pallet_margin] [enable_rotation] [one_shot] [image_scale]" >&2
  exit 1
fi

OCCUPANCY_GRID_TOPIC="${1:-${DEFAULT_OCCUPANCY_GRID_TOPIC}}"
OUTPUT_DIR="${2:-${DEFAULT_OUTPUT_DIR}}"
PALLETS_YAML_FILE="${3:-${DEFAULT_PALLETS_YAML_FILE}}"
CONTAINER_HEIGHT="${4:-${DEFAULT_CONTAINER_HEIGHT}}"
OCCUPIED_THRESHOLD="${5:-${DEFAULT_OCCUPIED_THRESHOLD}}"
PALLET_MARGIN="${6:-${DEFAULT_PALLET_MARGIN}}"
ENABLE_ROTATION="${7:-${DEFAULT_ENABLE_ROTATION}}"
ONE_SHOT="${8:-${DEFAULT_ONE_SHOT}}"
IMAGE_SCALE="${9:-${DEFAULT_IMAGE_SCALE}}"

exec ros2 run "${PACKAGE_NAME}" cargo_planner_visualizer.py \
  --ros-args \
  -p occupancy_grid_topic:="${OCCUPANCY_GRID_TOPIC}" \
  -p pallets_yaml_file:="${PALLETS_YAML_FILE}" \
  -p output_dir:="${OUTPUT_DIR}" \
  -p container_height:="${CONTAINER_HEIGHT}" \
  -p occupied_threshold:="${OCCUPIED_THRESHOLD}" \
  -p pallet_margin:="${PALLET_MARGIN}" \
  -p enable_rotation:="${ENABLE_ROTATION}" \
  -p one_shot:="${ONE_SHOT}" \
  -p image_scale:="${IMAGE_SCALE}"
