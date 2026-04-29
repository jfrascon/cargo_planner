#pragma once

/**
 * @file visualization.hpp
 * @brief RViz MarkerArray generation for the cargo placement plan.
 *
 * Creates one CUBE marker per placed cargo unit.  Markers are coloured using a
 * fixed palette that cycles when more cargo units than colours are placed.
 * The marker namespace is "cargo_plan"; IDs start at 0 and match the index
 * in the placements vector.
 *
 * Coordinate convention: all markers are in truck_frame.
 */

#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cargo_planner/types.hpp"

namespace cargo_planner
{

  /**
   * @brief Build a MarkerArray of cuboids representing the cargo placement plan.
   *
   * Each marker is a CUBE with:
   *   - scale = (effective_lx, effective_ly, lz)     (physical cargo dimensions)
   *   - pose  = centre of the cargo unit in truck_frame
   *   - color = cycling palette, alpha = 0.75
   *   - lifetime = 0 (persistent until replaced)
   *
   * @param placements  Result of CargoPlanner::plan().
   * @param resolution  Grid resolution used during planning [m/cell].
   * @param frame_id    TF frame for the markers (typically "truck_frame").
   * @param stamp       Timestamp to attach to all markers.
   * @return MarkerArray ready to publish on ~/cargo_markers.
   */
  visualization_msgs::msg::MarkerArray createPlacementMarkers(const std::vector<PlacementResult>& placements,
                                                              double resolution,
                                                              const std::string& frame_id,
                                                              const rclcpp::Time& stamp);

  /**
   * @brief Build a single DELETE_ALL MarkerArray to clear previous markers.
   *
   * @param frame_id  TF frame (must match the frame used when adding markers).
   * @param stamp     Timestamp.
   * @return MarkerArray with one DELETE_ALL action marker.
   */
  visualization_msgs::msg::MarkerArray createDeleteAllMarkers(const std::string& frame_id, const rclcpp::Time& stamp);

}  // namespace cargo_planner
