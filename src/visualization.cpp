/**
 * @file visualization.cpp
 * @brief RViz MarkerArray generation for the cargo placement plan.
 */

#include "cargo_planner/visualization.hpp"

#include <array>
#include <cmath>

#include <visualization_msgs/msg/marker.hpp>

namespace cargo_planner
{

  namespace
  {

    // Fixed colour palette for cargo markers.  Cycles when more cargo units
    // than colours are placed.  Values are (R, G, B) in [0, 1].
    constexpr std::array<std::array<float, 3>, 7> PALETTE = {{
      {0.20f, 0.80f, 0.20f},  // green
      {0.20f, 0.20f, 0.80f},  // blue
      {0.90f, 0.70f, 0.10f},  // amber
      {0.80f, 0.20f, 0.80f},  // magenta
      {0.10f, 0.80f, 0.80f},  // cyan
      {0.90f, 0.40f, 0.10f},  // orange
      {0.70f, 0.50f, 0.90f},  // lavender
    }};

  }  // namespace

  visualization_msgs::msg::MarkerArray createPlacementMarkers(const std::vector<PlacementResult>& placements,
                                                              double resolution,
                                                              const std::string& frame_id,
                                                              const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray array;
    array.markers.reserve(placements.size());

    for(std::size_t i = 0; i < placements.size(); ++i)
    {
      const PlacementResult& p = placements[i];

      // Centre of the cargo unit in truck_frame [m]
      const double x_center = p.anchor_col * resolution + p.effective_lx / 2.0;
      const double y_center = p.anchor_row * resolution + p.effective_ly / 2.0;
      const double z_center = p.lz / 2.0;

      // Yaw quaternion: 0° → identity; 90° → rotation around Z
      const double yaw = p.rotated ? (M_PI / 2.0) : 0.0;
      const double qz  = std::sin(yaw / 2.0);
      const double qw  = std::cos(yaw / 2.0);

      const auto& color = PALETTE[i % PALETTE.size()];

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp    = stamp;
      marker.ns              = "cargo_plan";
      marker.id              = static_cast<int>(i);
      marker.type            = visualization_msgs::msg::Marker::CUBE;
      marker.action          = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x    = x_center;
      marker.pose.position.y    = y_center;
      marker.pose.position.z    = z_center;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = qz;
      marker.pose.orientation.w = qw;

      // Scale = physical cargo dimensions (not rotated dimensions — the
      // orientation quaternion already handles the visual rotation).
      marker.scale.x = p.rotated ? p.effective_ly : p.effective_lx;  // extent in X
      marker.scale.y = p.rotated ? p.effective_lx : p.effective_ly;  // extent in Y
      marker.scale.z = p.lz;

      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = 0.75f;

      marker.lifetime = rclcpp::Duration(0, 0);  // persistent

      array.markers.push_back(std::move(marker));
    }

    return array;
  }

  visualization_msgs::msg::MarkerArray createDeleteAllMarkers(const std::string& frame_id, const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id;
    del.header.stamp    = stamp;
    del.action          = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(std::move(del));
    return array;
  }

}  // namespace cargo_planner
