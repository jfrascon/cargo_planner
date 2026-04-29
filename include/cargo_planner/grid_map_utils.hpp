#pragma once

/**
 * @file grid_map_utils.hpp
 * @brief Conversion utilities between nav_msgs/OccupancyGrid and cv::Mat.
 *
 * The OccupancyGrid produced by truck_inspector uses the following encoding:
 *   0   = free  (between door and first LiDAR hit in each Y column)
 *   100 = occupied (from first LiDAR hit to back wall)
 *
 * The cv::Mat used internally by the placement algorithm uses:
 *   255 = free
 *   0   = occupied
 *
 * Grid ↔ image index mapping:
 *   mat.at<uint8_t>(row, col)  ↔  grid.data[row * info.width + col]
 *   col = X cell index (0 = door,        nx-1 = near back wall)
 *   row = Y cell index (0 = right wall,  ny-1 = near left wall)
 */

#include <cstdint>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>

namespace cargo_planner
{

  /**
   * @brief Convert an OccupancyGrid to a binary cv::Mat (255 = free, 0 = occupied).
   *
   * Cells with a value in [0, occupied_threshold) are considered free.
   * Cells with a value >= occupied_threshold, or with value -1 (unknown),
   * are considered occupied.
   *
   * The resulting Mat has:
   *   rows = info.height  (Y axis, right wall → left wall)
   *   cols = info.width   (X axis, door → back wall)
   *   type = CV_8UC1
   *
   * @param grid              Source OccupancyGrid in truck_frame.
   * @param occupied_threshold Cells with value >= this threshold are occupied [0..100].
   * @return Binary cv::Mat representing the free space.
   */
  cv::Mat occupancyGridToMat(const nav_msgs::msg::OccupancyGrid& grid, int occupied_threshold = 50);

  /**
   * @brief Convert a binary cv::Mat back to an OccupancyGrid, reusing the
   *        metadata (resolution, origin, frame_id) from a reference grid.
   *
   * White pixels (255) become 0 (free), black pixels (0) become 100 (occupied).
   *
   * @param mat     Binary cv::Mat (CV_8UC1, same dimensions as the reference grid).
   * @param ref     Reference grid whose metadata (info, header) is copied.
   * @return OccupancyGrid with the same resolution and origin as @p ref.
   */
  nav_msgs::msg::OccupancyGrid matToOccupancyGrid(const cv::Mat& mat, const nav_msgs::msg::OccupancyGrid& ref);

}  // namespace cargo_planner
