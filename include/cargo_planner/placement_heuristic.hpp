#pragma once

/**
 * @file placement_heuristic.hpp
 * @brief Abstract interface for the placement heuristic (layer 2).
 *
 * The cargo_planner algorithm is divided into two independent layers:
 *
 *   Layer 1 – CV collision layer (cargo_planner.hpp):
 *     Applies cv::erode with a rectangular kernel equal to the pallet
 *     footprint.  The result is a binary image where each white pixel
 *     is a valid anchor position (the pallet fits there without collision).
 *
 *   Layer 2 – Selection heuristic (this interface):
 *     Given the valid-anchor image from layer 1, decides WHICH anchor
 *     to use.  Different strategies (greedy, MaxRects, ...) are interchangeable
 *     because they all implement this interface.
 *
 * Interface contract:
 *   cv::Point anchor = heuristic->select(valid_anchors, prefer_right_wall);
 *   if (anchor.x < 0) { ... no valid position found ... }
 *
 * The returned cv::Point uses OpenCV conventions: Point(x=col, y=row).
 * In truck_frame: col = X axis (door to back wall), row = Y axis (right to left wall).
 */

#include <opencv2/core.hpp>

namespace cargo_planner
{
  /**
   * @brief Abstract base class for placement selection heuristics.
   *
   * Implementations receive a valid-anchor image (output of cv::erode) and
   * return the best anchor position according to their strategy.
   */
  class PlacementHeuristic
  {
    public:
      virtual ~PlacementHeuristic() = default;

      /**
       * @brief Select the best anchor from a valid-anchor image.
       *
       * @param valid_anchors     Binary cv::Mat (255 = valid, 0 = invalid).
       *                          White pixels are positions where the pallet
       *                          fits without collision.
       * @param prefer_right_wall If true, prefer anchors near the right wall
       *                          (minimum Y = minimum row).  If false, prefer
       *                          anchors near the left wall (maximum Y = maximum row).
       *                          This flag alternates between consecutive pallets
       *                          to implement the wall-hugging strategy.
       * @return The selected anchor as cv::Point(col, row), or cv::Point(-1, -1)
       *         if no valid anchor exists in @p valid_anchors.
       */
      virtual cv::Point select(const cv::Mat& valid_anchors, bool prefer_right_wall) const = 0;
  };

}  // namespace cargo_planner
