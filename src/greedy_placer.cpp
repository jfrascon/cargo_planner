/**
 * @file greedy_placer.cpp
 * @brief Implementation of the greedy wall-hugging placement heuristic.
 */

#include "cargo_planner/greedy_placer.hpp"

namespace cargo_planner
{

  cv::Point GreedyPlacer::select(const cv::Mat& valid_anchors, bool prefer_right_wall) const
  {
    // Iterate columns from the back of the truck (highest col = highest X) toward
    // the door (col = 0).  This ensures the truck is filled from back to front.
    for(int col = valid_anchors.cols - 1; col >= 0; --col)
    {
      if(prefer_right_wall)
      {
        // Search for the minimum row (closest to Y=0, right wall) in this column.
        // A pallet anchored at row=0 will have its left edge at Y=0 (right wall).
        for(int row = 0; row < valid_anchors.rows; ++row)
        {
          if(valid_anchors.at<uint8_t>(row, col) != 0)
          {
            return cv::Point(col, row);  // cv::Point(x=col, y=row)
          }
        }
      }
      else
      {
        // Search for the maximum row (closest to Y=WT, left wall) in this column.
        // A pallet anchored at the maximum valid row will have its right edge as
        // close to the left wall as possible.
        for(int row = valid_anchors.rows - 1; row >= 0; --row)
        {
          if(valid_anchors.at<uint8_t>(row, col) != 0)
          {
            return cv::Point(col, row);
          }
        }
      }
    }

    // No valid anchor found anywhere in the image.
    return cv::Point(-1, -1);
  }

}  // namespace cargo_planner
