#pragma once

/**
 * @file greedy_placer.hpp
 * @brief Greedy wall-hugging placement heuristic (layer 2, iteration 1).
 *
 * Strategy
 * --------
 * For each cargo unit the heuristic searches the valid-anchor image from the
 * back of the truck (high X = high col) toward the door (low col).  Within
 * each column it picks either the minimum row (origin-side wall, Y=0) or the
 * maximum row (far wall, Y=WT) depending on the @p prefer_right_wall flag.
 *
 * The effect is:
 *   - Cargo units fill the truck from back to front (maximise occupancy depth).
 *   - Consecutive cargo units hug opposite walls, leaving the gap in the centre.
 *
 * Top-down view — coordinate convention for truck_container_XX_frame:
 *
 *   X+ goes toward the back wall / cab (increasing depth from door).
 *   Y+ goes toward the far lateral wall (increasing lateral offset from the right wall).
 *   Origin (X=0, Y=0) is at the door-side right corner of the container.
 *
 *             ^ X+
 *             |
 *   . . . . . . . . . . . . . .  <- back wall  (X = LT)
 *   .  [cargo B]    [cargo A]   .
 *   .  [cargo B]    [cargo A]   .
 *   .       gap                 .
 *   .                           .
 *   .                           .
 *   . . . . . . . . . . . . . .  <- door       (X = 0)
 *   ^                          ^
 *   far wall (Y = WT)          right wall (Y = 0)  <- origin
 *                              Y+ <-+
 *
 * This mirrors the practice of a real forklift operator who always loads
 * from the walls inward to keep the cargo stable in transit.
 *
 * Upgrading to MaxRects
 * ---------------------
 * To replace this heuristic with a MaxRects-based one (iteration 2), create
 * a new class that inherits from PlacementHeuristic and pass it to
 * CargoPlanner::CargoPlanner().  No other code needs to change.
 */

#include "cargo_planner/placement_heuristic.hpp"

namespace cargo_planner
{

/**
 * @brief Greedy wall-hugging heuristic: back-to-front, wall-to-wall.
 *
 * Time complexity: O(cols x rows) in the worst case.
 * For a 272x48 grid (13.6 m x 2.4 m at 0.05 m/cell) this is ~13 k iterations
 * per cargo unit, which is well within the real-time budget.
 */
class GreedyPlacer final: public PlacementHeuristic
{
  public:
  /**
   * @brief Select the deepest valid anchor, biased toward the indicated wall.
   *
   * Iterates columns from the highest index (back of truck) down to zero.
   * Within the first column that contains at least one valid anchor it picks:
   *   - prefer_right_wall == true  -> minimum row  (Y closest to 0,  right wall)
   *   - prefer_right_wall == false -> maximum row  (Y closest to max, left wall)
   *
   * @param valid_anchors     Binary image (255 = valid) from cv::erode.
   * @param prefer_right_wall Wall-hugging side for this placement.
   * @return Anchor as cv::Point(col, row), or {-1, -1} if the image is empty.
   */
  cv::Point select(const cv::Mat& valid_anchors, bool prefer_right_wall) const override;
};

}  // namespace cargo_planner
