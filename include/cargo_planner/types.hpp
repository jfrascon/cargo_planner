#pragma once

/**
 * @file types.hpp
 * @brief Core data structures for the cargo_planner algorithm.
 *
 * These types are pure C++ with no ROS dependencies so that the planning
 * logic can be unit-tested without a running ROS environment.
 *
 * Coordinate convention (truck_frame / truck_container_XX_frame):
 *   X axis: door opening (X=0) → back wall (X=LT, toward cab)
 *   Y axis: origin-side wall (Y=0) → far wall (Y=WT)
 *   Z axis: floor (Z=0) → ceiling (Z=HT)
 *
 * The truck_frame origin sits at the bottom corner of the door opening on
 * the Y=0 side.  "Origin-side wall" and "far wall" are preferred over
 * "right" and "left" because those labels depend on the observer's viewing
 * direction (standing at the door looking in vs. top-down screen view).
 */

#include <string>
#include <vector>

namespace cargo_planner
{

/**
 * @brief Physical dimensions of one cargo unit expressed in truck_frame axes.
 *
 * lx and ly define the 2D footprint used by the placement algorithm.
 * lz is carried through to the output 3D pose but does not participate
 * in the 2D collision check.
 */
struct CargoUnit
{
  std::string id;  ///< Unique cargo unit identifier.
  double lx;       ///< Cargo unit length along truck X axis [m] (door → back wall).
  double ly;       ///< Cargo unit width along truck Y axis [m] (right → left wall).
  double lz;       ///< Cargo unit height along truck Z axis [m] (load included).

  /** @brief Footprint area [m²]. Used internally for 2D collision checks. */
  double area() const
  {
    return lx * ly;
  }

  /**
   * @brief Volume [m³].
   *
   * Used as the primary sort key: cargo units with the largest volume are
   * placed first (deepest in the truck, near the cab), mirroring real forklift
   * practice where the heaviest and bulkiest loads go to the back.
   */
  double volume() const
  {
    return lx * ly * lz;
  }
};

/**
 * @brief Placement result for one cargo unit expressed in grid coordinates.
 *
 * The anchor is the grid cell at the minimum-X, minimum-Y corner of the
 * placed cargo footprint (i.e. the cell closest to the door and the right
 * wall).  The centre pose in metres can be derived as:
 *   x_center = anchor_col * resolution + effective_lx / 2
 *   y_center = anchor_row * resolution + effective_ly / 2
 *   z_center = lz / 2
 *
 * effective_lx and effective_ly reflect the chosen orientation:
 *   rotated == false  →  effective_lx = lx,  effective_ly = ly
 *   rotated == true   →  effective_lx = ly,  effective_ly = lx
 */
struct PlacementResult
{
  std::string id;            ///< Cargo unit identifier, matches CargoUnit::id.
  int anchor_col{0};         ///< Grid column of the anchor (X direction, col=0 → door).
  int anchor_row{0};         ///< Grid row    of the anchor (Y direction, row=0 → right wall).
  bool rotated{false};       ///< True if the cargo unit is placed at 90° yaw.
  double effective_lx{0.0};  ///< Placed cargo extent along X after rotation [m].
  double effective_ly{0.0};  ///< Placed cargo extent along Y after rotation [m].
  double lz{0.0};            ///< Cargo unit height [m], forwarded to the 3D pose.
};

}  // namespace cargo_planner
