#pragma once

/**
 * @file cargo_planner.hpp
 * @brief Core 2D cargo placement engine.
 *
 * CargoPlanner is the heart of the package.  It takes a binary free-space
 * image (cv::Mat) and a list of cargo units and computes a placement plan using
 * a two-layer approach:
 *
 *   Layer 1 – CV collision layer:
 *     For each cargo unit, applies cv::erode with a rectangular kernel equal
 *     to the cargo footprint.  The result is a binary image where each white
 *     pixel is a valid anchor position (the cargo unit fits there entirely).
 *
 *   Layer 2 – Selection heuristic (PlacementHeuristic):
 *     Chooses the best anchor from the valid-anchor image.  The default
 *     implementation is GreedyPlacer (wall-hugging, back-to-front).
 *     The heuristic is injected at construction time so it can be swapped
 *     without changing any other code.
 *
 * This class has no ROS dependencies and can be tested independently.
 * CargoPlacerNode owns the ROS interfaces and converts their data into these core types.
 */

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "cargo_planner/placement_heuristic.hpp"
#include "cargo_planner/types.hpp"

namespace cargo_planner
{

  /**
   * @brief 2D cargo placement engine.
   *
   * Typical usage:
   * @code
   *   CargoPlanner planner(std::make_unique<GreedyPlacer>());
   *   planner.setFreeSpace(free_mat, resolution, container_height);
   *   planner.setCargoUnits(cargo_units);
   *   auto result = planner.plan();
   * @endcode
   */
  class CargoPlanner
  {
    public:
      /** @brief Runtime configuration knobs. */
      struct Config
      {
          /// Safety clearance added around every obstacle before cargo placement [m].
          /// Implemented as a pre-erosion of the free-space image. Set to 0.0 to disable.
          double pallet_margin{0.05};
          /// If true, the planner may also test each cargo unit at 90 degrees yaw.
          /// If false, only the natural orientation (lx along X, ly along Y) is used.
          bool enable_rotation{true};
      };

      /** @brief Result returned by plan(). */
      struct Result
      {
          std::vector<PlacementResult> placements;  ///< Successfully placed cargo units.
          std::vector<std::string> no_placed;       ///< IDs of cargo units that did not fit.
          double placed_area_m2{0.0};
          double free_area_m2{0.0};
          double total_area_m2{0.0};
          double utilization_pct{0.0};
          cv::Mat final_work_map;  ///< Free-space image after all placements.
      };

      /**
       * @brief Construct the planner with a placement heuristic and default configuration.
       *
       * @param heuristic Layer-2 heuristic (e.g. GreedyPlacer).  Ownership is transferred.
       */
      explicit CargoPlanner(std::unique_ptr<PlacementHeuristic> heuristic);

      /**
       * @brief Construct the planner with a placement heuristic and explicit configuration.
       *
       * @param heuristic Layer-2 heuristic (e.g. GreedyPlacer).  Ownership is transferred.
       * @param cfg       Runtime configuration.
       */
      CargoPlanner(std::unique_ptr<PlacementHeuristic> heuristic, Config cfg);

      /**
       * @brief Provide the free-space map of the truck interior.
       *
       * @param free_space  Binary cv::Mat (255 = free, 0 = occupied, CV_8UC1).
       *                    rows = Y cells (right wall → left wall),
       *                    cols = X cells (door → back wall).
       * @param resolution  Cell size in metres.
       * @param container_height Positive finite container interior height [m].
       *                         Cargo units taller than this value are not placed.
       */
      void setFreeSpace(cv::Mat free_space, double resolution, double container_height);

      /**
       * @brief Provide the list of cargo units to be placed.
       *
       * Cargo units are sorted internally by volume descending before planning.
       *
       * @param cargo_units List of cargo units to load.
       */
      void setCargoUnits(std::vector<CargoUnit> cargo_units);

      /**
       * @brief Run the placement algorithm and return the plan.
       *
       * May be called multiple times with the same state (e.g. for re-planning).
       * Always starts from the last free-space image set by setFreeSpace().
       *
       * @return Result containing placed cargo units, unplaced IDs and area metrics.
       * @throws std::runtime_error if either required input has not been provided.
       */
      Result plan() const;

      /** @return True if both a free-space map and cargo units have been provided. */
      bool ready() const;

    private:
      /**
       * @brief Apply cv::erode to find all valid anchor positions for a cargo unit.
       *
       * @param work_map Binary free-space image (modified in-place by callers after this call).
       * @param krows    Kernel height (cargo extent in Y, in cells).
       * @param kcols    Kernel width (cargo extent in X, in cells).
       * @return Binary image (255 = valid anchor, 0 = invalid).
       */
      static cv::Mat computeValidAnchors(const cv::Mat& work_map, int krows, int kcols);

      /**
       * @brief Mark the footprint of a placed cargo unit as occupied in @p work_map.
       *
       * The occupied rectangle is expanded by @p margin_cells on every side so
       * that subsequent cargo units maintain clearance from this one.  This complements
       * the pre-erosion that provides clearance from walls and static obstacles.
       *
       * @param work_map     Binary free-space image to update.
       * @param anchor       Top-left corner of the cargo unit in grid coordinates.
       * @param krows        Cargo unit height in cells.
       * @param kcols        Cargo unit width in cells.
       * @param margin_cells Safety clearance in cells (0 = no extra margin).
       */
      static void markOccupied(cv::Mat& work_map, cv::Point anchor, int krows, int kcols, int margin_cells);

      /**
       * @brief Convert a physical cargo dimension to an integer number of grid cells.
       *
       * Uses ceiling to ensure the cell count always covers the full physical extent.
       *
       * @param metres     Physical dimension [m].
       * @param resolution Grid resolution [m/cell].
       * @return Zero when metres is zero; otherwise, a positive cell count.
       */
      static int toCells(double metres, double resolution);

      std::unique_ptr<PlacementHeuristic> heuristic_;
      Config cfg_;

      cv::Mat free_space_;  ///< Owned binary free-space image copied by setFreeSpace().
      double resolution_{0.05};
      double container_height_{0.0};
      std::vector<CargoUnit> cargo_units_;  ///< Sorted by volume descending.

      bool has_map_{false};
      bool has_cargo_units_{false};
  };

}  // namespace cargo_planner
