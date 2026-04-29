/**
 * @file cargo_planner.cpp
 * @brief Core 2D cargo placement engine implementation.
 */

#include "cargo_planner/cargo_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace cargo_planner
{

// ── Construction ────────────────────────────────────────────────────────────

CargoPlanner::CargoPlanner(std::unique_ptr<PlacementHeuristic> heuristic): CargoPlanner(std::move(heuristic), Config{})
{}

CargoPlanner::CargoPlanner(std::unique_ptr<PlacementHeuristic> heuristic, Config cfg):
  heuristic_(std::move(heuristic)),
  cfg_(cfg)
{
  if(!heuristic_)
  {
    throw std::invalid_argument("CargoPlanner: heuristic must not be null");
  }
}

// ── State setters ────────────────────────────────────────────────────────────

void CargoPlanner::setFreeSpace(cv::Mat free_space, double resolution, double container_height)
{
  if(free_space.empty())
  {
    throw std::invalid_argument("CargoPlanner::setFreeSpace: free_space is empty");
  }
  if(resolution <= 0.0)
  {
    throw std::invalid_argument("CargoPlanner::setFreeSpace: resolution must be > 0");
  }

  free_space_   = std::move(free_space);
  resolution_   = resolution;
  container_height_ = container_height;
  has_map_      = true;
}

void CargoPlanner::setCargoUnits(std::vector<CargoUnit> cargo_units)
{
  // Sort cargo units from largest to smallest volume (lx × ly × lz).
  // This mirrors the practice of forklift operators who load the bulkiest
  // cargo units at the back of the truck (near the cab) so that the load
  // height decreases toward the door.  Using volume rather than footprint
  // area also naturally deprioritises flat cargo that is wide but short.
  std::sort(cargo_units.begin(), cargo_units.end(), [](const CargoUnit& a, const CargoUnit& b) {
    return a.volume() > b.volume();
  });

  cargo_units_     = std::move(cargo_units);
  has_cargo_units_ = !cargo_units_.empty();
}

bool CargoPlanner::ready() const
{
  return has_map_ && has_cargo_units_;
}

// ── Planning ─────────────────────────────────────────────────────────────────

CargoPlanner::Result CargoPlanner::plan() const
{
  if(!has_map_)
  {
    throw std::runtime_error("CargoPlanner::plan(): no free-space map set");
  }
  if(!has_cargo_units_)
  {
    throw std::runtime_error("CargoPlanner::plan(): no cargo unit list set");
  }

  // work_map is a copy of free_space_ that is modified as cargo units are placed.
  // We start from free_space_ (not a previously modified work_map) so that
  // plan() is idempotent and can be called multiple times.
  cv::Mat work_map = free_space_.clone();

  // The safety margin is applied in two places:
  //
  //   1. Pre-erosion (here): shrinks the free space so that placed cargo units
  //      never touch walls or pre-existing obstacles.
  //
  //   2. markOccupied (per cargo unit): marks footprint + margin so that
  //      subsequent cargo units respect clearance from already-placed cargo.
  //
  // Both steps together guarantee a margin_cells gap on every side of every
  // placed cargo unit relative to both obstacles and other cargo units.
  const int margin_cells = toCells(cfg_.pallet_margin, resolution_);
  if(margin_cells > 0)
  {
    const int ks = 2 * margin_cells + 1;
    cv::erode(work_map, work_map, cv::Mat::ones(ks, ks, CV_8U), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);
  }

  Result result;
  result.total_area_m2 = static_cast<double>(free_space_.cols) * static_cast<double>(free_space_.rows) * resolution_ *
                         resolution_;

  // Count how many cargo units have been placed so far. This is used to alternate
  // the wall-hugging preference (even → right wall, odd → left wall).
  int placed_count = 0;

  for(const auto& cargo_unit: cargo_units_)
  {
    // ── Orientation 0°: lx along X (cols), ly along Y (rows) ──────────────
    const int kcols_0 = toCells(cargo_unit.lx, resolution_);
    const int krows_0 = toCells(cargo_unit.ly, resolution_);

    const bool prefer_right = (placed_count % 2 == 0);

    // Orientation 0 degrees is always considered. The optional 90 degree
    // orientation is controlled by cfg_.enable_rotation so deployments can
    // forbid rotated cargo without changing the message-level cargo dimensions.
    const cv::Mat anchors_0 = computeValidAnchors(work_map, krows_0, kcols_0);
    const cv::Point anchor_0 = heuristic_->select(anchors_0, prefer_right);
    const bool valid_0 = (anchor_0.x >= 0);

    cv::Point anchor_90(-1, -1);
    bool valid_90 = false;
    int kcols_90 = 0;
    int krows_90 = 0;
    if(cfg_.enable_rotation)
    {
      // ── Orientation 90°: ly along X (cols), lx along Y (rows) ──────────
      kcols_90 = toCells(cargo_unit.ly, resolution_);
      krows_90 = toCells(cargo_unit.lx, resolution_);

      const cv::Mat anchors_90 = computeValidAnchors(work_map, krows_90, kcols_90);
      anchor_90 = heuristic_->select(anchors_90, prefer_right);
      valid_90 = (anchor_90.x >= 0);
    }

    if(!valid_0 && !valid_90)
    {
      // Neither orientation fits, so record this cargo unit as unplaced.
      result.no_placed.push_back(cargo_unit.id);
      continue;
    }

    // Choose orientation: prefer the one with higher col (deeper in the truck).
    // If both are equally deep, prefer orientation 0° (natural orientation).
    bool use_90 = false;
    if(valid_0 && valid_90)
    {
      use_90 = (anchor_90.x > anchor_0.x);
    }
    else
    {
      use_90 = valid_90;
    }

    const cv::Point anchor = use_90 ? anchor_90 : anchor_0;
    const int krows        = use_90 ? krows_90 : krows_0;
    const int kcols        = use_90 ? kcols_90 : kcols_0;
    const double eff_lx    = use_90 ? cargo_unit.ly : cargo_unit.lx;
    const double eff_ly    = use_90 ? cargo_unit.lx : cargo_unit.ly;

    // Record the placement.
    PlacementResult placement;
    placement.id           = cargo_unit.id;
    placement.anchor_col   = anchor.x;
    placement.anchor_row   = anchor.y;
    placement.rotated      = use_90;
    placement.effective_lx = eff_lx;
    placement.effective_ly = eff_ly;
    placement.lz           = cargo_unit.lz;
    result.placements.push_back(placement);

    // Mark footprint + margin as occupied so subsequent cargo units respect
    // the safety clearance from this one (see markOccupied for details).
    markOccupied(work_map, anchor, krows, kcols, margin_cells);

    result.placed_area_m2 += eff_lx * eff_ly;
    ++placed_count;
  }

  // Count remaining free cells after all placements.
  const int free_cells   = cv::countNonZero(work_map);
  result.free_area_m2    = static_cast<double>(free_cells) * resolution_ * resolution_;
  result.utilization_pct = (result.total_area_m2 > 0.0) ? (result.placed_area_m2 / result.total_area_m2) * 100.0 : 0.0;
  result.final_work_map  = work_map;

  return result;
}

// ── Private helpers ───────────────────────────────────────────────────────────

cv::Mat CargoPlanner::computeValidAnchors(const cv::Mat& work_map, int krows, int kcols)
{
  if(krows <= 0 || kcols <= 0)
  {
    return cv::Mat::zeros(work_map.size(), CV_8UC1);
  }

  cv::Mat valid_anchors;
  // cv::erode with BORDER_CONSTANT = 0 ensures that any anchor position where
  // the kernel would extend beyond the image boundary is automatically marked
  // as invalid (the cargo unit would fall outside the truck).
  // Anchor at (0,0) (top-left of the kernel) so that the output pixel (r,c)
  // represents "the cargo unit with its top-left corner at (row=r, col=c) fits".
  // This is consistent with how markOccupied() fills the rectangle at anchor.x/y.
  cv::erode(work_map, valid_anchors, cv::Mat::ones(krows, kcols, CV_8U), cv::Point(0, 0), 1, cv::BORDER_CONSTANT, 0);
  return valid_anchors;
}

void CargoPlanner::markOccupied(cv::Mat& work_map, cv::Point anchor, int krows, int kcols, int margin_cells)
{
  // anchor = (col, row) = (x, y) in OpenCV convention.
  // Expand the occupied rectangle by margin_cells on every side so that
  // subsequent cargo units cannot be placed closer than margin_cells to this one.
  const int x0 = anchor.x - margin_cells;
  const int y0 = anchor.y - margin_cells;
  const int w  = kcols + 2 * margin_cells;
  const int h  = krows + 2 * margin_cells;
  const cv::Rect rect(x0, y0, w, h);
  // Clip to image bounds to avoid out-of-range access.
  const cv::Rect safe = rect & cv::Rect(0, 0, work_map.cols, work_map.rows);
  work_map(safe).setTo(0);
}

int CargoPlanner::toCells(double metres, double resolution)
{
  if(metres <= 0.0 || resolution <= 0.0)
  {
    return 0;
  }
  return std::max(1, static_cast<int>(std::ceil(metres / resolution)));
}

}  // namespace cargo_planner
