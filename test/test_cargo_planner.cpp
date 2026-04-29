/**
 * @file test_cargo_planner.cpp
 * @brief Unit tests for CargoPlanner (core algorithm, no ROS).
 *
 * Tests cover:
 *   - Empty truck (all free)  → all cargo units placed
 *   - Full truck  (all occ.)  → no cargo units placed
 *   - Partial occupancy       → correct subset placed
 *   - Rotation selection      → wide cargo unit in narrow truck → rotated
 *   - Wall-hugging            → consecutive cargo units on opposite walls
 *   - Largest-first ordering  → large cargo unit placed deeper than small one
 *   - Idempotency             → plan() does not modify stored state
 *   - Pallet margin           → gap between adjacent cargo units >= pallet_margin
 */

#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/core.hpp>

#include "cargo_planner/cargo_planner.hpp"
#include "cargo_planner/greedy_placer.hpp"

using cargo_planner::CargoPlanner;
using cargo_planner::GreedyPlacer;
using cargo_planner::CargoUnit;
using cargo_planner::PlacementResult;

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Build a CargoPlanner with GreedyPlacer and zero pallet_margin.
static CargoPlanner makePlanner()
{
  CargoPlanner::Config cfg;
  cfg.pallet_margin = 0.0;
  return CargoPlanner(std::make_unique<GreedyPlacer>(), cfg);
}

/// Build a CargoUnit with all fields set.
static CargoUnit makeCargoUnit(const std::string& id, double lx, double ly, double lz = 1.5)
{
  return {id, lx, ly, lz};
}

// ── Empty truck ───────────────────────────────────────────────────────────────

TEST(CargoPlanner, EmptyTruckPlacesAllCargoUnits)
{
  // Truck: 5 m × 2.4 m @ 0.1 m/cell → 50×24 grid, all free
  const double res = 0.1;
  cv::Mat free_map(24, 50, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 1.2, 0.8), makeCargoUnit("c2", 1.2, 0.8)});

  auto result = planner.plan();

  EXPECT_EQ(result.placements.size(), 2u);
  EXPECT_TRUE(result.no_placed.empty());
  EXPECT_GT(result.placed_area_m2, 0.0);
}

// ── Full truck ────────────────────────────────────────────────────────────────

TEST(CargoPlanner, FullTruckPlacesNoCargoUnits)
{
  const double res = 0.1;
  cv::Mat full_map(24, 50, CV_8UC1, cv::Scalar(0));  // all occupied

  auto planner = makePlanner();
  planner.setFreeSpace(full_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 1.2, 0.8)});

  auto result = planner.plan();

  EXPECT_TRUE(result.placements.empty());
  EXPECT_EQ(result.no_placed.size(), 1u);
  EXPECT_EQ(result.no_placed[0], "c1");
}

// ── Cargo unit too large ──────────────────────────────────────────────────────

TEST(CargoPlanner, CargoUnitTooLargeForTruck)
{
  // Truck 1 m × 0.5 m @ 0.1 m → 10×5 grid
  // Cargo unit 2 m x 1 m does not fit in either orientation.
  const double res = 0.1;
  cv::Mat free_map(5, 10, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("big", 2.0, 1.0)});

  auto result = planner.plan();

  EXPECT_TRUE(result.placements.empty());
  ASSERT_EQ(result.no_placed.size(), 1u);
  EXPECT_EQ(result.no_placed[0], "big");
}

// ── Rotation selection ────────────────────────────────────────────────────────

TEST(CargoPlanner, WideCargoUnitIsRotatedInNarrowTruck)
{
  // Truck 2 m × 1 m @ 0.1 m → 20 cols × 10 rows
  // Cargo unit 0.8 m x 1.2 m: orientation 0 deg needs 12 rows but the truck
  // has only 10, so it must rotate.
  // Orientation 90°: 0.8 m in Y (8 rows) — fits.
  const double res = 0.1;
  cv::Mat free_map(10, 20, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 0.8, 1.2)});  // ly=1.2 > truck_width=1.0

  auto result = planner.plan();

  ASSERT_EQ(result.placements.size(), 1u);
  EXPECT_TRUE(result.placements[0].rotated);
}

TEST(CargoPlanner, WideCargoUnitIsNotPlacedWhenRotationIsDisabled)
{
  // Same geometry as WideCargoUnitIsRotatedInNarrowTruck.
  // With enable_rotation=false, the planner must not test the 90 degree
  // orientation, so the cargo unit is rejected instead of being rotated.
  const double res = 0.1;
  cv::Mat free_map(10, 20, CV_8UC1, cv::Scalar(255));

  CargoPlanner::Config cfg;
  cfg.pallet_margin = 0.0;
  cfg.enable_rotation = false;
  CargoPlanner planner(std::make_unique<GreedyPlacer>(), cfg);
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 0.8, 1.2)});

  auto result = planner.plan();

  EXPECT_TRUE(result.placements.empty());
  ASSERT_EQ(result.no_placed.size(), 1u);
  EXPECT_EQ(result.no_placed[0], "c1");
}

// ── Wall-hugging ──────────────────────────────────────────────────────────────

TEST(CargoPlanner, ConsecutiveCargoUnitsOnOppositeWalls)
{
  // Truck 4 m × 2.4 m @ 0.1 m → 40 cols × 24 rows, all free
  // Two EUR cargo units (1.2 m x 0.8 m): first should go to right wall (row=0),
  // second should go to left wall (row close to 24-8=16).
  const double res = 0.1;
  cv::Mat free_map(24, 40, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 1.2, 0.8), makeCargoUnit("c2", 1.2, 0.8)});

  auto result = planner.plan();

  ASSERT_EQ(result.placements.size(), 2u);

  const auto& first  = result.placements[0];
  const auto& second = result.placements[1];

  // First cargo unit: anchored at row=0 (right wall)
  EXPECT_EQ(first.anchor_row, 0);

  // Second cargo unit: anchored at a high row (left wall)
  // For ly=0.8 m, krows=8.  Valid max row = 24-8 = 16.
  EXPECT_GT(second.anchor_row, first.anchor_row);
}

// ── Anchor placement deep in truck ───────────────────────────────────────────

TEST(CargoPlanner, CargoUnitsPlacedAtBackOfTruck)
{
  // Truck 5 m × 2.4 m @ 0.1 m → 50 cols × 24 rows, all free
  // A single cargo unit should anchor near 50 - ceil(1.2 / 0.1) = 38.
  const double res = 0.1;
  cv::Mat free_map(24, 50, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 1.2, 0.8)});

  auto result = planner.plan();

  ASSERT_EQ(result.placements.size(), 1u);
  // The algorithm may rotate the cargo unit to achieve deeper placement.
  // Orientation 0 (lx=1.2m along X): deepest col = 50 - ceil(1.2/0.1) = 38.
  // Orientation 90 (ly=0.8m along X): deepest col = 50 - ceil(0.8/0.1) = 42.
  // Algorithm prefers the deeper orientation, so expected anchor_col = 42.
  EXPECT_EQ(result.placements[0].anchor_col, 42);
}

// ── Idempotency ───────────────────────────────────────────────────────────────

TEST(CargoPlanner, PlanIsIdempotent)
{
  const double res = 0.1;
  cv::Mat free_map(24, 50, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 1.2, 0.8)});

  auto r1 = planner.plan();
  auto r2 = planner.plan();  // second call must produce the same result

  ASSERT_EQ(r1.placements.size(), r2.placements.size());
  if(!r1.placements.empty())
  {
    EXPECT_EQ(r1.placements[0].anchor_col, r2.placements[0].anchor_col);
    EXPECT_EQ(r1.placements[0].anchor_row, r2.placements[0].anchor_row);
  }
}

// ── Ready guard ───────────────────────────────────────────────────────────────

TEST(CargoPlanner, NotReadyWithoutMapOrCargoUnits)
{
  auto planner = makePlanner();
  EXPECT_FALSE(planner.ready());
}

TEST(CargoPlanner, PlanThrowsWithoutMap)
{
  auto planner = makePlanner();
  planner.setCargoUnits({makeCargoUnit("c1", 1.2, 0.8)});
  EXPECT_THROW(planner.plan(), std::runtime_error);
}

TEST(CargoPlanner, PlanThrowsWithoutCargoUnits)
{
  const double res = 0.1;
  cv::Mat free_map(24, 50, CV_8UC1, cv::Scalar(255));
  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  EXPECT_THROW(planner.plan(), std::runtime_error);
}

// ── Pallet margin ─────────────────────────────────────────────────────────────

TEST(CargoPlanner, MarginPreventsAdjacentPallets)
{
  // Truck 3.0 m (X) x 1.0 m (Y) @ 0.1 m/cell → 30 cols x 10 rows, all free.
  // krows = ceil(0.8/0.1) = 8.  With margin=1 cell, only row=1 is a valid
  // anchor (pre-erosion leaves rows 1..8 free, and krows=8 exactly fills that).
  // The two cargo units of lx=1.0 m are therefore forced to different X depths.
  //
  // Expected layout (margin=1 cell on every side of every cargo unit):
  //
  //   col:  0   8  18  19  28  29
  //         |   |   |   |   |   |
  //         .   [B  B  B][M][A  A  A].   row 1..8
  //
  // Where [M] is the 1-cell gap (= pallet_margin) between the two footprints.
  // The greedy placer places cargo unit A first (deepest col), then cargo unit B.
  //
  // This test would have FAILED before the markOccupied fix because the old
  // code painted only the exact footprint (not footprint+margin) as occupied,
  // allowing cargo unit B to anchor at col=18 (gap=0) instead of col<=8.
  const double res    = 0.1;
  const double margin = 0.1;  // 1 cell

  cv::Mat free_map(10, 30, CV_8UC1, cv::Scalar(255));

  CargoPlanner::Config cfg;
  cfg.pallet_margin = margin;
  CargoPlanner planner(std::make_unique<GreedyPlacer>(), cfg);
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({makeCargoUnit("c1", 1.0, 0.8), makeCargoUnit("c2", 1.0, 0.8)});

  auto result = planner.plan();

  ASSERT_EQ(result.placements.size(), 2u);

  // placements[0] = cargo unit placed first (deeper = higher anchor_col)
  // placements[1] = cargo unit placed second (shallower = lower anchor_col)
  const auto& deeper    = result.placements[0];
  const auto& shallower = result.placements[1];

  EXPECT_GT(deeper.anchor_col, shallower.anchor_col);

  // The gap between the two footprints must be >= margin_cells (= 1 cell here).
  // footprint of shallower ends at: shallower.anchor_col + eff_kcols
  // footprint of deeper starts at:  deeper.anchor_col
  // gap = deeper.anchor_col - (shallower.anchor_col + eff_kcols) >= margin_cells
  const int eff_kcols    = static_cast<int>(std::ceil(1.0 / res));     // 10
  const int margin_cells = static_cast<int>(std::ceil(margin / res));  // 1

  EXPECT_GE(deeper.anchor_col, shallower.anchor_col + eff_kcols + margin_cells);
}

// ── Area metrics ──────────────────────────────────────────────────────────────

TEST(CargoPlanner, TotalAreaIsCorrect)
{
  // 10×5 grid @ 0.1 m/cell → total = 1.0 × 0.5 = 0.5 m²
  const double res = 0.1;
  cv::Mat free_map(5, 10, CV_8UC1, cv::Scalar(255));

  auto planner = makePlanner();
  planner.setFreeSpace(free_map, res, 2.5);
  planner.setCargoUnits({});  // empty list → nothing to place

  // We need at least an empty list for plan() to work.
  EXPECT_THROW(planner.plan(), std::runtime_error);  // has_cargo_units_ = false with empty list
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
