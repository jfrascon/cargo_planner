/**
 * @file test_grid_map_utils.cpp
 * @brief Unit tests for grid_map_utils conversion functions.
 */

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "cargo_planner/grid_map_utils.hpp"

using cargo_planner::matToOccupancyGrid;
using cargo_planner::occupancyGridToMat;

// Helper: build a minimal OccupancyGrid with given data
static nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, std::vector<int8_t> data, float resolution = 0.1f)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width      = static_cast<uint32_t>(width);
  g.info.height     = static_cast<uint32_t>(height);
  g.info.resolution = resolution;
  g.data            = std::move(data);
  return g;
}

// ── occupancyGridToMat ────────────────────────────────────────────────────────

TEST(GridMapUtils, FreeCellsMapToWhite)
{
  // 1×1 grid, cell value 0 (free) → should produce white pixel (255)
  auto grid   = makeGrid(1, 1, {0});
  cv::Mat mat = occupancyGridToMat(grid, 50);
  ASSERT_EQ(mat.rows, 1);
  ASSERT_EQ(mat.cols, 1);
  EXPECT_EQ(mat.at<uint8_t>(0, 0), 255);
}

TEST(GridMapUtils, OccupiedCellsMapToBlack)
{
  // 1×1 grid, cell value 100 (occupied) → should produce black pixel (0)
  auto grid   = makeGrid(1, 1, {100});
  cv::Mat mat = occupancyGridToMat(grid, 50);
  EXPECT_EQ(mat.at<uint8_t>(0, 0), 0);
}

TEST(GridMapUtils, UnknownCellMapsToBlack)
{
  // Cell value -1 (unknown) → treated as occupied
  nav_msgs::msg::OccupancyGrid g;
  g.info.width      = 1;
  g.info.height     = 1;
  g.info.resolution = 0.05f;
  g.data            = {-1};
  cv::Mat mat       = occupancyGridToMat(g, 50);
  EXPECT_EQ(mat.at<uint8_t>(0, 0), 0);
}

TEST(GridMapUtils, ThresholdApplied)
{
  // With threshold=80, value 79 is free and value 80 is occupied
  auto g_free = makeGrid(1, 1, {79});
  auto g_occ  = makeGrid(1, 1, {80});
  EXPECT_EQ(occupancyGridToMat(g_free, 80).at<uint8_t>(0, 0), 255);
  EXPECT_EQ(occupancyGridToMat(g_occ, 80).at<uint8_t>(0, 0), 0);
}

TEST(GridMapUtils, DimensionsMatch)
{
  // 3 (width) × 2 (height) grid → Mat must be rows=2, cols=3
  auto grid   = makeGrid(3, 2, {0, 0, 0, 0, 0, 0});
  cv::Mat mat = occupancyGridToMat(grid, 50);
  EXPECT_EQ(mat.rows, 2);
  EXPECT_EQ(mat.cols, 3);
}

TEST(GridMapUtils, RowMajorMapping)
{
  // 2×2 grid: top-left free, others occupied
  //   data = [ 0, 100, 100, 100 ]  (row0: cols 0,1; row1: cols 0,1)
  auto grid   = makeGrid(2, 2, {0, 100, 100, 100});
  cv::Mat mat = occupancyGridToMat(grid, 50);
  EXPECT_EQ(mat.at<uint8_t>(0, 0), 255);  // row=0, col=0 → free
  EXPECT_EQ(mat.at<uint8_t>(0, 1), 0);    // row=0, col=1 → occupied
  EXPECT_EQ(mat.at<uint8_t>(1, 0), 0);    // row=1, col=0 → occupied
}

TEST(GridMapUtils, InvalidDimensionsThrow)
{
  auto grid = makeGrid(0, 0, {});
  EXPECT_THROW(occupancyGridToMat(grid, 50), std::invalid_argument);
}

// ── matToOccupancyGrid ────────────────────────────────────────────────────────

TEST(GridMapUtils, RoundTripFreeCell)
{
  auto ref    = makeGrid(1, 1, {0});
  cv::Mat mat = occupancyGridToMat(ref, 50);  // 255
  auto out    = matToOccupancyGrid(mat, ref);
  EXPECT_EQ(out.data[0], 0);  // white → free (0)
}

TEST(GridMapUtils, RoundTripOccupiedCell)
{
  auto ref    = makeGrid(1, 1, {100});
  cv::Mat mat = occupancyGridToMat(ref, 50);  // 0
  auto out    = matToOccupancyGrid(mat, ref);
  EXPECT_EQ(out.data[0], 100);  // black → occupied (100)
}

TEST(GridMapUtils, MetadataPreserved)
{
  auto ref            = makeGrid(4, 3, std::vector<int8_t>(12, 0), 0.05f);
  ref.header.frame_id = "truck_frame";
  cv::Mat mat         = occupancyGridToMat(ref, 50);
  auto out            = matToOccupancyGrid(mat, ref);
  EXPECT_EQ(out.info.width, 4u);
  EXPECT_EQ(out.info.height, 3u);
  EXPECT_FLOAT_EQ(out.info.resolution, 0.05f);
  EXPECT_EQ(out.header.frame_id, "truck_frame");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
