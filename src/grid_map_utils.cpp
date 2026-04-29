/**
 * @file grid_map_utils.cpp
 * @brief Conversion between nav_msgs/OccupancyGrid and cv::Mat.
 */

#include "cargo_planner/grid_map_utils.hpp"

#include <stdexcept>

namespace cargo_planner
{

  cv::Mat occupancyGridToMat(const nav_msgs::msg::OccupancyGrid& grid, int occupied_threshold)
  {
    const int nx = static_cast<int>(grid.info.width);   // X cells (door → back wall)
    const int ny = static_cast<int>(grid.info.height);  // Y cells (right → left wall)

    if(nx <= 0 || ny <= 0)
    {
      throw std::invalid_argument("occupancyGridToMat: grid has zero dimensions");
    }

    if(static_cast<int>(grid.data.size()) != nx * ny)
    {
      throw std::invalid_argument("occupancyGridToMat: data size does not match width * height");
    }

    cv::Mat mat(ny, nx, CV_8UC1);

    for(int row = 0; row < ny; ++row)
    {
      for(int col = 0; col < nx; ++col)
      {
        // data is stored row-major: data[row * width + col]
        const int8_t cell = grid.data[static_cast<std::size_t>(row * nx + col)];

        // -1 (unknown) and values >= occupied_threshold are treated as occupied
        if(cell < 0 || cell >= occupied_threshold)
        {
          mat.at<uint8_t>(row, col) = 0;  // occupied → black
        }
        else
        {
          mat.at<uint8_t>(row, col) = 255;  // free → white
        }
      }
    }

    return mat;
  }

  nav_msgs::msg::OccupancyGrid matToOccupancyGrid(const cv::Mat& mat, const nav_msgs::msg::OccupancyGrid& ref)
  {
    if(mat.type() != CV_8UC1)
    {
      throw std::invalid_argument("matToOccupancyGrid: mat must be CV_8UC1");
    }

    if(mat.rows != static_cast<int>(ref.info.height) || mat.cols != static_cast<int>(ref.info.width))
    {
      throw std::invalid_argument("matToOccupancyGrid: mat dimensions do not match reference grid");
    }

    nav_msgs::msg::OccupancyGrid result;
    result.header = ref.header;
    result.info   = ref.info;
    result.data.resize(static_cast<std::size_t>(mat.rows * mat.cols));

    for(int row = 0; row < mat.rows; ++row)
    {
      for(int col = 0; col < mat.cols; ++col)
      {
        const int idx                              = row * mat.cols + col;
        result.data[static_cast<std::size_t>(idx)] = (mat.at<uint8_t>(row, col) == 0) ? 100 : 0;
      }
    }

    return result;
  }

}  // namespace cargo_planner
